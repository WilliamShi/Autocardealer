/* YourDuino.com Example Software Sketch
 IR Remote Kit Test: Returns numeric value for button pressed
 Uses YourDuino.com IR Infrared Remote Control Kit V2
 http://arduino-direct.com/sunshop/index.php?l=product_detail&p=153
 based on code by Ken Shirriff - http://arcfn.com
 terry@yourduino.com */

/*-----( Import needed libraries )-----*/
#include "IRremote.h"
#include "IRremoteInt.h"
#include "DFRobot_QMC5883.h"
#include "Wire.h"
#define DEBUG 1
/*-----( Declare Constants )-----*/
#define  REPEAT_DELAY  500   // Delay before checking for another button / repeat
int receiver = 11;           // pin 1 of IR receiver to Arduino digital pin 11
                             // NOTE: Other pins can be used, except pin 3 and 13
int MotorApin_d = 2;
int MotorApin_r = 4;
int MotorBpin_d = 7;
int MotorBpin_r = 8;
int MotorCpin_d = 12;
int MotorCpin_r = 10;
int iPlayers = 4;           //set 4 default players
int iCardEach = 13;         //set 13 playing cards for each one as default
int iPlayerDistance = 90;   //set 90 degrees for players distance
int iDealCards = 2000;      //dealing one card with 2 seconds
bool isSetPlayer = false;   //will set players number if true
bool isSetCardForEach = false;//will set card for each player if true
bool isDoneSetCardForEach = false;//completed set card for each player if true
/*-----( Declare objects )-----*/
IRrecv irrecv(receiver);           // create instance of 'irrecv'
decode_results results;            // create instance of 'decode_results'

/*-----( Declare Variables )-----*/
int inPin = 3;       // pin 3 for get signal from the infrared obstacle avoidance module
DFRobot_QMC5883 compass;
int dx = 0;
int dy = 0;

void setup()   /*----( SETUP: RUNS ONCE )----*/
{
  if (DEBUG){
    Serial.begin(9600);
    Serial.println("YourDuino.com IR Infrared Remote Control Kit V2");  
    Serial.println("IR Receiver Raw Data + Button Decode Test");
  }
  irrecv.enableIRIn(); // Start the receiver
  pinMode(MotorApin_d, OUTPUT); //Motor A and Motor B drive the wheel of the card dealer car
  pinMode(MotorApin_r, OUTPUT);
  pinMode(MotorBpin_d, OUTPUT);
  pinMode(MotorBpin_r, OUTPUT);
  pinMode(MotorCpin_d, OUTPUT); //Motor C is the drive wheel to deal the card
  pinMode(MotorCpin_r, OUTPUT);
  pinMode(inPin, INPUT);
  digitalWrite(MotorApin_d, LOW);
  digitalWrite(MotorApin_r, LOW);
  digitalWrite(MotorBpin_d, LOW);
  digitalWrite(MotorBpin_r, LOW);
  digitalWrite(MotorCpin_d, LOW);
  digitalWrite(MotorCpin_r, LOW);
  //randomSeed(analogRead(9));  //random number for random card dealing
  InitCompass();
}/*--(end setup )---*/


void loop()   /*----( LOOP: RUNS CONSTANTLY )----*/
{
  if (irrecv.decode(&results)) // have we received an IR signal?
  {
    if (DEBUG)
      Serial.println(results.value, HEX); // UN Comment to see raw values
    TranslateIR();
    delay(REPEAT_DELAY);    // Adjust for repeat / lockout time
    irrecv.resume(); // receive the next value
  }  
}/* --(end main loop )-- */

void InitCompass()
{
    if (DEBUG) Serial.println("Initialize QMC5883");
    while (!compass.begin()){
        if (DEBUG) Serial.println("QMC5883 not found, check the connection!");
        delay(500);
    }
    if(compass.isHMC()){
        if (DEBUG) Serial.println("Initialize HMC5883L");
        compass.setRange(HMC5883L_RANGE_1_3GA);
        compass.setMeasurementMode(HMC5883L_CONTINOUS);
        compass.setDataRate(HMC5883L_DATARATE_15HZ);
        compass.setSamples(HMC5883L_SAMPLES_8);
        if (DEBUG) Serial.println("Initialize HMC5883L Completed");
    }
    else if(compass.isQMC()){
        if (DEBUG) Serial.println("Initialize QMC5883");
        compass.setRange(QMC5883_RANGE_2GA);
        compass.setMeasurementMode(QMC5883_CONTINOUS); 
        compass.setDataRate(QMC5883_DATARATE_50HZ);
        compass.setSamples(QMC5883_SAMPLES_8);
        if (DEBUG) Serial.println("Initialize QMC5883 Completed");
    }
}

int GetHeadingDegrees()
{
  Vector norm = compass.readNormalize();

  // Calculate heading
  float heading = atan2(norm.YAxis, norm.XAxis);

  // Set declination angle on your location and fix heading
  // You can find your declination on: http://magnetic-declination.com/
  // (+) Positive or (-) for negative
  // For Bytom / Poland declination angle is 4'26E (positive)
  // Formula: (deg + (min / 60.0)) / (180 / PI);
  float declinationAngle = (4.0 + (26.0 / 60.0)) / (180 / PI);
  heading += declinationAngle;

  // Correct for heading < 0deg and heading > 360deg
  if (heading < 0){
    heading += 2 * PI;
  }

  if (heading > 2 * PI){
    heading -= 2 * PI;
  }
  // Convert to degrees
  float headingDegrees = heading * 180/PI; 
  if (DEBUG){
    Serial.print(" Heading = ");
    Serial.print(heading);
    Serial.print(" Degress = ");
    Serial.println(headingDegrees);
  }
  delay(100);
  return headingDegrees;
}

void CompassCalibrate()
{
  //analogWrite(9, 255);
  digitalWrite(MotorApin_d, HIGH); // start player motor
  digitalWrite(MotorBpin_r, HIGH); // start player motor
  int dx_max = -10000;
  int dx_min = 10000;
  int dy_max = -10000;
  int dy_min = 10000;
  unsigned long t = millis();
  
  while (millis() - t < 4000){
    Vector norm = compass.readNormalize();
    if (norm.XAxis > dx_max) dx_max = norm.XAxis;
    if (norm.XAxis < dx_min) dx_min = norm.XAxis;
    if (norm.YAxis > dy_max) dy_max = norm.YAxis;
    if (norm.YAxis < dy_min) dy_min = norm.YAxis;
    delay(5);
  }
  digitalWrite(MotorApin_d, LOW); // stop player motor
  digitalWrite(MotorBpin_r, LOW); // stop player motor
  dx = (dx_max + dx_min) / 2;
  dy = (dy_max + dy_min) / 2;
  if (DEBUG){
    Serial.print("dx = ");
    Serial.print(dx);
    Serial.print(", dy = ");
    Serial.println(dy);
  }
}

int Player(int pos)
{
  pos = pos % 360;
  digitalWrite(MotorApin_d, HIGH); // start player motor
  if (DEBUG) Serial.println("Motor A pin d set to high");
  digitalWrite(MotorBpin_r, HIGH); // start player motor
  if (DEBUG) Serial.println("Motor B pin r set to high");
  int prePos = GetHeadingDegrees() - pos;
  if (DEBUG){
    Serial.print("Pos is:");
    Serial.println(pos);
    Serial.print("prePos is:");
    Serial.println(prePos);
  }
  unsigned long t = millis();
  while (millis() - t < 4000){
    delay(5);
    int curPos = GetHeadingDegrees() - pos;
    if (DEBUG){
      Serial.print("curPos is:");
      Serial.println(curPos);
    }
    if (prePos <= 0 && curPos >= 0){
      delay(50);
      digitalWrite(MotorApin_d, LOW); // stop player motor
      if (DEBUG) Serial.println("Motor A pin d set to low");
      digitalWrite(MotorBpin_r, LOW); // stop player motor
      if (DEBUG) Serial.println("Motor B pin r set to low");
	  delay(50);
      return 1;
    }
    prePos = curPos;
  }
  delay(50);
  digitalWrite(MotorApin_d, LOW); // stop player motor
  if (DEBUG) Serial.println("Motor A pin d set to low");
  digitalWrite(MotorBpin_r, LOW); // stop player motor
  if (DEBUG) Serial.println("Motor B pin r set to low");
  delay(50);
  return 0;
}

int PlayCard()
{
  delay(5);
  digitalWrite(MotorCpin_d, HIGH);
  if (DEBUG) Serial.println("Motor C pin d set to high");
  unsigned long t = millis();
  while (millis() - t < iDealCards){
    if (digitalRead(inPin) == LOW){
	  if (DEBUG) Serial.println("Low signal detected of the infrared obstacle avoidance");
      delay(70);
      digitalWrite(MotorCpin_d, LOW);
	  if (DEBUG) Serial.println("Motor C pin d set to low");
      return 1;
    }
  }
  digitalWrite(MotorCpin_d, LOW);
  if (DEBUG) Serial.println("Motor C pin d set to low");
  return 0;
}

void Deal()
{
  int pos = 60;
  for (int i = 0; i < iPlayers*iCardEach; i++){
    if (Player(pos) == 0) break;
    delay(1000);
    if (PlayCard() == 0) break;
    delay(1000);
    pos = (pos + iPlayerDistance) % 360;
    delay(200);
  }
}

/* Press number it has 3 cases: 
1. After VOL-, set player numbers
2. After VOL+, set cards for each player
3. Just the number, turn to the player 
*/
void NumberProcessing(int number){
    if(DEBUG) Serial.println(number);
    if (isSetPlayer){
      iPlayers = number;
      isSetPlayer = false;
      }
    else if (isSetCardForEach){
      if (isDoneSetCardForEach){
        iCardEach = number;
        isDoneSetCardForEach = false;
      }
      else
      {
        iCardEach = iCardEach*10 + number;
        isSetCardForEach = false;
      }
      }
    else
      Player((number-1)*iPlayerDistance+60);
  }
/*-----( Declare User-written Functions )-----*/

void TranslateIR() // takes action based on IR code received // describing Car MP3 IR codes
{
switch (results.value)
{

  case 0xFFA25D:  
    if(DEBUG) Serial.println(" CH-            "); 
    delay(50);
    digitalWrite(MotorApin_d, LOW);
    digitalWrite(MotorApin_r, LOW);
    digitalWrite(MotorBpin_d, LOW);
    digitalWrite(MotorBpin_r, LOW);
    digitalWrite(MotorCpin_d, LOW);
    digitalWrite(MotorCpin_r, LOW);
    delay(50);    
    break;

  case 0xFF629D:  
    if(DEBUG) Serial.println(" CH             "); 
    break;

  case 0xFFE21D:  
    if(DEBUG) Serial.println(" CH+            "); 
    iDealCards += 1000;
    if (iDealCards > 3000){
      iDealCards %= 3000;
      }
    break;
  
  case 0xFF22DD:
    if(DEBUG) Serial.println(" PREV ");
    break;
  case 0xFF02FD:
    if(DEBUG) Serial.println(" NEXT ");
    PlayCard();
    break;
  case 0xFFC23D:
    if(DEBUG) Serial.println(" PLAY/PAUSE ");
    delay(500);
    Deal();
    delay(500);
    break;

  case 0xFFE01F:  
    if(DEBUG) Serial.println(" VOL- "); 
    isSetPlayer = true;
    break;

  case 0xFFA857:  
    if(DEBUG) Serial.println(" VOL+ "); 
    isSetCardForEach = true;
    isDoneSetCardForEach = true;
    break;

  case 0xFF906F:
    if(DEBUG) Serial.println(" EQ ");
    CompassCalibrate();
    break;

  case 0xFF6897:  
    if(DEBUG) Serial.println(" 0 "); 
    if (isSetPlayer)
        {
        iPlayers = 4;           //set 4 default players
        iCardEach = 13;         //set 13 playing cards for each one as default
        iPlayerDistance = 90;   //set 90 degrees for players distance
        iDealCards = 2000;      //dealing one card with 2 seconds
        isSetPlayer = false;   //will set players number if true
        isSetCardForEach = false;//will set card for each player if true
        isDoneSetCardForEach = false;//completed set card for each player if true
        }
    else
        {
        iPlayers = 4;           //set 4 default players
        iCardEach = 13;         //set 13 playing cards for each one as default
        iPlayerDistance = 90;   //set 90 degrees for players distance
        iDealCards = 2000;      //dealing one card with 2 seconds
        isSetPlayer = false;   //will set players number if true
        isSetCardForEach = false;//will set card for each player if true
        isDoneSetCardForEach = false;//completed set card for each player if true
        //compass.Turn(EAST); 
        }
    break;

  case 0xFF9867:  
    if(DEBUG) Serial.println(" 100+ "); 
    break;

  case 0xFFB04F:  
    if(DEBUG) Serial.println(" 200+ "); 
    iPlayerDistance += 10;
    if (iPlayerDistance > 90)
      iPlayerDistance = 30;
    break;

  case 0xFF30CF:
    if(DEBUG) Serial.println(" 1 ");
    NumberProcessing(1);
    break;
  case 0xFF18E7:
    if(DEBUG) Serial.println(" 2 ");
    NumberProcessing(2);
    break;
  case 0xFF7A85:
    if(DEBUG) Serial.println(" 3 ");
    NumberProcessing(3);
    break;
  case 0xFF10EF:
    if(DEBUG) Serial.println(" 4 ");
    NumberProcessing(4);
    break;

  case 0xFF38C7:  
    if(DEBUG) Serial.println(" 5 "); 
    NumberProcessing(5);
    break;

  case 0xFF5AA5:  
    if(DEBUG) Serial.println(" 6 "); 
    NumberProcessing(6);
    break;

  case 0xFF42BD:  
    if(DEBUG) Serial.println(" 7 "); 
    NumberProcessing(7);
    break;

  case 0xFF4AB5:  
    if(DEBUG) Serial.println(" 8 "); 
    NumberProcessing(8);
    break;

  case 0xFF52AD:  
    if(DEBUG) Serial.println(" 9 "); 
    NumberProcessing(9);
    break;

  default:
    if(DEBUG) Serial.println(" other button ");
    break;
}
delay(100);
} //END TranslateIR
/* ( THE END ) */


