/* This is the auto playing cards dealer machine */

/*-----( Import needed libraries )-----*/
#include "IRremote.h"
//#include "IRremoteInt.h"
#include "DFRobot_QMC5883.h"
#include "Wire.h"
#define DEBUGLEVEL 1
/*-----( Declare Constants )-----*/
//#define  500  500   // Delay before checking for another button / repeat
//int 11 = 11;           // pin 1 of IR 11 to Arduino digital pin 11
/*                             // NOTE: Other pins can be used, except pin 3 and 13
int 5 = 5;
int 4 = 4;
int 7 = 7;
int 8 = 8;
int 12 = 12;
int 10 = 10;
*/
int iPlayers = 4;           //set 4 default players
int iCardEach = 13;         //set 13 playing cards for each one as default
int iPlayerDistance = 90;   //set 90 degrees for players distance
unsigned long iDealCards = 2000;      //dealing one card with 2 seconds
unsigned long iIntervalofgotoplayer = 4000;
bool isSetPlayer = false;   //will set players number if true
bool isSetCardForEach = false;//will set card for each player if true
bool isDoneSetCardForEach = false;//completed set card for each player if true
/*-----( Declare objects )-----*/
IRrecv irrecv(11);         // create instance of 'irrecv'
decode_results results;            // create instance of 'decode_results'

/*-----( Declare Variables )-----*/
//int 6 = 6;       // pin 6 for get signal from the infrared obstacle avoidance module
DFRobot_QMC5883 compass;
//int dx = 0;
//int dy = 0;

void setup()   /*----( SETUP: RUNS ONCE )----*/
{
  if (DEBUGLEVEL){
    Serial.begin(9600);
    Serial.println("YourDuino.com IR Infrared Remote Control Kit V2");  
    Serial.println("IR Receiver Raw Data + Button Decode Test");
  }
  irrecv.enableIRIn(); // Start the 11
  pinMode(5, OUTPUT); //Motor A and Motor B drive the wheel of the card dealer car
  pinMode(4, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(12, OUTPUT); //Motor C is the drive wheel to deal the card
  pinMode(10, OUTPUT);
  pinMode(6, INPUT);
  digitalWrite(5, LOW);
  digitalWrite(4, LOW);
  digitalWrite(7, LOW);
  digitalWrite(8, LOW);
  digitalWrite(12, LOW);
  digitalWrite(10, LOW);
  //randomSeed(analogRead(9));  //random number for random card dealing
  InitCompass();
}/*--(end setup )---*/


void loop()   /*----( LOOP: RUNS CONSTANTLY )----*/
{
  if (irrecv.decode(&results)) // have we received an IR signal?
  {
    if (DEBUGLEVEL) Serial.println(results.value, HEX); // UN Comment to see raw values
    //TranslateIR();
    translateir();
    irrecv.resume(); // receive the next value
  }
  //delay(10);
}/* --(end main loop )-- */

void InitCompass()
{
    if (DEBUGLEVEL) Serial.println("Initialize QMC5883");
    while (!compass.begin()){
        if (DEBUGLEVEL) Serial.println("QMC5883 not found, check the connection!");
        delay(500);
    }
    if(compass.isHMC()){
        if (DEBUGLEVEL) Serial.println("Initialize HMC5883L");
        compass.setRange(HMC5883L_RANGE_1_3GA);
        compass.setMeasurementMode(HMC5883L_CONTINOUS);
        compass.setDataRate(HMC5883L_DATARATE_15HZ);
        compass.setSamples(HMC5883L_SAMPLES_8);
        if (DEBUGLEVEL) Serial.println("Initialize HMC5883L Completed");
    }
    else if(compass.isQMC()){
        if (DEBUGLEVEL) Serial.println("Initialize QMC5883");
        compass.setRange(QMC5883_RANGE_2GA);
        compass.setMeasurementMode(QMC5883_CONTINOUS); 
        compass.setDataRate(QMC5883_DATARATE_50HZ);
        compass.setSamples(QMC5883_SAMPLES_8);
        if (DEBUGLEVEL) Serial.println("Initialize QMC5883 Completed");
    }
}

/*
int GetHeading()
{
Vector norm = compass.readNormalize();
int heading = atan2(norm.YAxis - dy, norm.XAxis - dx) / PI * 180;
if (heading < 0) heading += 360;
return heading;
}
*/

int GetHeadingDegrees()
{
   Vector norm = compass.readNormalize();

  // Calculate heading
  float heading = atan2(norm.YAxis, norm.XAxis);
  if(DEBUGLEVEL == 1){
    Serial.print("YAxis is       :");
    //Serial println(norm.YAxis);
    Serial.print("XAxis is       :");
    //Serial println(norm.XAxis);
  }
  // Set declination angle on your location and fix heading
  // You can find your declination on: http://magnetic-declination.com/
  // (+) Positive or (-) for negative
  // For Bytom / Poland declination angle is 4'26E (positive), Shanghai magnetic declination angle is -5'57
  // Formula: (deg + (min / 60.0)) / (180 / PI); 
  float declinationAngle = (-(5 + (57.0 / 60.0))) / (180 / PI);
  heading += declinationAngle;
  if (heading <= 0){
    heading += 2 * PI;
  }
  if (heading > 2 * PI){
    heading -= 2 * PI;
  }
  // Convert to degrees
  float headingDegrees = heading * 180/PI; 
  if (DEBUGLEVEL){
    Serial.print(" Heading = ");
    Serial.print(heading);
    Serial.print(" Degress = ");
    Serial.println(headingDegrees);
  }
  ////delay(10);
  return headingDegrees;
}
/*
void CompassCalibrate()
{
  //analogWrite(9, 255);
  digitalWrite(5, HIGH); // start player motor
  digitalWrite(8, HIGH); // start player motor
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
  digitalWrite(5, LOW); // stop player motor
  digitalWrite(8, LOW); // stop player motor
  dx = (dx_max + dx_min) / 2;
  dy = (dy_max + dy_min) / 2;
  if (DEBUGLEVEL){
    Serial.print("dx = ");
    Serial.print(dx);
    Serial.print(", dy = ");
    Serial.println(dy);
  }
}
*/
int Player(int pos)
{
  pos = pos % 360;
  int prePos = GetHeadingDegrees() - pos;
  if (DEBUGLEVEL){
    Serial.print("parameter pos is:");
    Serial.println(pos);
    Serial.print("first prePos is:");
    Serial.println(prePos);
  }
  digitalWrite(5, HIGH); // start player motor
  if (DEBUGLEVEL) Serial.println("Motor A pin d set to high");
  digitalWrite(8, HIGH); // start player motor
  if (DEBUGLEVEL) Serial.println("Motor B pin r set to high");
  unsigned long t = millis();
  while ((millis() - t) < iIntervalofgotoplayer){
    if (DEBUGLEVEL) Serial.println("Enter while loop of Player");
    delay(5);
    int curPos = GetHeadingDegrees() - pos;
    if (DEBUGLEVEL){
      Serial.print("prePos is                :");
      Serial.println(prePos);
	    Serial.print("curPos is                  :");
      Serial.println(curPos);
    }
    if ((prePos <= 0) && (curPos >= 0)){
      digitalWrite(5, LOW); // stop player motor
      if (DEBUGLEVEL) Serial.println("Motor A pin d set to low");
      digitalWrite(8, LOW); // stop player motor
      if (DEBUGLEVEL) Serial.println("Motor B pin r set to low");
      if (DEBUGLEVEL) Serial.println("Turn to right angle");
      return 1;
    }
    prePos = curPos;
  }
  if (DEBUGLEVEL) Serial.println("Time exceed 4s");
  digitalWrite(5, LOW); // stop player motor
  if (DEBUGLEVEL) Serial.println("Motor A pin d set to low");
  digitalWrite(8, LOW); // stop player motor
  if (DEBUGLEVEL) Serial.println("Motor B pin r set to low");
  return 0;
}

int PlayCard()
{
  delay(5);
  digitalWrite(12, HIGH);
  if (DEBUGLEVEL) Serial.println("Motor C pin d set to high");
  unsigned long t = millis();
  while ((millis() - t) < iDealCards){
    if (digitalRead(6) == LOW){
	  if (DEBUGLEVEL) Serial.println("Low signal detected of the infrared obstacle avoidance");
      delay(70);
      digitalWrite(12, LOW);
	  if (DEBUGLEVEL) Serial.println("Motor C pin d set to low");
      return 1;
    }
  }
  digitalWrite(12, LOW);
  if (DEBUGLEVEL) {
    Serial.println("time exceeds iDealCards 2s");
    Serial.println("Motor C pin d set to low");
  }
  return 0;
}

void Deal()
{
  int pos = 60;
  for (int i = 0; i < iPlayers*iCardEach; i++){
    if (Player(pos) == 0) break;
    if (PlayCard() == 0) break;
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
    if(DEBUGLEVEL) Serial.println(number);
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
/*
void TranslateIR() // takes action based on IR code received // describing Car MP3 IR codes
{
switch (results.value)
{

  case 0xFFA25D:  
    if(DEBUGLEVEL) Serial.println(" CH-            "); 
    delay(50);
    digitalWrite(5, LOW);
    digitalWrite(4, LOW);
    digitalWrite(7, LOW);
    digitalWrite(8, LOW);
    digitalWrite(12, LOW);
    digitalWrite(10, LOW);
    delay(50);    
    break;

  case 0xFF629D:  
    if(DEBUGLEVEL) Serial.println(" CH             "); 
    break;

  case 0xFFE21D:  
    if(DEBUGLEVEL) Serial.println(" CH+            "); 
    iDealCards += 1000;
    if (iDealCards > 3000){
      iDealCards %= 3000;
      }
    break;
  
  case 0xFF22DD:
    if(DEBUGLEVEL) Serial.println(" PREV ");
    break;
  case 0xFF02FD:
    if(DEBUGLEVEL) Serial.println(" NEXT ");
    PlayCard();
    break;
  case 0xFFC23D:
    if(DEBUGLEVEL) Serial.println(" PLAY/PAUSE ");
    delay(500);
    Deal();
    delay(500);
    break;

  case 0xFFE01F:  
    if(DEBUGLEVEL) Serial.println(" VOL- "); 
    isSetPlayer = true;
    break;

  case 0xFFA857:  
    if(DEBUGLEVEL) Serial.println(" VOL+ "); 
    isSetCardForEach = true;
    isDoneSetCardForEach = true;
    break;

  case 0xFF906F:
    if(DEBUGLEVEL) Serial.println(" EQ ");
    CompassCalibrate();
    break;

  case 0xFF6897:  
    if(DEBUGLEVEL) Serial.println(" 0 "); 
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
    if(DEBUGLEVEL) Serial.println(" 100+ "); 
    break;

  case 0xFFB04F:  
    if(DEBUGLEVEL) Serial.println(" 200+ "); 
    iPlayerDistance += 10;
    if (iPlayerDistance > 90)
      iPlayerDistance = 30;
    break;

  case 0xFF30CF:
    if(DEBUGLEVEL) Serial.println(" 1 ");
    NumberProcessing(1);
    break;
  case 0xFF18E7:
    if(DEBUGLEVEL) Serial.println(" 2 ");
    NumberProcessing(2);
    break;
  case 0xFF7A85:
    if(DEBUGLEVEL) Serial.println(" 3 ");
    NumberProcessing(3);
    break;
  case 0xFF10EF:
    if(DEBUGLEVEL) Serial.println(" 4 ");
    NumberProcessing(4);
    break;

  case 0xFF38C7:  
    if(DEBUGLEVEL) Serial.println(" 5 "); 
    NumberProcessing(5);
    break;

  case 0xFF5AA5:  
    if(DEBUGLEVEL) Serial.println(" 6 "); 
    NumberProcessing(6);
    break;

  case 0xFF42BD:  
    if(DEBUGLEVEL) Serial.println(" 7 "); 
    NumberProcessing(7);
    break;

  case 0xFF4AB5:  
    if(DEBUGLEVEL) Serial.println(" 8 "); 
    NumberProcessing(8);
    break;

  case 0xFF52AD:  
    if(DEBUGLEVEL) Serial.println(" 9 "); 
    NumberProcessing(9);
    break;

  default:
    if(DEBUGLEVEL) Serial.println(" other button ");
    break;
}
delay(500);
} //END TranslateIR
*/

void translateir() // takes action based on IR code received // describing Car MP3 IR codes
{
switch (results.value)
{

  case 0xC544EDC4:  
    if(DEBUGLEVEL) Serial.println(" |<            "); 
    delay(50);
    digitalWrite(5, LOW);
    digitalWrite(4, LOW);
    digitalWrite(7, LOW);
    digitalWrite(8, LOW);
    digitalWrite(12, LOW);
    digitalWrite(10, LOW);
    delay(50);    
    break;

  case 0x9962293B:  
    if(DEBUGLEVEL) Serial.println(" -^             "); 
    break;

  case 0x555E3747:  
    if(DEBUGLEVEL) Serial.println(" >|            "); 
    iDealCards += 1000;
    if (iDealCards > 3000){
      iDealCards %= 3000;
      }
    break;
  
  case 0xD99766D5:
    if(DEBUGLEVEL) Serial.println(" << ");
    break;
  case 0xAC4BDD79:
    if(DEBUGLEVEL) Serial.println(" >> ");
    PlayCard();
    break;
  case 0x3778AFF2:
    if(DEBUGLEVEL) Serial.println(" OK PLAY/PAUSE ");
    delay(500);
    Deal();
    delay(500);
    break;

  case 0xB1DD4311:  
    if(DEBUGLEVEL) Serial.println(" VOL- "); 
    isSetPlayer = true;
    break;

  case 0xB6A89DF3:  
    if(DEBUGLEVEL) Serial.println(" VOL+ "); 
    isSetCardForEach = true;
    isDoneSetCardForEach = true;
    break;

  case 0xBD4971EE:
    if(DEBUGLEVEL) Serial.println(" INFO ");
    //CompassCalibrate();
    break;

  case 0xC1EE7333:
    if(DEBUGLEVEL) Serial.println(" 0 "); 
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

  case 0xFE5F5ABA:  
    if(DEBUGLEVEL) Serial.println(" ^ "); 
    break;

  case 0xA86901ED:  
    if(DEBUGLEVEL) Serial.println(" V "); 
    iPlayerDistance += 10;
    if (iPlayerDistance > 90)
      iPlayerDistance = 30;
    break;

  case 0xA877CD25:
    if(DEBUGLEVEL) Serial.println(" 1 ");
    NumberProcessing(1);
    break;
  case 0x4FDC3242:
    if(DEBUGLEVEL) Serial.println(" 2 ");
    NumberProcessing(2);
    break;
  case 0x9FF90E1F:
    if(DEBUGLEVEL) Serial.println(" 3 ");
    NumberProcessing(3);
    break;
  case 0xCD39E45D:
    if(DEBUGLEVEL) Serial.println(" 4 ");
    NumberProcessing(4);
    break;

  case 0x20DB57B9:  
    if(DEBUGLEVEL) Serial.println(" 5 "); 
    NumberProcessing(5);
    break;

  case 0x5D8B2362:  
    if(DEBUGLEVEL) Serial.println(" 6 "); 
    NumberProcessing(6);
    break;

  case 0xC7CA768D:  
    if(DEBUGLEVEL) Serial.println(" 7 "); 
    NumberProcessing(7);
    break;

  case 0xD3969CC3:  
    if(DEBUGLEVEL) Serial.println(" 8 "); 
    NumberProcessing(8);
    break;

  case 0xD6284F44:  
    if(DEBUGLEVEL) Serial.println(" 9 "); 
    NumberProcessing(9);
    break;

  default:
    if(DEBUGLEVEL) Serial.println(" other button ");
    break;
}
delay(500);
} //END translateir
/* ( THE END ) */


