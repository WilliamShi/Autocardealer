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

/*-----( Declare Constants )-----*/
#define  REPEAT_DELAY  500   // Delay before checking for another button / repeat
int receiver = 11; // pin 1 of IR receiver to Arduino digital pin 11
                   // NOTE: Other pins can be used, except pin 3 and 13
int MotorApin_d = 2;
int MotorApin_r = 4;
int MotorBpin_d = 7;
int MotorBpin_r = 8;
int MotorCpin_d = 12;
int MotorCpin_r = 10;
int Players = 4;
int CardEach = 13;
bool isSetPlayer = false;
bool isSetCardEach = false;
bool isCompleteSetCard = false;
/*-----( Declare objects )-----*/
IRrecv irrecv(receiver);           // create instance of 'irrecv'
decode_results results;            // create instance of 'decode_results'

/*-----( Declare Variables )-----*/
//int  ButtonValue;  // 0..9,100,200, top 9 buttons encoded 10..18, -1 Bad Code
int inPin = 3;
//IRrecv irrecv(recvPin);
//decode_results results;
DFRobot_QMC5883 compass;
int dx = 0;
int dy = 0;

void setup()   /*----( SETUP: RUNS ONCE )----*/
{
  Serial.begin(9600);
  Serial.println("YourDuino.com IR Infrared Remote Control Kit V2");  
  Serial.println("IR Receiver Raw Data + Button Decode Test");
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
  InitCompass();
  
}/*--(end setup )---*/


void loop()   /*----( LOOP: RUNS CONSTANTLY )----*/
{
  if (irrecv.decode(&results)) // have we received an IR signal?

  {
    Serial.println(results.value, HEX); // UN Comment to see raw values
    //ButtonValue = translateIR(); 
    TranslateIR();
    //Serial.println(ButtonValue, DEC);
    delay(REPEAT_DELAY);    // Adjust for repeat / lockout time
    irrecv.resume(); // receive the next value

  }  
}/* --(end main loop )-- */


void InitCompass()
{
Serial.println("Initialize QMC5883");
while (!compass.begin())
{
Serial.println("QMC5883 not found, check the connection!");
delay(500);
}
    if(compass.isHMC()){
        Serial.println("Initialize HMC5883L");
        compass.setRange(HMC5883L_RANGE_1_3GA);
        compass.setMeasurementMode(HMC5883L_CONTINOUS);
        compass.setDataRate(HMC5883L_DATARATE_15HZ);
        compass.setSamples(HMC5883L_SAMPLES_8);
    }
   else if(compass.isQMC()){
        Serial.println("Initialize QMC5883");
        compass.setRange(QMC5883_RANGE_2GA);
        compass.setMeasurementMode(QMC5883_CONTINOUS); 
        compass.setDataRate(QMC5883_DATARATE_50HZ);
        compass.setSamples(QMC5883_SAMPLES_8);
   }

//compass.setRange(HMC5883L_RANGE_1_3GA);
//compass.setMeasurementMode(HMC5883L_CONTINOUS);
//compass.setDataRate(HMC5883L_DATARATE_75HZ);
//compass.setSamples(HMC5883L_SAMPLES_4);
}

int GetHeading()
{
Vector norm = compass.readNormalize();
int heading = atan2(norm.YAxis - dy, norm.XAxis - dx) / PI * 180;
if (heading < 0) heading += 360;
return heading;
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

while (millis() - t < 4000)
{
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

Serial.print("dx = ");
Serial.print(dx);
Serial.print(", dy = ");
Serial.println(dy);
}


int Player(int pos)
{
//analogWrite(9, 150);
digitalWrite(MotorApin_d, HIGH); // start player motor
digitalWrite(MotorBpin_r, HIGH); // start player motor
int prePos = GetHeading() - pos;
Serial.print("Pos is:");
Serial.println(pos);
Serial.print("prePos is:");
Serial.println(prePos);
unsigned long t = millis();
while (millis() - t < 4000)
{
delay(5);
int curPos = GetHeading() - pos;
Serial.print("curPos is:");
Serial.println(curPos);
if (prePos <= 0 && curPos >= 0)
{
digitalWrite(MotorApin_d, LOW); // stop player motor
digitalWrite(MotorBpin_r, LOW); // stop player motor
return 1;
}
prePos = curPos;
}
digitalWrite(MotorApin_d, LOW); // stop player motor
digitalWrite(MotorBpin_r, LOW); // stop player motor
return 0;
}

int PlayCard()
{
delay(5);
digitalWrite(MotorCpin_d, HIGH);
unsigned long t = millis();
while (millis() - t < 2000)
{
if (digitalRead(inPin) == LOW)
{
delay(70);
digitalWrite(MotorCpin_d, LOW);
return 1;
}
}
digitalWrite(MotorCpin_d, LOW);
return 0;
}

void Deal()
{
int pos = 60;
for (int i = 0; i < 52; i++)
{
if (Player(pos) == 0) break;
if (PlayCard() == 0) break;
pos = (pos + 90) % 360;
delay(200);
}
}
/*-----( Declare User-written Functions )-----*/
/*
int translateIR() // returns value of "Car MP3 remote IR code received

{

 switch(results.value)

  {

  case 0xFFA25D:  
    digitalWrite(MotorApin_d, HIGH);
    delay(2000);
    digitalWrite(MotorApin_d, LOW);
    return 10;  // CH-
    break;

  case 0xFF629D:  
    digitalWrite(MotorBpin_r, HIGH);
    delay(2000);
    digitalWrite(MotorBpin_r, LOW);
    return 11; // CH
    break;

  case 0xFFE21D:  
    digitalWrite(MotorApin_d, LOW);
    Serial.println("MotorApin_d set as low");
    return 12; // CH+
    break;

  case 0xFF22DD:  
    digitalWrite(MotorBpin_r, LOW);
    Serial.println("MotorBpin_r set as low");
    return 13; // PREV
    break;

  case 0xFF02FD:  
    return 14; // NEXT
    break;

  case 0xFFC23D:  
    PlayCard();
    return 15; //  PLAY/PAUSE     
    break;

  case 0xFFE01F:  
    return 16; // VOL-
    break;

  case 0xFFA857:  
    return 17; // VOL+ 
    break;

  case 0xFF906F:  
    return 18; // EQ 
    break;

  case 0xFF6897:  
    return 0; // ZERO
    break;

  case 0xFF9867:  
    return 100; // 100+ 
    break;

  case 0xFFB04F:  
    return 200; // 200+ 
    break;

  case 0xFF30CF:  
    return 1;  // 1 etc.. to 9
    break;

  case 0xFF18E7:  
    return 2; 
    break;

  case 0xFF7A85:  
    return 3; 
    break;

  case 0xFF10EF:  
    return 4;  
    break;

  case 0xFF38C7:  
    return 5; 
    break;

  case 0xFF5AA5:  
    return 6; 
    break;

  case 0xFF42BD:  
    return 7; 
    break;

  case 0xFF4AB5:  
    return 8;  
    break;

  case 0xFF52AD:  
    return 9; // 9 
    break;

  case 0xFFFFFFFF:  
    return -2; // REPEAT: Button Held down longer than 
    break;
  default: 
    break;//return -1; // Other Button  / Bad Code
  }
  delay(100); //END case
} //END translateIR
*/

void TranslateIR() // takes action based on IR code received // describing Car MP3 IR codes
{
switch (results.value)
{

  case 0xFFA25D:  
    Serial.println(" CH-            "); 
    break;

  case 0xFF629D:  
    Serial.println(" CH             "); 
    break;

  case 0xFFE21D:  
    Serial.println(" CH+            "); 
    break;
  
  case 0xFF22DD:
    Serial.println(" PREV ");
    break;
  case 0xFF02FD:
    Serial.println(" NEXT ");
    PlayCard();
    break;
  case 0xFFC23D:
    Serial.println(" PLAY/PAUSE ");
    Deal();
    break;

  case 0xFFE01F:  
    Serial.println(" VOL-           "); 
    isSetPlayer = true;
    break;

  case 0xFFA857:  
    Serial.println(" VOL+           "); 
    iSetCardEach = true;
    isCompleteCardEach = true;
    break;

  case 0xFF906F:
    Serial.println(" EQ ");
    CompassCalibrate();
    break;

  case 0xFF6897:  
    Serial.println(" 0              "); 
    if (isSetPlayer)
        {
        Players = 4;
        CardEach = 13;
        isSetPlayer = false;
        }
    else
      //compass.Turn(EAST); 
    break;

  case 0xFF30CF:
    Serial.println(" 1 ");
    if (isSetPlayer){
      Players = 1;
      isSetPlayer = false;
      }
    else if (isSetCardEach){
      if (isCompleteCardEach)
        CardEach = 1;
        isCompleteCardEach = false;
      else
      {
        CardEach = CardEach*10 + 1;
        isSetCardEach = false;
      }
      }
    else
      Player(60);
    break;
  case 0xFF18E7:
    Serial.println(" 2 ");
    if (isSetPlayer){
      Players = 2;
      isSetPlayer = false;
      }
    else if (isSetCardEach){
      if (isCompleteCardEach)
        CardEach = 2;
        isCompleteCardEach = false;
      else
      {
        CardEach = CardEach*10 + 2;
        isSetCardEach = false;
      }
      }
    else
      Player(150);
    break;
  case 0xFF7A85:
    Serial.println(" 3 ");
    if (isSetPlayer){
      Players = 3;
      isSetPlayer = false;
      }
    else if (isSetCardEach){
      if (isCompleteCardEach)
        CardEach = 3;
        isCompleteCardEach = false;
      else
      {
        CardEach = CardEach*10 + 3;
        isSetCardEach = false;
      }
      }
    else
      Player(240);
    break;
  case 0xFF10EF:
    Serial.println(" 4 ");
    if (isSetPlayer){
      Players = 4;
      isSetPlayer = false;
      }
    else if (isSetCardEach){
      if (isCompleteCardEach)
        CardEach = 4;
        isCompleteCardEach = false;
      else
      {
        CardEach = CardEach*10 + 4;
        isSetCardEach = false;
      }
      }
    else
      Player(330);
    break;
  default:
  //Serial.println(" other button ");
    break;
}
delay(100);
} //END TranslateIR


/* ( THE END ) */

