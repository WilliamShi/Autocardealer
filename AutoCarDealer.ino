/* YourDuino.com Example Software Sketch
 IR Remote Kit Test: Returns numeric value for button pressed
 Uses YourDuino.com IR Infrared Remote Control Kit V2
 http://arduino-direct.com/sunshop/index.php?l=product_detail&p=153
 based on code by Ken Shirriff - http://arcfn.com
 terry@yourduino.com */

/*-----( Import needed libraries )-----*/
#include "IRremote.h"
#include "IRremoteInt.h"
#include "HMC5883L.h"

/*-----( Declare Constants )-----*/
#define  REPEAT_DELAY  500   // Delay before checking for another button / repeat
int receiver = 11; // pin 1 of IR receiver to Arduino digital pin 11
                   // NOTE: Other pins can be used, except pin 3 and 13
                  
/*-----( Declare objects )-----*/
IRrecv irrecv(receiver);           // create instance of 'irrecv'
decode_results results;            // create instance of 'decode_results'

/*-----( Declare Variables )-----*/
int  ButtonValue;  // 0..9,100,200, top 9 buttons encoded 10..18, -1 Bad Code
int inPin = 7;
//IRrecv irrecv(recvPin);
//decode_results results;
HMC5883L compass;
int dx = 0;
int dy = 0;

void setup()   /*----( SETUP: RUNS ONCE )----*/
{
  Serial.begin(9600);
  Serial.println("YourDuino.com IR Infrared Remote Control Kit V2");  
  Serial.println("IR Receiver Raw Data + Button Decode Test");
  irrecv.enableIRIn(); // Start the receiver
  pinMode(2, OUTPUT); //play card PWM
  pinMode(3, OUTPUT); //play card
  pinMode(4, OUTPUT); //play card
  pinMode(5, OUTPUT); //play card
  pinMode(inPin, INPUT); //play card
  digitalWrite(2, LOW);
  digitalWrite(3, LOW);
  digitalWrite(4, LOW);
  digitalWrite(5, LOW);
  
}/*--(end setup )---*/


void loop()   /*----( LOOP: RUNS CONSTANTLY )----*/
{
  if (irrecv.decode(&results)) // have we received an IR signal?

  {
//    Serial.println(results.value, HEX); // UN Comment to see raw values
    ButtonValue = translateIR(); 
    Serial.println(ButtonValue, DEC);
    delay(REPEAT_DELAY);    // Adjust for repeat / lockout time
    irrecv.resume(); // receive the next value

  }  
}/* --(end main loop )-- */


void InitCompass()
{
Serial.println("Initialize HMC5883L");
while (!compass.begin())
{
Serial.println("HMC5883L not found, check the connection!");
delay(500);
}
compass.setRange(HMC5883L_RANGE_1_3GA);
compass.setMeasurementMode(HMC5883L_CONTINOUS);
compass.setDataRate(HMC5883L_DATARATE_75HZ);
compass.setSamples(HMC5883L_SAMPLES_4);
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
analogWrite(9, 255);
digitalWrite(8, 1); // start player motor
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

digitalWrite(8, 0); // stop player motor
dx = (dx_max + dx_min) / 2;
dy = (dy_max + dy_min) / 2;

Serial.print("dx = ");
Serial.print(dx);
Serial.print(", dy = ");
Serial.println(dy);
}


int Player(int pos)
{
analogWrite(9, 150);
digitalWrite(8, 1); // start player motor
int prePos = GetHeading() - pos;
unsigned long t = millis();
while (millis() - t < 4000)
{
delay(5);
int curPos = GetHeading() - pos;
if (prePos <= 0 && curPos >= 0)
{
digitalWrite(8, 0); // stop player motor
return 1;
}
prePos = curPos;
}
digitalWrite(8, 0); // stop player motor
return 0;
}

int PlayCard()
{
digitalWrite(5, 1);
unsigned long t = millis();
while (millis() - t < 2000)
{
if (digitalRead(inPin) == LOW)
{
delay(70);
digitalWrite(5, 0);
return 1;
}
}
digitalWrite(5, 0);
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
int translateIR() // returns value of "Car MP3 remote IR code received

{

 switch(results.value)

  {

  case 0xFFA25D:  
    digitalWrite(2, HIGH);
    delay(2000);
    digitalWrite(2, LOW);
    return 10;  // CH-
    break;

  case 0xFF629D:  
    digitalWrite(5, HIGH);
    delay(2000);
    digitalWrite(5, LOW);
    return 11; // CH
    break;

  case 0xFFE21D:  
    digitalWrite(2, LOW);
    Serial.println("pin2 set as low");
    return 12; // CH+
    break;

  case 0xFF22DD:  
    digitalWrite(5, LOW);
    Serial.println("pin5 set as low");
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
    return -1; // Other Button  / Bad Code

  } //END case

} //END translateIR



/* ( THE END ) */

