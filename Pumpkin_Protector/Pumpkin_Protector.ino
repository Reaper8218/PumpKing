//Pumpkin Protector/PumpKing/PumpkinHead.
// Developed By Andrew, James and, Sterling Lownik 2018-2020

//Include
#include <Wire.h>
#include <Servo.h> //For driving the Servos
#include <Adafruit_MMA8451.h>
#include <LiquidCrystal.h>
#include <SoftwareSerial.h>
#include <Adafruit_Sensor.h>
Adafruit_MMA8451 mma = Adafruit_MMA8451();
//PS2 Controler setup
#include <PS2X_lib.h>  //for v1.6

/******************************************************************
   set pins connected to PS2 controller:
     - 1e column: original
     - 2e colmun: Stef?
   replace pin numbers by the ones you use
 ******************************************************************/
#define PS2_DAT        30  //14    
#define PS2_CMD        31  //15
#define PS2_SEL        32  //16
#define PS2_CLK        33  //17

/******************************************************************
   select modes of PS2 controller:
     - pressures = analog reading of push-butttons
     - rumble    = motor rumbling
   uncomment 1 of the lines for each mode selection
 ******************************************************************/
#define pressures   true
//#define pressures   false
#define rumble      true
//#define rumble      false

PS2X ps2x; // create PS2 Controller Class

//right now, the library does NOT support hot pluggable controllers, meaning
//you must always either restart your Arduino after you connect the controller,
//or call config_gamepad(pins) again after connecting the controller.

int error = 0;
byte type = 0;
byte vibrate = 0;


//


//Boleen
bool DriveSystem = false; //Enable/Disable Drive Systems (Default is off/false)
bool Lights = false;//Enable/Disable Lighting/Infared Systems (Default is off/false)
bool Optics = false;//Enable/Disable Camraw Systems (Default is off/false)
bool Wepons = false;//Enable/Disable Wepons Systems (Default is off/false)

//Pin Constants
const int PinY = A0;
const int PinX = A1;
const int LightPin = 13;
//COMMUNICATIONS Pins
const int HBL_TX = 19;
const int HBL_RX = 18;
const int HBR_TX = 17;
const int HBR_RX = 16;
const int ESP8266_TX = 15;
const int ESP8266_RX = 14;

//Variables
int Page = 0;
int PosX, PosY, PosZ = 0; // variable to store the servo position
int x, y, z; //triple axis data
int rangeX;
int rangeY;
int rangeZ;
int numReadingsY = 20;
int numReadingsX = 20;
int readIndexY = 0;              // the index of the current reading
int readIndexX = 0;              // the index of the current reading
//int readingsY[numReadingsY];      // the readings from the analog input
//int readingsX[numReadingsX];      // the readings from the analog input
int totalY = 0;
int totalX = 0;                  // the running total
int averageY = 0;
int averageX = 0;                // the average

float LDV = 0;
float RDV = 0;
float DDV = 0;

//Define ADDRESSES
#define SLAVE_ADDR 9
#define ANSWERSIZE 5

//Define I2C Address of Module
#define HMC58003L 0x1E
const word MMA845X_Address = 0x1C; //0x1D

// Objects
//------------lcd(en,rw,lsb,x,x,msb)
LiquidCrystal lcd(7, 6, 5, 4, 3, 2);
//LiquidCrystal lcd(22, 23, 27, 26, 25, 24);
SoftwareSerial HBL (HBL_RX, HBL_TX);
SoftwareSerial HBR (HBR_RX, HBR_TX);
SoftwareSerial ESP8266 (ESP8266_RX, ESP8266_TX);
// twelve servo objects can be created on most boards
Servo ServoTilt; //Create servo object representing (Y)
Servo ServoPan; //Create servo object representing (X)

void setup() {

  //CHANGES for v1.6 HERE!!! **************PAY ATTENTION*************

  //setup pins and settings: GamePad(clock, command, attention, data, Pressures?, Rumble?) check for error
  error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);

  if (error == 0) {
    Serial.print("Found Controller, configured successful ");
    Serial.print("pressures = ");
    if (pressures)
      Serial.println("true ");
    else
      Serial.println("false");
    Serial.print("rumble = ");
    if (rumble)
      Serial.println("true)");
    else
      Serial.println("false");
    Serial.println("Try out all the buttons, X will vibrate the controller, faster as you press harder;");
    Serial.println("holding L1 or R1 will print out the analog stick values.");
    Serial.println("Note: Go to www.billporter.info for updates and to report bugs.");
  }
  else if (error == 1)
    Serial.println("No controller found, check wiring, see readme.txt to enable debug. visit www.billporter.info for troubleshooting tips");

  else if (error == 2)
    Serial.println("Controller found but not accepting commands. see readme.txt to enable debug. Visit www.billporter.info for troubleshooting tips");

  else if (error == 3)
    Serial.println("Controller refusing to enter Pressures mode, may not support it. ");

  //  Serial.print(ps2x.Analog(1), HEX);

  type = ps2x.readType();
  switch (type) {
    case 0:
      Serial.print("Unknown Controller type found ");
      break;
    case 1:
      Serial.print("DualShock Controller found ");
      break;
    case 2:
      Serial.print("GuitarHero Controller found ");
      break;
    case 3:
      Serial.print("Wireless Sony DualShock Controller found ");
      break;
  }







  // put your setup code here, to run once:
  lcd.begin(16, 2); //16x2,32x2,40x2,ect.
  delay(1); lcd.clear(); lcd.setCursor(0, 1); lcd.print("PumpKing V1.0");
  lcd.setCursor(0, 0); lcd.print("L"); Serial.begin(9600);
  delay(50); lcd.print("o"); HBL.begin (9600);
  delay(50); lcd.print("a"); HBR.begin (9600);
  delay(50); lcd.print("d"); ESP8266.begin (9600);
  delay(50); lcd.print("i"); ServoPan.attach(11); // attaches the ServoPan to pin 2
  delay(50); lcd.print("n"); ServoTilt.attach(10); // attaches the ServoTilt to pin 3
  delay(50); lcd.print("g"); //for(int thisReading = 0; thisReading < numReadingsX; thisReading++) {readingsX[thisReading] = 0;}//initialize all the readingsX to 0:
  delay(50); lcd.print("."); //for (int thisReading = 0; thisReading < numReadingsY; thisReading++) {readingsY[thisReading] = 0;}//initialize all the readingsY to 0:
  delay(50); lcd.print("."); Wire.begin();
  delay(50); lcd.print("."); delay(500); lcd.clear();

  delay(500); lcd.setCursor(0, 1); lcd.print("HMC5833L Test!");
  lcd.setCursor(0, 1); lcd.print("              ");
  //delay(1000);lcd.setCursor(0,1);lcd.print("Coultn't Start");
  Wire.beginTransmission(HMC58003L); //start talking
  Wire.write(0x02); // Set the Register
  Wire.write(0x00); // Tell the HMC5883 to Continuously Measure
  Wire.endTransmission();
  delay(1000);
  lcd.setCursor(0, 1); lcd.print("              ");
  lcd.setCursor(0, 1); lcd.print("HMC5833L Found");
  lcd.setCursor(0, 0);
  lcd.print("MMA8451 Test!");
  delay(1000);
  if (! mma.begin()) {
    lcd.setCursor(0, 0);
    lcd.print("             ");
    lcd.setCursor(0, 0);
    lcd.print("Couldnt Start");
    while (1);
  }
  lcd.setCursor(0, 0);
  lcd.print("              ");
  lcd.setCursor(0, 0);
  lcd.print("MMA8451 Found");
  mma.setRange(MMA8451_RANGE_2_G);
  delay(1500);
}

void loop() {

  //Tell the HMC what regist to begin writing data into
  Wire.beginTransmission(HMC58003L);
  Wire.write(0x03); //start with register 3.
  Wire.endTransmission();
  //Read the data.. 2 bytes for each axis.. 6 total bytes
  Wire.requestFrom(HMC58003L, 6);
  if (6 <= Wire.available()) {
    x = Wire.read() << 8; //MSB  x
    x |= Wire.read(); //LSB  x
    z = Wire.read() << 8; //MSB  z
    z |= Wire.read(); //LSB z
    y = Wire.read() << 8; //MSB y
    y |= Wire.read(); //LSB y
  }

  /* You must Read Gamepad to get new values and set vibration values
      ps2x.read_gamepad(small motor on/off, larger motor strenght from 0-255)
      if you don't enable the rumble, use ps2x.read_gamepad(); with no values
      You should call this at least once a second
  */
  if (error == 1) //skip loop if no controller found
    return;

  if (type == 2) { //Guitar Hero Controller
    ps2x.read_gamepad();          //read controller
  }

  else { //DualShock Controller
    ps2x.read_gamepad(false, vibrate); //read controller and set large motor to spin at 'vibrate' speed


    if (ps2x.Button(PSB_START))        //will be TRUE as long as button is pressed
      Serial.println("Start is being held");
    if (ps2x.Button(PSB_SELECT))
      Serial.println("Select is being held");

    if (ps2x.Button(PSB_PAD_UP)) {     //will be TRUE as long as button is pressed
      Serial.print("Up held this hard: ");
      Serial.println(ps2x.Analog(PSAB_PAD_UP), DEC);
      Page = 0;

    }
    if (ps2x.Button(PSB_PAD_RIGHT)) {
      Serial.print("Right held this hard: ");
      Serial.println(ps2x.Analog(PSAB_PAD_RIGHT), DEC);
      digitalWrite(9, HIGH);
      delay (50);
      digitalWrite(9, LOW);
      Page = 1;
    }
    if (ps2x.Button(PSB_PAD_LEFT)) {
      Serial.print("LEFT held this hard: ");
      Serial.println(ps2x.Analog(PSAB_PAD_LEFT), DEC);
      digitalWrite(6, HIGH);
      delay (50);
      digitalWrite(6, LOW);
      Page = 2;
    }
    if (ps2x.Button(PSB_PAD_DOWN)) {
      Serial.print("DOWN held this hard: ");
      Serial.println(ps2x.Analog(PSAB_PAD_DOWN), DEC);
      digitalWrite(5, HIGH);
      delay (50);
      digitalWrite(5, LOW);
      Page = 3;
    }

    vibrate = ps2x.Analog(PSAB_CROSS);  //this will set the large motor vibrate speed based on how hard you press the blue (X) button
    if (ps2x.NewButtonState()) {        //will be TRUE if any button changes state (on to off, or off to on)
      if (ps2x.Button(PSB_L3))
        Serial.println("L3 pressed");
      if (ps2x.Button(PSB_R3))
        Serial.println("R3 pressed");
      if (ps2x.Button(PSB_L2))
        Serial.println("L2 pressed");
      if (ps2x.Button(PSB_R2))
        Serial.println("R2 pressed");
      if (ps2x.Button(PSB_TRIANGLE)) {
        Serial.println("Triangle pressed");
        Lights = !Lights;

      }


    }

    if (ps2x.ButtonPressed(PSB_CIRCLE))              //will be TRUE if button was JUST pressed
      Serial.println("Circle just pressed");
    if (ps2x.NewButtonState(PSB_CROSS))              //will be TRUE if button was JUST pressed OR released
      Serial.println("X just changed");
    if (ps2x.ButtonReleased(PSB_SQUARE))             //will be TRUE if button was JUST released
      Serial.println("Square just released");

    if (ps2x.Button(PSB_L1) || ps2x.Button(PSB_R1)) { //print stick values if either is TRUE
      Serial.print("Stick Values:");
      Serial.print(ps2x.Analog(PSS_LY), DEC); //Left stick, Y axis. Other options: LX, RY, RX
      Serial.print(",");
      Serial.print(ps2x.Analog(PSS_LX), DEC);
      Serial.print(",");
      Serial.print(ps2x.Analog(PSS_RY), DEC);
      Serial.print(",");
      Serial.println(ps2x.Analog(PSS_RX), DEC);
    }




  }



  /*
    // subtract the last reading:
    totalX = totalX - readingsX[readIndexX];
    // read from the sensor:
    readingsX[readIndexX] = analogRead(PinX);
    // add the reading to the total:
    totalX = totalX + readingsX[readIndexX];
    // advance to the next position in the array:
    readIndexX = readIndexX + 1;
    // if we're at the end of the array...
    if (readIndexX >= numReadingsX) {
     // ...wrap around to the beginning:
     readIndexX = 0;
    }
    // calculate the average:
    averageX = totalX / numReadingsX;
    // send it to the computer as ASCII digits
    Serial.println(averageX);


    // subtract the last reading:
    totalY = totalY - readingsY[readIndexY];
    // read from the sensor:
    readingsY[readIndexY] = analogRead(PinY);
    // add the reading to the total:
    totalY = totalY + readingsY[readIndexY]{};
    // advance to the next position in the array:
    readIndexY = readIndexY + 1;
    // if we're at the end of the array...
    if (readIndexY >= numReadingsY) {
     // ...wrap around to the beginning:
     readIndexY = 0;
    }
    // calculate the average:
    averageY = totalY / numReadingsY;
    // send it to the computer as ASCII digits
    //end avg script

  */

  ps2x.read_gamepad(false, vibrate); //read controller and set large motor to spin at 'vibrate' speed
  Serial.println(averageY);
  //PosY = map(averageY,10,1000,125,10);//Down,Up
  //PosX = map(averageX,10,1000,0,177);//Right/Left

  PosY = map(ps2x.Analog(PSS_RY), 254, 0, 25, 145); //Down,Up
  PosX = map(ps2x.Analog(PSS_RX), 0, 254, 0, 180); //Right/Left
  PosZ = analogRead(A3);


  ServoTilt.write(PosX);
  ServoPan.write(PosY);


  DDV = map(ps2x.Analog(PSS_LX), 0, 254, -50, 50);
  //DDV = map(DDV 0,1023,-10000,10000 );
  LDV = map(ps2x.Analog(PSS_LY), 252, 1, -129, 127); //Down,Up
  RDV = map(ps2x.Analog(PSS_LY), 252, 1, -129, 127); //Right/Left
  LDV = LDV - DDV;

  RDV = RDV + DDV;

  if (Page == 0) {
    // Show Values
    lcd.clear();
    lcd.print("X:");
    lcd.setCursor(2, 0);
    lcd.print(x + 445);
    lcd.setCursor(8, 0);
    lcd.print("Y:");
    lcd.setCursor(10, 0);
    lcd.print(y);
    lcd.setCursor(0, 1);
    lcd.print("Z: ");
    lcd.setCursor(2, 1);
    lcd.print(z + 105);
    lcd.setCursor(5, 1);
    lcd.print(Lights);
    //delay(33);
    //Serial.print("X Value: ");
    //Serial.println(x);
    //Serial.print("Y Value: ");
    //Serial.println(y);
    //Serial.print("Z Value: ");
    //Serial.println(z+180);
    //Serial.println();
    delay(35);
  }
  else if (Page == 1) {
    lcd.setCursor(0, 1);
    lcd.print("PosX");
    lcd.setCursor(4, 1);
    lcd.print("   ");
    lcd.setCursor(4, 1);
    lcd.print(PosX);
    lcd.setCursor(8, 1);
    lcd.print("    ");
    lcd.setCursor(8, 1);
    lcd.print(averageX);
    lcd.setCursor(12, 0);
    lcd.print("Pump");
    lcd.setCursor(12, 1);
    lcd.print("King");
    lcd.setCursor(0, 0);
    lcd.print("PosY");
    lcd.setCursor(4, 0);
    lcd.print("   ");
    lcd.setCursor(4, 0);
    lcd.print(PosY);
    lcd.setCursor(8, 0);
    lcd.print("    ");
    lcd.setCursor(8, 0);
    lcd.print(averageY);
    int rangeY = map(PosY, 145, 25, 0, 3);
    switch (rangeY) {
      case 1: lcd.setCursor(7, 0); lcd.print("U"); break;
      case 2: lcd.setCursor(7, 0); lcd.print("C"); break;
      case 3: lcd.setCursor(7, 0); lcd.print("D"); break;
    }
    int rangeX = map(PosX, 180, 87, 0, 3);
    switch (rangeX) {
      case 1: lcd.setCursor(7, 1); lcd.print("L"); break;
      case 2: lcd.setCursor(7, 1); lcd.print("C"); break;
      case 3: lcd.setCursor(7, 1); lcd.print("R"); break;
    }
    delay(35);
  }
  else if (Page == 2 ) {
    lcd.clear();
    /*
      float LDV = 0;
      float RDV = 0;
      float DDV = 0;
    */
    lcd.setCursor(2, 0);
    lcd.print(LDV);
    lcd.setCursor(9, 0);
    lcd.print(RDV);
    lcd.setCursor(2, 1);
    lcd.print(DDV);

    delay(35);
  }
  pinMode(LightPin, !Lights);
}
/*
  delay(1);        // delay in between reads for stability
    break;


  case 2:    // your hand is on the sensor

  // Show Values
  lcd.clear();
  lcd.print("X:");
  lcd.setCursor(2, 0);
  lcd.print(x+445);
  lcd.setCursor(8, 0);
  lcd.print("Y:");
  lcd.setCursor(10, 0);
  lcd.print(y);
  lcd.setCursor(0, 1);
  lcd.print("Z: ");
  lcd.setCursor(2, 1);
  lcd.print(z+105);
  //delay(33);
  //Serial.print("X Value: ");
  //Serial.println(x);
  //Serial.print("Y Value: ");
  //Serial.println(y);
  //Serial.print("Z Value: ");
  //Serial.println(z+180);
  //Serial.println();
  delay(60);
  break;





  Serial.println(PosX);
  Serial.println(PosY);

  }
  }





  /*int CalcAve( int P){


  // subtract the last reading:
  T = T - R[NR];
  // read from the sensor:
  R[NR] = analogRead(P);
  // add the reading to the total:
  T = T + R[NR];
  // advance to the next position in the array:
  RI = RI + 1;

  // if we're at the end of the array...
  if (RI >= NR) {
  // ...wrap around to the beginning:
  RI = 0;
  }
  // calculate the average:
  A = T / NR;
  result = A
  return result;
  }
*/




