/*
 * THE ALL-IN-ONE-PACKAGE - Robotic Football Edition!!!
 * Principle Author: Aaron Roggow aaron.roggow@valpo.edu 618-477-7092
 */
/* 
 * Enable and disable the desired features here. 
 * There is error handling below for if things are enabled/disabled that shouldn't be.
 * Make sure if you add additional functionality, to add error handling for it being turned on at the wrong time
 */
//#define BASIC_DRIVETRAIN
#define OMNIWHEEL_DRIVETRAIN
//#define CENTER_PERIPHERALS
#define QB_PERIPHERALS
//#define WR_PERIPHERALS
//#define KICKER_PERIPHERALS
//#define RECEIVER_PERIPHERALS
#define LED_STRIP
//#define TACKLE
/*
 * 
 * Vesion History
 * 
 * 1.0 - AARON ROGGOW - adding pre-existant functionality for basic drivetrain, omniwheel drivetrain, center, qb, and kicker, and calibration and kids mode
 * 1.1 - TIM KRENTZ   - Added IR tracking camera (a QB peripheral) functionality
 * 
 * 
 * Controls...
 * SELECT - enter/exit CALIBRATION MODE - note, will exit into normal drive mode
 *    UP/DOWN - adjust motor offset
 * START - enter/exit KIDS MODE - will make the robots move much slower
 * Basic Drivetrain:
 *    Left Joystick U/D - forward/back
 *    Right Joystick L/R - turning
 *    R2 - turbo
 * Omniwheel Drivetrain:
 *    Left Joystick U/D/L/R - straffing any direction
 *    Right Joystick L/R - rotate
 *    L1 - reverse controls (back of robot is now front. Also reduces turn speed)
 * Center
 *    CROSS - lower ball release
 *    TRIANGLE - raise ball release
 * Kicker
 *    CROSS - kick at full force
 *    TRIANGLE - rotate kicker backwards slowly... for manual reloading
 * QB
 *    TRIANGLE - full power throw
 *    CIRCLE - 75% throw
 *    CROSS - 50% throw
 *    SQUARE - 25% throw... for handoffs
 *    UP/DOWN - adjust throwing offset - strengthen or weaken CIRCLE and CROSS throws
 *    R1 - thrower reload
 *    R3 - toggle WR tracking
 * WR
 *    R1  - Toggle Mast State
 * Pins...
 * LED Strip -
 *  Green - 12
 *  Blue  - 13
 * Tackle - 6
 * Basic Drivetrain - 
 *  Left Motor  - 9
 *  Right Motor - 10
 * Omniwheel Drivetrain - 
 *  Motor1 - 7
 *  Motor2 - 8
 *  Motor3 - 9
 *  Motor4 - 10
 * QB Thrower - 11
 * Kicker - 11
 * Center - 11
 */

//Error handling for what parts of the code are enabled
#ifdef BASIC_DRIVETRAIN
  #ifdef OMNIWHEEL_DRIVETRAIN
    #error Two drivetrains are enabled!
  #endif
  #ifdef QB_PERIPHERALS
    #error Quarterback peripherals enabled with basic drivetrain. Quarterback requires an omniwheel drive
    #ifdef CENTER_PERIPHERALS
      #error Multiple peripherals enabled!
    #endif
    #ifdef RECEIVER_PERIPHERALS
      #error Multiple peripherals enabled!
    #endif
    #ifdef KICKER_PERIPHERALS
      #error Multiple peripherals enabled!
    #endif
  #endif
  #ifdef RECEIVER_PERIPHERALS
    #warning You are making a receiver with a basic drivetrain. Make sure this is right.
    #ifdef CENTER_PERIPHERALS
      #error Multiple peripherals enabled!
    #endif
    #ifdef QB_PERIPHERALS
      #error Multiple peripherals enabled!
    #endif
    #ifdef WR_PERIPHERALS
      #error Multiple peripherals enabled!
    #endif
    #ifdef KICKER_PERIPHERALS
      #error Multiple peripherals enabled!
    #endif
  #endif
#endif

#ifdef OMNIWHEEL_DRIVETRAIN
  #ifdef CENTER_PERIPHERALS
    #error The center does not have an omniwheel drive, last I checked...
    #ifdef QB_PERIPHERALS
      #error Multiple peripherals enabled!
    #endif
    #ifdef RECEIVER_PERIPHERALS
      #error Multiple peripherals enabled!
    #endif
    #ifdef KICKER_PERIPHERALS
      #error Multiple peripherals enabled!
    #endif
  #endif
  #ifdef KICKER_PERIPHERALS
    #error Kicker does not use an omniwheel drive!
    #ifdef CENTER_PERIPHERALS
      #error Multiple peripherals enabled!
    #endif
    #ifdef RECEIVER_PERIPHERALS
      #error Multiple peripherals enabled!
    #endif
    #ifdef QB_PERIPHERALS
      #error Multiple peripherals enabled!
    #endif
  #endif
  #ifdef RECEIVER_PERIPHERALS
    #warning You are making a receiver with an omniwheel drivetrain. Make sure this is right.
    #ifdef CENTER_PERIPHERALS
      #error Multiple peripherals enabled!
    #endif
    #ifdef QB_PERIPHERALS
      #error Multiple peripherals enabled!
    #endif
    #ifdef KICKER_PERIPHERALS
      #error Multiple peripherals enabled!
    #endif
  #endif
#endif  

#ifndef BASIC_DRIVETRAIN
  #ifndef OMNIWHEEL_DRIVETRAIN
    #warning You don't have a drivetrain enabled! Don't expect this robot to drive!
  #endif
#endif

// mode definitions
#define DRIVING 1
#define CALIBRATION 2
#define KID 3 

//Include libraries
#include "PS3BT.h"
#include "usbhub.h"
#include <Servo.h>
#ifdef OMNIWHEEL_DRIVETRAIN
  #include <math.h> //used for trig in determining magnitude and angle
#endif


#ifdef LED_STRIP
  #define GREEN_LED 5
  #define BLUE_LED 6
#endif

#ifdef TACKLE
  #define TACKLE_INPUT 13
  int tackled = 1; //Tackle detects if the robot has been tackled
#endif

#ifdef BASIC_DRIVETRAIN
  #define LEFT_MOTOR_REVERSE -1 // these are to reverse the motor direction if a motor is wired backwards.
  #define RIGHT_MOTOR_REVERSE 1 // In almost every case, it would be better to fix the wiring than to change this in code

  #define LEFT_MOTOR 9
  #define RIGHT_MOTOR 10 
  #define DEFAULT_HANDICAP 3 //omniwheel and basic drivetrains both have their own version of this
  #define TURBO_HANDICAP 1
  #define KID_HANDICAP 7 //and this

  #define MAX_DRIVE 84 //limited because of issues with calibrating victors to full 0-180 range
  
  Servo leftMotor,rightMotor;
  int drive = 0; //Initial speed before turning calculations
  int turn = 0; //Turn is adjustment to drive for each motor separately to create turns
  int xInput,yInput,throttleL,throttleR;
#endif

#ifdef OMNIWHEEL_DRIVETRAIN
  #define MOTOR_1 7  //   1//-FRONT-\\4
  #define MOTOR_2 8  //     |       |
  #define MOTOR_3 9  //     |       |                                                                          <-- check this for accuracy
  #define MOTOR_4 10 //   2\\-------//3 
  #define DEFAULT_HANDICAP 1
  #define TURBO_HANDICAP 3
  #define KID_HANDICAP 7
  Servo motor1, motor2, motor3, motor4;
  #define PI_OVER_2 M_PI/2
  #define PI_OVER_4 M_PI/4
  int motor1Drive, motor2Drive, motor3Drive, motor4Drive;
  int xInput, yInput, turnInput; //we just make these global so we don't have to reallocate memory every single loop
  float magn, angle;
  float motorReverse = 0; //0 for not reversed, M_PI for reversed (think about your trig)
  int turnHandicap = 1;
  #define TURN_HANDICAP_AMOUNT 1

  int aimingFactor = 0;
#endif

#ifdef CENTER_PERIPHERALS
  #define CENTER_RELEASE 11
  #define CENTER_RELEASE_DOWN 120
  #define CENTER_RELEASE_UP 70
  Servo centerRelease;
#endif

#ifdef WR_PERIPHERALS
  #define MAST_PIN  3
  bool mastState;
#endif

#ifdef QB_PERIPHERALS
  void qbThrowerCtrl();

  //IR Camera Things
  void cameraCapture();
  void Write_2bytes(byte d1, byte d2);
  #include <Wire.h>
  int IRsensorAddress = 0xB0;
  int slaveAddress;
  byte data_buf[16];
  int CamX[4];
  int CamY[4];
  
  #define QB_THROWER 11
  Servo qbThrower;
  #define TRIANGLE_THROW 160 
  #define CIRCLE_THROW 150
  #define CROSS_THROW 125
  #define SQUARE_THROW 105
  #define RELOAD_THROW 86
  int throwOffset = 0; //used to adjust strength of cross and circle throws

  #define CAMERA_CENTER_X 485
  #define CAMERA_CENTER_WIDTH 3
  #define CAMERA_SPEED_FACTOR 35
  bool isWRSeen = false;
  bool isWRTracking = false;

  unsigned int calcThrow = 140;

  unsigned int distanceToWR;
#endif

#ifdef KICKER_PERIPHERALS
  #define SOLENOID_PIN 20
#endif


/////////////////////////////////////////////////////////////////////
// Universal stuffs
/////////////////////////////////////////////////////////////////////
int state = DRIVING; // the current state the robot is in
int handicap = DEFAULT_HANDICAP; //because of this line we have to have one of the drivetrains enabled
int motorCorrect = 0; //This will help center the stop value of the motors
//This is stuff for connecting the PS3 to USB.
int newconnect = 0; //Variable(boolean) for connection to ps3, also activates rumble
USB Usb;
USBHub Hub1(&Usb);
BTD Btd(&Usb);
PS3BT PS3(&Btd);

void eStop();
void driveCtrl();
void kickerCtrl();
void qbCtrl();
void flashLed();


void setup() {
  #ifdef BASIC_DRIVETRAIN
    leftMotor.attach(LEFT_MOTOR,1000,2000);
    leftMotor.writeMicroseconds(1500); //stopped
    rightMotor.attach(RIGHT_MOTOR,1000,2000);
    rightMotor.writeMicroseconds(1500);
  #endif

  #ifdef OMNIWHEEL_DRIVETRAIN
    motor1.attach(MOTOR_1,1000,2000);
    motor1.writeMicroseconds(1500);
    motor2.attach(MOTOR_2,1000,2000);
    motor2.writeMicroseconds(1500);
    motor3.attach(MOTOR_3,1000,2000);
    motor3.writeMicroseconds(1500);
    motor4.attach(MOTOR_4,1000,2000);
    motor4.writeMicroseconds(1500);
  #endif

  #ifdef LED_STRIP
    pinMode(BLUE_LED, OUTPUT);
    pinMode(GREEN_LED, OUTPUT);
    
    flashLed();
  #endif

  #ifdef TACKLE
    pinMode(TACKLE_INPUT, INPUT);
  #endif

  #ifdef CENTER_PERIPHERALS
    centerRelease.attach(CENTER_RELEASE);
    centerRelease.write(CENTER_RELEASE_UP);
  #endif

  #ifdef WR_PERIPHERALS
    pinMode(MAST_PIN, OUTPUT);
    mastState = false;
  #endif

  #ifdef QB_PERIPHERALS

    slaveAddress = IRsensorAddress >> 1; // This results in 0x21 as the address to pass toTWI
    Wire.begin();
    Write_2bytes(0x30,0x01); delay(10);
    Write_2bytes(0x30,0x08); delay(10);
    Write_2bytes(0x06,0x90); delay(10);
    Write_2bytes(0x08,0xC0); delay(10);
    Write_2bytes(0x1A,0x40); delay(10);
    Write_2bytes(0x33,0x33); delay(10);
    delay(100);
    
    qbThrower.attach(QB_THROWER);
    qbThrower.writeMicroseconds(1500);
    
  #endif

  #ifdef KICKER_PERIPHERALS
    pinMode(SOLENOID_PIN,OUTPUT);
    digitalWrite(SOLENOID_PIN, LOW);
  #endif

  //Begin Serial Communications
  Serial.begin(115200);
  if (Usb.Init() == -1)
  { // this is for an error message with USB connections
    Serial.print(F("\r\nOSC did not start"));
    while (1);
  }
  Serial.print(F("\r\nPS3 Bluetooth Library Started"));
}

void loop()
{
  Usb.Task(); //This updates the input from the PS3 controller
  
  if (PS3.PS3Connected || PS3.PS3NavigationConnected) //This only lets the program run if the PS3 controller is connected.
  {
    if (newconnect == 0)
    { //this is the vibration that you feel when you first connect
      newconnect++;
      //Serial.println("Rumble is on!");
      PS3.moveSetRumble(64);
      PS3.setRumbleOn(100, 255, 100, 255); //VIBRATE!!!
    }

    if(state==CALIBRATION)
    { //CALIBRATION MODE
      if (PS3.getButtonClick(UP)) 
      {
        motorCorrect++;
        PS3.setRumbleOn(5,255,5,0); //vibrate ON THE LEFT!
              }
      else if (PS3.getButtonClick(DOWN)) 
      {
        motorCorrect--;
        PS3.setRumbleOn(5,0,5,255); //vibrate ON THE RIGHT!
      }
      else if (PS3.getButtonClick(SELECT))
      {
        state = DRIVING;
        PS3.setLedRaw(1); // OFF OFF OFF ON
        PS3.setRumbleOn(5,0,5,255); //should vibrate left, then right, then both
        PS3.setRumbleOn(5,255,5,0);
        PS3.setRumbleOn(5,255,5,255);
      }

      eStop();
    }
    else
    {  
    #ifdef TACKLE //this fancy bit here changes the condition for the if statement for whether or not tackle is enabled. cool stuff
      tackled = digitalRead(TACKLE_INPUT);

      if(tackled&&(state==DRIVING||state==KID))
      {
    #else
      if(state==DRIVING||state==KID)
      {
    #endif
        
        if(PS3.getButtonClick(SELECT))
        {
          state = CALIBRATION;
          PS3.setLedRaw(15); // ON ON ON ON
          PS3.setRumbleOn(5,255,5,255); //vibrate both, then left, then right
          PS3.setRumbleOn(5,255,5,0);
          PS3.setRumbleOn(5,0,5,255);
        }
        if(PS3.getButtonClick(START))
        { //switches between normal driving mode and kid mode
          if(state==DRIVING)
          {
            state=KID;
            PS3.setLedRaw(9); //ON OFF OFF ON
            PS3.setRumbleOn(5,255,5,255); //vibrate both, then left, then right
          }
          else if(state==KID)
          {
            state=DRIVING;
            PS3.setLedRaw(1); //OFF OFF OFF ON
            PS3.setRumbleOn(5,255,5,255); //vibrate both, then left, then right
          }
        }
        
      #ifdef OMNIWHEEL_DRIVETRAIN
        if(PS3.getButtonPress(L1))
        {
          motorReverse = 0; // this is reversed
          turnHandicap = TURN_HANDICAP_AMOUNT;
          handicap = TURBO_HANDICAP;
        }
        else
        {
          motorReverse = M_PI;
          turnHandicap = 1;
          handicap = DEFAULT_HANDICAP;
        }
      #endif

      #ifdef LED_STRIP
        digitalWrite(BLUE_LED,LOW);
        digitalWrite(GREEN_LED,HIGH);
      #endif

      #ifdef CENTER_PERIPHERALS
        centerCtrl();
      #endif

      #ifdef WR_PERIPHERALS
        if(PS3.getButtonClick(R1))
        {
          if(mastState == true)
          {
            mastState = false;
            digitalWrite(MAST_PIN,LOW);
          }
          else
          {
            mastState = true;
            digitalWrite(MAST_PIN,HIGH);
          }
        }
      #endif

      #ifdef QB_PERIPHERALS
        cameraCapture();
        qbThrowerCtrl();
      #endif
        
      #ifdef KICKER_PERIPHERALS
        kickerCtrl();
      #endif

        if(state==DRIVING)
        {
          if(PS3.getButtonPress(R2))
          {
            handicap = TURBO_HANDICAP; //TURBO!!!!!!!!!!!!!!
          }
          else 
          {
            handicap = DEFAULT_HANDICAP;
          }
        }
        else if(state==KID)
        {
          handicap = KID_HANDICAP;
        }
        
        driveCtrl();
      }
      else
      {
        eStop();

      #ifdef LED_STRIP
        digitalWrite(GREEN_LED, LOW);
        digitalWrite(BLUE_LED, HIGH);
      #endif

        PS3.moveSetRumble(64);
        PS3.setRumbleOn(10,255,10,255); //vibrate!
      }
    }
  }
  else
  {
    eStop();

    #ifdef LED_STRIP
      digitalWrite(GREEN_LED, LOW);
      digitalWrite(BLUE_LED, HIGH);
    #endif
  }
}

void eStop()
{
  #ifdef BASIC_DRIVETRAIN
    leftMotor.writeMicroseconds(1500);
    rightMotor.writeMicroseconds(1500);
  #endif
  #ifdef OMNIWHEEL_DRIVETRAIN
    motor1.writeMicroseconds(1500);
    motor2.writeMicroseconds(1500);
    motor3.writeMicroseconds(1500);
    motor4.writeMicroseconds(1500);
  #endif
  #ifdef QB_PERIPHERALS
    qbThrower.writeMicroseconds(1500);
  #endif
}

void driveCtrl()
{
  #ifdef BASIC_DRIVETRAIN
    yInput = map(PS3.getAnalogHat(LeftHatY), 0, 255, -90, 90); //Recieves PS3 forward/backward input
    xInput = map(PS3.getAnalogHat(RightHatX), 0, 255, 90, -90); //Recieves PS3 horizontal input and sets it to an inverted scale of 90 to -90

    if(abs(yInput) < 8) yInput = 0; //deals with the stickiness of PS3 joysticks
    if(abs(xInput) < 8) xInput = 0;

    if((yInput==0)&&(xInput==0))
    { //if no input this should ensure that the motors actually stop, and skip the rest of the drive function
      leftMotor.writeMicroseconds(1500);
      rightMotor.writeMicroseconds(1500);
      return;
    }
    
    //Instead of following some sort of equation to slow down acceleration
    //We just increment the speed by one towards the desired speed.
    //The acceleration is then slowed because of the loop cycle time
    if (drive < yInput)drive++; //Accelerates
    else if (drive > yInput) drive--; //Decelerates

    if (turn < xInput) turn++;
    else if (turn > xInput) turn--;

    throttleL = LEFT_MOTOR_REVERSE * ((drive + turn) / handicap) + motorCorrect; //This is the final variable that decides motor speed.
    throttleR = RIGHT_MOTOR_REVERSE * ((drive - turn) / handicap ) + motorCorrect;

    if (throttleL > MAX_DRIVE) throttleL = MAX_DRIVE;
    else if (throttleL < -MAX_DRIVE)throttleL = -MAX_DRIVE;
    if (throttleR > MAX_DRIVE) throttleR = MAX_DRIVE;
    else if (throttleR < -MAX_DRIVE)throttleR = -MAX_DRIVE;

    leftMotor.write(throttleL + 90); //Sending values to the speed controllers
    rightMotor.write(throttleR + 90);
  #endif



  #ifdef OMNIWHEEL_DRIVETRAIN
    yInput = map(PS3.getAnalogHat(LeftHatY), 0, 255, 90, -90); //Recieves PS3 forward/backward input
    xInput = map(PS3.getAnalogHat(LeftHatX), 0, 255, 90, -90); //Recieves PS3 horizontal input and sets it to an inverted scale of 90 to -90
    turnInput = map(PS3.getAnalogHat(RightHatX), 0, 255, -10, 10); //received turn input from right joystick

    if(PS3.getButtonPress(UP))
    {
      yInput = 90;
      xInput = 0;
    }
    else if(PS3.getButtonPress(DOWN))
    {
      yInput = -90;
      xInput = 0;
    }
    else if(PS3.getButtonPress(LEFT))
    {
      yInput = 0;
      xInput = 90;
    }
    else if(PS3.getButtonPress(RIGHT))
    {
      yInput = 0;
      xInput = -90;
    }
    
    if(abs(yInput)<8)yInput=0; //taking care of sticky input
    if(abs(xInput)<8)xInput=0;
    if(abs(turnInput)<2)turnInput=0; //this one is likely taken care of by integer rounding already rounding

    magn = sqrt(pow(xInput,2) + pow(yInput,2)); //finding magnitude of input
    //magn = map(magn, 0, float(90)*sqrt(2.0), 0, 60); //need to investigate the purpose of this
    angle = atan2(double(yInput),double(xInput)); //atan2 accounts for quadrants of input

    {
      motor4Drive = ((magn*(sin(angle + PI_OVER_4 + motorReverse))/handicap)                                    + (float)(turnHandicap*turnInput)+90+motorCorrect) + aimingFactor;
      motor1Drive = ((magn*(sin(angle + PI_OVER_4 + PI_OVER_2 + motorReverse))/handicap)                        + (float)(turnHandicap*turnInput)+90+motorCorrect) + aimingFactor;
      motor2Drive = ((magn*(sin(angle + PI_OVER_4 + PI_OVER_2 + PI_OVER_2 + motorReverse))/handicap)            + (float)(turnHandicap*turnInput)+90+motorCorrect) + aimingFactor;
      motor3Drive = ((magn*(sin(angle + PI_OVER_4 + PI_OVER_2 + PI_OVER_2 + PI_OVER_2 + motorReverse))/handicap)+ (float)(turnHandicap*turnInput)+90+motorCorrect) + aimingFactor;
    }

    if (motor1Drive < 5)motor1Drive = 5;
    else if (motor1Drive > 175)motor1Drive = 175;
    if (motor2Drive < 5)motor2Drive = 5;
    else if (motor2Drive > 175)motor2Drive = 175;
    if (motor3Drive < 5)motor3Drive = 5;
    else if (motor3Drive > 175)motor3Drive = 175;
    if (motor4Drive < 5)motor1Drive = 5;
    else if (motor4Drive > 175)motor4Drive = 175;

    motor1.write(motor1Drive);
    motor2.write(motor2Drive);
    motor3.write(motor3Drive);
    motor4.write(motor4Drive);
  #endif
}

#ifdef QB_PERIPHERALS
void qbThrowerCtrl()
{
  long response = 0;
  int val = 0;
  if(PS3.getButtonPress(L2))
  {
    if(PS3.getButtonClick(UP))
    {
      throwOffset+=5;
      PS3.setRumbleOn(5,255,5,0); //vibrate on the left!
    }
    else if(PS3.getButtonClick(DOWN))
    {
      throwOffset-=5;
      PS3.setRumbleOn(5,0,5,255); //vibrate on the right!
    }
  }

  if(PS3.getButtonPress(TRIANGLE))    qbThrower.write(TRIANGLE_THROW);
  else if(PS3.getButtonPress(CIRCLE)) qbThrower.write(CIRCLE_THROW+throwOffset);
  else if(PS3.getButtonPress(CROSS))  qbThrower.write(CROSS_THROW+throwOffset);
  else if(PS3.getButtonPress(SQUARE)) qbThrower.write(SQUARE_THROW);
  else if(PS3.getButtonPress(R1))
  {
    qbThrower.write(RELOAD_THROW);
    throwOffset=0;
  }
  else if((PS3.getButtonPress(L2)) && (PS3.getButtonPress(R2)) && (isWRTracking == true))
  {
    qbThrower.write(calcThrow);
    //isWRTracking = false;
  }
  /*
  {
    while(Serial.available() <= 0);
    response = Serial.parseInt();
    val = (int)response;
    Serial.println(val);
    qbThrower.write(val);
    delay(400);
    qbThrower.writeMicroseconds(1500);
  }
  */
  else qbThrower.writeMicroseconds(1500);
}

void cameraCapture()
{
  int i;
  int s;
  int numGoodPoints = 0;
  int firstPoint = 0;
  int secondPoint = 0;
  int pixWidth = 0;
  
  Wire.beginTransmission(slaveAddress);
  Wire.write(0x36);
  Wire.endTransmission();
  Wire.requestFrom(slaveAddress, 16); // Request the 2 byte heading (MSB comes first)
  for (i=0;i<16;i++) { data_buf[i]=0; }
  
  i=0;
  
  while(Wire.available() && i < 16) {
    data_buf[i] = Wire.read();
    i++;
  }
  
  CamX[0] = data_buf[1];
  CamY[0] = data_buf[2];
  s = data_buf[3];
  CamX[0] += (s & 0x30) <<4;
  CamY[0] += (s & 0xC0) <<2;
  
  CamX[1] = data_buf[4];
  CamY[1] = data_buf[5];
  s = data_buf[6];
  CamX[1] += (s & 0x30) <<4;
  CamY[1] += (s & 0xC0) <<2;
  
  CamX[2] = data_buf[7];
  CamY[2] = data_buf[8];
  s = data_buf[9];
  CamX[2] += (s & 0x30) <<4;
  CamY[2] += (s & 0xC0) <<2;
  
  CamX[3] = data_buf[10];
  CamY[3] = data_buf[11];
  s = data_buf[12];
  CamX[3] += (s & 0x30) <<4;
  CamY[3] += (s & 0xC0) <<2;

  // DETERMINE IF WE CAN SEE THE WR
  isWRSeen = false;
  digitalWrite(BLUE_LED, LOW);
  for (i=0; i < 4; i++)
  {
    if (CamX[i] != 1023)
    {
      isWRSeen = true;
      digitalWrite(BLUE_LED,HIGH);
      if (numGoodPoints == 0)
      {
        firstPoint = i;
        numGoodPoints++;
      } else if (numGoodPoints == 1)
      {
        secondPoint = i;
        numGoodPoints++;
        pixWidth = CamY[firstPoint] - CamY[secondPoint];
        pixWidth = abs(pixWidth);
        //Serial.print("PixWidth: ");
        //Serial.print(pixWidth);
        distanceToWR = 17688/pixWidth;
        //Serial.print("Distance Raw: ");
        //Serial.print(distanceToWR);
        distanceToWR -= 12;
        distanceToWR = distanceToWR * 7;
        distanceToWR = distanceToWR / 8;
        //Serial.print("Distance: ");
        //Serial.print(distanceToWR);
        //Serial.print("Distance: ");
        //Serial.println(distanceToWR);

        calcThrow = distanceToWR*6;
        //Serial.print("calcThrow1: ");
        //Serial.print(calcThrow);
        calcThrow = calcThrow/18;
        //Serial.print("calcThrow2: ");
        //Serial.print(calcThrow);
        calcThrow = calcThrow+100
        ;
        //Serial.print("calcThrow3: ");
        //Serial.println(calcThrow);

        if(calcThrow > 145) calcThrow = 145;
        else if(calcThrow < 110) calcThrow = 110;
        break;
      }
    }
  }
  //===========================================================================================
  // NOTE: ITERATING VARIABLE 'i' IS RETAINED AFTER FOR LOOP FOR USE IN UPDATNG ROTATION OFFSET
  // BE CAREFUL BE CAREFUL BE CAREFUL BE CAREFUL BE CAREFUL BE CAREFUL BE CAREFUL
  //===========================================================================================

  // UPDATE TRACKING STATE
  if(PS3.getButtonClick(R3))
  {
    if((isWRTracking == false) && (isWRSeen == true))   // Only track if we currently see the WR
    {
      isWRTracking = true;
      PS3.moveSetRumble(64);
      PS3.setRumbleOn(100, 255, 100, 255); //VIBRATE!!!
    }
    else if(isWRTracking == true)
    {
      isWRTracking = false;
    }
  }
  if(isWRSeen == false)
  {
    isWRTracking = false;
  }

  // UPDATE ROTATION OFFSET
  if ((isWRTracking == true) && (isWRSeen == true))
  {
      aimingFactor = CamX[firstPoint] - CAMERA_CENTER_X;
      if (abs(aimingFactor) < CAMERA_CENTER_WIDTH)
      {
        aimingFactor = 0;
      }
      else
      {
        aimingFactor = map(aimingFactor,0-CAMERA_CENTER_X,1023-CAMERA_CENTER_X,-1*CAMERA_SPEED_FACTOR,CAMERA_SPEED_FACTOR);
      }
  }
  else if (isWRSeen == false)
  {
    aimingFactor = 0;
  }
}

void Write_2bytes(byte d1, byte d2)
{
Wire.beginTransmission(slaveAddress);
Wire.write(d1); Wire.write(d2);
Wire.endTransmission();
}
#endif

#ifdef KICKER_PERIPHERALS
void kickerCtrl()
{
  if(PS3.getButtonClick(CROSS))
  {
    digitalWrite(SOLENOID_PIN, HIGH);
    delay(500);
    digitalWrite(SOLENOID_PIN, LOW);
  }
  else if(PS3.getButtonClick(TRIANGLE))
  {
    digitalWrite(SOLENOID_PIN, HIGH);
    delay(70);
    digitalWrite(SOLENOID_PIN, LOW);
  }
}
#endif

#ifdef LED_STRIP
void flashLed()
{
  //flash the leds
    digitalWrite(BLUE_LED, LOW);
    digitalWrite(GREEN_LED, HIGH);
    delay(300);
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(BLUE_LED, HIGH);
    delay(300);
    digitalWrite(BLUE_LED, LOW);
    digitalWrite(GREEN_LED, HIGH);
    delay(300);
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(BLUE_LED, HIGH);
}
#endif

#ifdef CENTER_PERIPHERALS
void centerCtrl()
{
  if(PS3.getButtonClick(TRIANGLE)) centerRelease.write(CENTER_RELEASE_UP);
  else if(PS3.getButtonClick(CROSS)) centerRelease.write(CENTER_RELEASE_DOWN);
}
#endif


