#include <EEPROM.h>
/*
   THE ALL-IN-ONE-PACKAGE - Robotic Football Edition!!!
   
   
   
   
   
   
   
   
   
   Enable and disable the desired features here.
   There is error handling below for if things are enabled/disabled that shouldn't be.
   Make sure if you add additional functionality, to add error handling for it being turned on at the wrong time

*/
      
int driveState = EEPROM.read(0); //Reads value from the first value form the EEPROM
int inverting = 0;              //Sets inverting to 0
 
//#define BASIC_DRIVETRAIN    //uncomment for 2 drive wheels
//#define DUAL_MOTORS
//#define LR_TACKLE_PERIPHERALS         //uncomment for special handicap for the tackles
#define OMNIWHEEL_DRIVETRAIN  //uncomment for omniwheel robots


//#define CENTER_PERIPHERALS  //uncomment for special features of center 
//#define QB_PERIPHERALS      //uncomment for special QB features
#define IR_MAST
//#define QB_TRACKING
//#define KICKER_PERIPHERALS  //uncomment for special Kicker features
//#define RECEIVER_PERIPHERALS  
#define LED_STRIP       //uncomment for LED functionality
#define TACKLE          //uncomment for tackle sensor functionality
//#define ROTATION_LOCK
/*
   Vesion History
   1.0.0 - AARON ROGGOW - adding pre-existant functionality for basic drivetrain, omniwheel drivetrain, center, qb, and kicker, and calibration and kids mode
   1.0.1 - Matt Bull- fixed eStop functionality for if controller becomes disconnected, added ability to disconnect controller in calibration mode when PS is pressed
   1.0.2 - Matt Bull- added ability to disconnect controller to the main loop for testing eStop
   1.0.3 - Aaron Roggow - added compass code to quarterback (rotation_lock)
   1.0.4 - Aaron Roggow - added red led
   1.0.5 - Jacob Gehring
   1.0.6 - Julia Jenks - adjusted LEDs to function green for idle, red for tackled, and blue for non-ball carriers
                         commented code  
 
   Controls...
   SELECT - enter/exit CALIBRATION MODE - note, will exit into normal drive mode
      UP/DOWN - adjust motor offset
   START - enter/exit KIDS MODE - will make the robots move much slower
   Basic Drivetrain:
      Left Joystick U/D - forward/back
      Right Joystick L/R - turning
      R2 - turbo
      To switch to tank drive hold L1 and press Select
   Omniwheel Drivetrain:
      Left Joystick U/D/L/R - straffing any direction
      Right Joystick L/R - rotate
      L1 - reverse controls (back of robot is now front. Also reduces turn speed)
   Center
      CROSS - lower ball release
      TRIANGLE - raise ball release
   Kicker
      CROSS - kick at full force
      TRIANGLE - rotate kicker backwards slowly... for manual reloading
   QB
      TRIANGLE - full power throw
      CIRCLE - 75% throw
      CROSS - 50% throw
      SQUARE - 25% throw... for handoffs
      UP/DOWN - adjust throwing offset - strengthen or weaken CIRCLE and CROSS throws
      R1 - thrower reload
   Rotation Lock
      L3 - reset orientation
   Pins...
   LED Strip -
    Red   - 11
    Green - 12
    Blue  - 13
   Tackle - 6
   Basic Drivetrain -
    Left Motor  - 9
    Right Motor - 10
   Omniwheel Drivetrain -
    Motor1 - 7
    Motor2 - 8
    Motor3 - 9
    Motor4 - 10
   QB Thrower - 5
   Kicker - 5
   Center - 5
*/
 
// mode definitions
#define DRIVING         1
#define CALIBRATION     2
#define KID             3
 
//Include libraries
#include <PS3BT.h>
#include <usbhub.h>
#include <Servo.h>
 
 
#ifdef OMNIWHEEL_DRIVETRAIN
  #include <math.h>                   // used for trig in determining magnitude and angle
#endif
 
#ifdef ROTATION_LOCK
  #include <Wire.h>
  #include <Adafruit_Sensor.h>
  #include <Adafruit_BNO055.h>
  #include <utility/imumaths.h>
#endif
 
#ifdef LED_STRIP
  #define RED_LED         11          //Red LED control is wired to pin 11
  #define GREEN_LED       12          //Green LED control is wired to pin 12
  #define BLUE_LED        13          //Blue LED control is wired to pin 13
#endif
 
#ifdef TACKLE
  #define TACKLE_INPUT    6           // Tackle sensor is wired to pin 6
  int tackled = 1;                    // Tackle detects if the robot has been tackled (1 = not tackled)
  bool hasIndicatedTackle = false;    // variable to check if robot is previously in tackled state
#endif
 
#ifdef BASIC_DRIVETRAIN
  /*  these are to reverse the motor direction if a motor is wired backwards.
   *  In almost every case, it would be better to fix the wiring than to change this in code
   */
  #define LEFT_MOTOR_REVERSE    -1     
  #define RIGHT_MOTOR_REVERSE   1     
   
  #define LEFT_MOTOR            9     // left motor is wired to pin 9
  #define RIGHT_MOTOR           10    // right motor is wired to pin 10

  #ifdef DUAL_MOTORS
    #define LEFT_MOTOR2           7
    #define RIGHT_MOTOR2          8
    Servo leftMotor2, rightMotor2;
  #endif
  
  // omniwheel and basic drivetrains both have their own version of these two
  #define ALTERNATE_HANDICAP    1    
  #define DEFAULT_HANDICAP      3     // when not using boost, speed is divided by 3
  #define KID_HANDICAP          7     // when in kids mode, speed s divided by 7
   
  #define MAX_DRIVE             84    // limited because of issues with calibrating victors to full 0-180 range
  
  #define ARCADE_MODE			0
  #define TANK_MODE				1
   
  Servo leftMotor, rightMotor;        // Define motor objects
  int drive = 0;                      // Initial speed before turning calculations
  int turn = 0;                       // Turn is adjustment to drive for each motor separately to create turns
  int xInput, yInput, throttleL, throttleR;
#endif
 
#ifdef OMNIWHEEL_DRIVETRAIN
  // Pins for Omni drive robots
  #define MOTOR_1               7     //   1//-FRONT-\\4
  #define MOTOR_2               8     //     |       |
  #define MOTOR_3               9     //     |       |                                                                          <-- check this for accuracy
  #define MOTOR_4               10    //   2\\-------//3
  #define KID_HANDICAP          5     // when omni is in kids mode, speed is divided by 5 
  #define PI_OVER_2             M_PI/2
  #define PI_OVER_4             M_PI/4
  #define TURN_HANDICAP_AMOUNT  1     // divide turn speed by 1
  #define MAX_TURN 14                 // limit the value for turning for calculations to send final speed to motors
  #ifdef QB_PERIPHERALS               // If this is the QB then R2 slows down otherwise R2 is boost.
    #define DEFAULT_HANDICAP      1   // when not using boost, drive full speed
    #define ALTERNATE_HANDICAP    3   // when using boost, divide speed by 3
  #else
    #define DEFAULT_HANDICAP      3   // when not using boost, divide speed by 3
    #define ALTERNATE_HANDICAP    2   // when using boost, divide speed by 2 
  #endif
  Servo motor1, motor2, motor3, motor4;                                       // Define omni motor objects
  // we just make these global so we don't have to reallocate memory every single loop
  int motor1Drive, motor2Drive, motor3Drive, motor4Drive;                     // Define global variables for driving
  int motor1Input = 90, motor2Input = 90, motor3Input = 90, motor4Input = 90; 
  int xInput, yInput, turnInput;                          
  float magn, angle;
  float motorReverse = 0;             // 0 for not reversed, M_PI for reversed (think about your trig) again better to flip motor leads
  int turnHandicap = 1;
#endif

#ifdef CENTER_PERIPHERALS
  #define CENTER_RELEASE        5     // the ball release servo is wired to pin 5
  #define CENTER_RELEASE_DOWN   120   // these are the angles between 0 and 180 to set servo for releasing and holding the ball
  #define CENTER_RELEASE_UP     70    
  Servo centerRelease;                // define servo object for ball release 
#endif

#ifdef IR_MAST
  #define IR_MAST_SERVO         5
  Servo irMast;
  #define SERVO_UP_POS 25
  #define SERVO_DN_POS 110
  #define SERVO_UP true
  #define SERVO_DN false
  bool servoState = SERVO_DN;
#endif
 
#ifdef QB_PERIPHERALS

#define QB_THROWER            5
Servo qbThrower;
#define TRIANGLE_THROW        175
#define CIRCLE_THROW          125
#define CROSS_THROW           108

#define SQUARE_THROW          102
#define RELOAD_THROW          88
int throwOffset = 0;                //used to adjust strength of cross and circle throws
#endif

#ifdef QB_TRACKING
  int aimingFactor = 0;             //value sent to the motors to aim

  //IR Camera Things
  void cameraCapture();
  void Write_2bytes(byte d1, byte d2);
  #include <Wire.h>
  int IRsensorAddress = 0xB0;
  int slaveAddress;
  byte data_buf[16];
  int CamX[4];
  int CamY[4];
  
  #define CAMERA_CENTER_X 485
  #define CAMERA_CENTER_WIDTH 3
  #define CAMERA_SPEED_FACTOR 35
  bool isWRSeen = false;
  bool isWRTracking = false;

  unsigned int calcThrow = 140;
  unsigned int distanceToWR;
#endif

#ifdef KICKER_PERIPHERALS
  #define KICKER_MOTOR          5     // Kicker motor is wired to pin 5
  //these are the speeds for kicking and reload the kicker foot
  #define KICKER_POWER          175   
  #define KICKER_RELOAD         85
  Servo kicker;                       // Define motor object for the kicker motor
#endif
 
#ifdef ROTATION_LOCK
  #define MINIMUM_ANGLE               5
  #define SAMPLE_PERIOD               50
  #define ROTATION_CORRECT_MAGNITUDE  5
  Adafruit_BNO055 gyro = Adafruit_BNO055(55); //our rotation sensor;
  int rotationCorrect = 0;
  int desiredRotation = 0;
  sensors_event_t rotationReadout;
  int sample = 0;
  int wasIturning = 0;
#endif
 
/////////////////////////////////////////////////////////////////////
// Universal stuffs
/////////////////////////////////////////////////////////////////////
int state = DRIVING;                // the current state the robot is in
int handicap = DEFAULT_HANDICAP;    // This line requires one drivetrain to be enabled
int motorCorrect = 0;               // This will help center the stop value of the motors

// This is stuff for connecting the PS3 to USB.
int newconnect = 0;                 // Variable(boolean) for connection to ps3, also activates rumble
USB Usb;
USBHub Hub1(&Usb);
BTD Btd(&Usb);
PS3BT PS3(&Btd);
 
void setup() {
#ifdef BASIC_DRIVETRAIN
  /* These lines are attaching the motor objects to their output pins on the arduino
   * 1000, 2000 refers to the minimum and maximum pulse widths to send to the motors (AKA full forward/reverse)
   * 1500 represents stop
   */
  leftMotor.attach(LEFT_MOTOR, 1000, 2000);
  leftMotor.writeMicroseconds(1500);            //stopped
  rightMotor.attach(RIGHT_MOTOR, 1000, 2000);
  rightMotor.writeMicroseconds(1500);
  if (driveState != TANK_MODE && driveState != ARCADE_MODE) {   //If the EPPROM does not contain any values it will set it to one and default to arcade drive
    EEPROM.write(0, 0);
    driveState = ARCADE_MODE;
  }
#endif

#ifdef DUAL_MOTORS
  leftMotor2.attach(LEFT_MOTOR2, 1000, 2000);
  leftMotor2.writeMicroseconds(1500);
  rightMotor2.attach(RIGHT_MOTOR2, 1000, 2000);
  rightMotor2.writeMicroseconds(1500);
#endif
 
#ifdef OMNIWHEEL_DRIVETRAIN
  /* These lines are attaching the motor objects to their output pins on the arduino
   * 1000, 2000 refers to the minimum and maximum pulse widths to send to the motors (AKA full forward/reverse)
   * 1500 represents stop
   */
  motor1.attach(MOTOR_1, 1000, 2000);
  motor1.writeMicroseconds(1500);
  motor2.attach(MOTOR_2, 1000, 2000);
  motor2.writeMicroseconds(1500);
  motor3.attach(MOTOR_3, 1000, 2000);
  motor3.writeMicroseconds(1500);
  motor4.attach(MOTOR_4, 1000, 2000);
  motor4.writeMicroseconds(1500);
#endif
 
#ifdef LED_STRIP
  //define pins for LEDs as outputs
  pinMode(BLUE_LED,  OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(RED_LED,   OUTPUT);
  flashLed();                   // call the function to test functionality of all three LED colors individually
#endif
 
#ifdef TACKLE
  pinMode(TACKLE_INPUT, INPUT); // define the tackle sensor pin as an input
#endif
 
#ifdef CENTER_PERIPHERALS
  centerRelease.attach(CENTER_RELEASE);   // attach ball release servo to its pin
  centerRelease.write(CENTER_RELEASE_UP); // 
#endif

#ifdef IR_MAST
  irMast.attach(IR_MAST_SERVO,    560, 25200);
  setServo(SERVO_DN);
#endif

#ifdef QB_PERIPHERALS
  qbThrower.attach(QB_THROWER);
  qbThrower.writeMicroseconds(1500);
#endif
  
#ifdef QB_TRACKING
    slaveAddress = IRsensorAddress >> 1; // This results in 0x21 as the address to pass toTWI
    Wire.begin();
    Write_2bytes(0x30,0x01); delay(10);
    Write_2bytes(0x30,0x08); delay(10);
    Write_2bytes(0x06,0x90); delay(10);
    Write_2bytes(0x08,0xC0); delay(10);
    Write_2bytes(0x1A,0x40); delay(10);
    Write_2bytes(0x33,0x33); delay(10);
    delay(100);
    
#endif
#ifdef KICKER_PERIPHERALS
  kicker.attach(KICKER_MOTOR);
  kicker.writeMicroseconds(1500);
#endif
 
 
  //Begin Serial Communications
  Serial.begin(115200);
  if (Usb.Init() == -1)                 // this is for an error message with USB connections
  {
    Serial.print(F("\r\nOSC did not start"));
    while (1);
  }
  Serial.print(F("\r\nPS3 Bluetooth Library Started"));
 
#ifdef ROTATION_LOCK
  if (!gyro.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1)
    {
      flashLed();
    }
  }
#endif
}
 
void loop()
{
  Usb.Task();                           // This updates the input from the PS3 controller
 
  if (PS3.PS3Connected)                 // This only lets the program run if the PS3
    // controller is connected.
  {
    if (PS3.getButtonClick(PS)) {
      PS3.disconnect();
      newconnect = 0;
    }
    if (newconnect == 0)                // this is the vibration that you feel when you
      // first connect
    {
      newconnect++;
      //Serial.println("Rumble is on!");
      PS3.moveSetRumble(64);
      PS3.setRumbleOn(100, 255, 100, 255); //VIBRATE!!!
 
#ifdef ROTATION_LOCK
      gyro.getEvent(&rotationReadout);
      desiredRotation = rotationReadout.orientation.x; // setting up our baseline value
#endif
    }
 
    if (state == CALIBRATION)
    { //CALIBRATION MODE
      if (PS3.getButtonClick(UP))
      {
        motorCorrect++;
        PS3.setRumbleOn(5, 255, 5, 0);    //vibrate ON THE LEFT!
      }
      else if (PS3.getButtonClick(DOWN))
      {
        motorCorrect--;
        PS3.setRumbleOn(5, 0, 5, 255);    //vibrate ON THE RIGHT!
      }
      else if (PS3.getButtonClick(SELECT))
      {
        state = DRIVING;
        PS3.setLedRaw(1);                 // OFF OFF OFF ON
        PS3.setRumbleOn(5, 0, 5, 255);    //should vibrate left, then right, then both
        PS3.setRumbleOn(5, 255, 5, 0);
        PS3.setRumbleOn(5, 255, 5, 255);
      }
      if (PS3.getButtonClick(PS)) {
        PS3.disconnect();
        newconnect = 0;
      }
      eStop();
    }
 
#ifdef LED_STRIP
#ifdef TACKLE
    // NORMAL OPERATION MODE
    // for the if statement for whether or not
    // tackle is enabled. cool stuff
    tackled = !digitalRead(TACKLE_INPUT);
 
    if (tackled)
    {
      digitalWrite(RED_LED, HIGH);
      digitalWrite(GREEN_LED, LOW);
      digitalWrite(BLUE_LED, LOW);
      if (!hasIndicatedTackle)                    //Detects if the controller had vibrated when tackled
      {
        PS3.setRumbleOn(10, 255, 10, 255);
        hasIndicatedTackle = true;
      }
    }
    else
    {
      digitalWrite(RED_LED,  LOW);
      digitalWrite(BLUE_LED, LOW);
      digitalWrite(GREEN_LED, HIGH);
      if (hasIndicatedTackle)hasIndicatedTackle = false;
    }
#else
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(BLUE_LED, HIGH);
    digitalWrite(RED_LED, LOW);
#endif
#endif
    if (state == DRIVING || state == KID)
    {
      if (PS3.getButtonClick(SELECT)) //Switch between tank drive and arcade mode. 0 is arcade 1 is tank
      {
        if (PS3.getButtonPress(L1)) {
          if (driveState == ARCADE_MODE) {
            EEPROM.write(0, 1);
            driveState = TANK_MODE;
          }
          else if (driveState == TANK_MODE) {
            EEPROM.write(0, 0);
            driveState = ARCADE_MODE;
          }
        }
        //PS3.setLedRaw(15);                // ON ON ON ON
        //PS3.setRumbleOn(5, 255, 5, 255);  // vibrate both, then left, then right
        //PS3.setRumbleOn(5, 255, 5, 0);
        //PS3.setRumbleOn(5, 0, 5, 255);
        //state = CALIBRATION;
 
      }
      if (PS3.getButtonClick(START))
      { // switches between normal driving mode
        // and kid mode
        if (state == DRIVING)
        {
          state = KID;
          PS3.setLedRaw(9);               // ON OFF OFF ON
          PS3.setRumbleOn(5, 255, 5, 255);// vibrate both, then left, then right
        }
        else if (state == KID)
        {
          state = DRIVING;
          PS3.setLedRaw(1);               // OFF OFF OFF ON
          PS3.setRumbleOn(5, 255, 5, 255);// vibrate both, then left, then right
        }
      }
 
#ifdef QB_PERIPHERALS
      if (PS3.getButtonPress(L1))
      {
        motorReverse = M_PI;              // this is reversed
        turnHandicap = TURN_HANDICAP_AMOUNT;
      }
      else
      {
        motorReverse = 0;
        turnHandicap = 1;
      }
#endif
 
 
#ifdef CENTER_PERIPHERALS
      centerCtrl();
#endif

#ifdef IR_MAST
    if (PS3.getButtonClick(L1))
    {
      toggleServo();
    }
#endif

#ifdef QB_TRACKING
        cameraCapture();
#endif

#ifdef QB_PERIPHERALS
      qbThrowerCtrl();
#endif
 
#ifdef KICKER_PERIPHERALS
      kickerCtrl();
#endif
 
      if (state == DRIVING)
      {
        if (PS3.getButtonPress(R2))
        {
          handicap = ALTERNATE_HANDICAP; // TURBO!!!!!!!!!!!!!!
        }
        else
        {
          #ifdef LR_TACKLE_PERIPHERALS
          handicap = 1;
          #else
          handicap = DEFAULT_HANDICAP;
          #endif
        }
       
      }
      else if (state == KID)
      {
        handicap = KID_HANDICAP;
      }
 
      driveCtrl();
    }
    else
    {
      eStop();
 
#ifdef LED_STRIP
      digitalWrite(RED_LED,   LOW);
      digitalWrite(GREEN_LED, LOW);
      digitalWrite(BLUE_LED, HIGH);
#endif
      PS3.moveSetRumble(64);
      PS3.setRumbleOn(10, 255, 10, 255); // vibrate!
    }
  }
 
  else
  {
    eStop();
#ifdef LED_STRIP
    digitalWrite(RED_LED,   LOW);
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(BLUE_LED, HIGH);
#endif
  }
#ifdef BASIC_DRIVETRAIN
  if (PS3.getButtonClick(R1)) {       //This inverts the driving for tank drive.
    if (inverting == 0) {
      inverting = 1;
      PS3.setRumbleOn(10, 255, 10, 255);
    }
    else if (inverting == 1) {
      inverting = 0;
      PS3.setRumbleOn(10, 255, 10, 255);
    }
  }
#endif
}
 
void eStop()
{
#ifdef BASIC_DRIVETRAIN
  leftMotor.writeMicroseconds(1500);
  rightMotor.writeMicroseconds(1500);
#endif

#ifdef DUAL_MOTORS
  leftMotor2.writeMicroseconds(1500);
  rightMotor2.writeMicroseconds(1500);
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
#ifdef IR_MAST
  setServo(SERVO_DN);
#endif
}
 
void driveCtrl()
{
#ifdef BASIC_DRIVETRAIN
  if (driveState == ARCADE_MODE) {
    yInput = map(PS3.getAnalogHat(LeftHatY), 0, 255, -90, 90);  // Recieves PS3
    // forward/backward input
    xInput = map(PS3.getAnalogHat(RightHatX), 0, 255, 90, -90); // Recieves PS3
    // horizontal input and
    // sets it to an inverted
    // scale of 90 to -90
 
    if (abs(yInput) < 8) yInput = 0;                            // deals with the stickiness
    if (abs(xInput) < 8) xInput = 0;                            // of PS3 joysticks
 
    if ((yInput == 0) && (xInput == 0))
    { // if no input this should ensure that
      // the motors actually stop, and skip the rest
      // of the drive function
      #ifdef DUAL_MOTORS
          leftMotor2.writeMicroseconds(1500);
          rightMotor2.writeMicroseconds(1500);
      #endif
      leftMotor.writeMicroseconds(1500);
      rightMotor.writeMicroseconds(1500);
      return;
    }
 
    // Instead of following some sort of
    // equation to slow down acceleration
    // We just increment the speed by one
    // towards the desired speed.
    // The acceleration is then slowed
    // because of the loop cycle time
 
    if (drive < yInput)drive++;                     // Accelerates
    else if (drive > yInput) drive--;               // Decelerates
 
    if (turn < xInput) turn++;
    else if (turn > xInput) turn--;
 
    throttleL = LEFT_MOTOR_REVERSE * ((drive + turn) / handicap) + motorCorrect;
    // This is the final variable that
    // decides motor speed.
    throttleR = RIGHT_MOTOR_REVERSE * ((drive - turn) / handicap ) + motorCorrect;
 
    if (throttleL > MAX_DRIVE) throttleL = MAX_DRIVE;
    else if (throttleL < -MAX_DRIVE)throttleL = -MAX_DRIVE;
    if (throttleR > MAX_DRIVE) throttleR = MAX_DRIVE;
    else if (throttleR < -MAX_DRIVE)throttleR = -MAX_DRIVE;
 
    leftMotor.write(throttleL + 90);                // Sending values to the speed controllers
    rightMotor.write(throttleR + 90);
    #ifdef DUAL_MOTORS
      leftMotor2.write(throttleL + 90);                // Sending values to the speed controllers
      rightMotor2.write(throttleR + 90);
    #endif
  }
  else if (driveState == TANK_MODE) {
    if (inverting == 0) {
      yInput = map(PS3.getAnalogHat(LeftHatY), 0, 255, -90, 90);  // Recieves PS3
      // forward/backward input
      xInput = map(PS3.getAnalogHat(RightHatY), 0, 255, -90, 90); // Recieves PS3
    }
    else if (inverting == 1) {
     yInput = map(PS3.getAnalogHat(RightHatY), 0, 255, 90, -90);  // Recieves PS3
      // forward/backward input
      xInput = map(PS3.getAnalogHat(LeftHatY), 0, 255, 90, -90); // Recieves PS3
    }
 
    // forward/backward input
    // sets it to an inverted
    // scale of 90 to -90
 
    if (abs(yInput) < 8) yInput = 0;                            // deals with the stickiness
    if (abs(xInput) < 8) xInput = 0;                            // of PS3 joysticks
 
    if ((yInput == 0) && (xInput == 0))
    { // if no input this should ensure that
      // the motors actually stop, and skip the rest
      // of the drive function
      #ifdef DUAL_MOTORS
          leftMotor2.writeMicroseconds(1500);
          rightMotor2.writeMicroseconds(1500);
      #endif
      leftMotor.writeMicroseconds(1500);
      rightMotor.writeMicroseconds(1500);
      return;
    }
 
    // Instead of following some sort of
    // equation to slow down acceleration
    // We just increment the speed by one
    // towards the desired speed.
    // The acceleration is then slowed
    // because of the loop cycle time
 
    if (drive < yInput)drive++;                     // Accelerates
    else if (drive > yInput) drive--;               // Decelerates
 
    if (turn < xInput) turn++;
    else if (turn > xInput) turn--;                //Since this is tank drive it is not actually turning
 
    throttleL = LEFT_MOTOR_REVERSE * ((drive) / handicap) + motorCorrect;
    // This is the final variable that
    // decides motor speed.
    throttleR = RIGHT_MOTOR_REVERSE * ((turn) / handicap ) + motorCorrect;
 
    if (throttleL > MAX_DRIVE) throttleL = MAX_DRIVE;
    else if (throttleL < -MAX_DRIVE)throttleL = -MAX_DRIVE;
    if (throttleR > MAX_DRIVE) throttleR = MAX_DRIVE;
    else if (throttleR < -MAX_DRIVE)throttleR = -MAX_DRIVE;
 
    leftMotor.write(throttleL + 90);                // Sending values to the speed controllers
    rightMotor.write(throttleR + 90);

    #ifdef DUAL_MOTORS
      leftMotor2.write(throttleL + 90);                // Sending values to the speed controllers
      rightMotor2.write(throttleR + 90);
    #endif
  }
#endif
#ifdef OMNIWHEEL_DRIVETRAIN
  yInput = map(PS3.getAnalogHat(LeftHatY), 0, 255, 90, -90);      // Recieves PS3 forward/backward input
  xInput = map(PS3.getAnalogHat(LeftHatX), 0, 255, 90, -90);      // Recieves PS3 horizontal input and
  // sets it to an inverted scale of 90 to -90
  turnInput = map(PS3.getAnalogHat(RightHatX), 0, 255, -MAX_TURN, MAX_TURN);  // received turn input from right joystick
  if (!PS3.getButtonPress(L2))
  {
    if (PS3.getButtonPress(UP))
    {
      yInput = 90;
      xInput = 0;
    }
    else if (PS3.getButtonPress(DOWN))
    {
      yInput = -90;
      xInput = 0;
    }
    else if (PS3.getButtonPress(LEFT))
    {
      yInput = 0;
      xInput = 90;
    }
    else if (PS3.getButtonPress(RIGHT))
    {
      yInput = 0;
      xInput = -90;
    }
  }
  if (abs(yInput) < 8)yInput = 0;                     // taking care of sticky input
  if (abs(xInput) < 8)xInput = 0;
  if (abs(turnInput) < 2)turnInput = 0;               // this one is likely taken care
  // of by integer rounding already
  // rounding
 
  magn = sqrt(pow(xInput, 2) + pow(yInput, 2));       // finding magnitude of input
  //magn = map(magn, 0, float(90) * sqrt(2.0), 0, 60);// need to investigate the purpose of this
  angle = atan2(double(yInput), double(xInput));      // atan2 accounts for quadrants of input
 
 
#ifdef ROTATION_LOCK
  sample++;
  if (PS3.getButtonClick(R3))
  {
    gyro.getEvent(&rotationReadout);
    desiredRotation = rotationReadout.orientation.x;
    rotationCorrect = 0;
    PS3.setRumbleOn(10, 255, 10, 255); //vibrate!
    sample = 0;
  }
 
  if (turnInput)
  {
    rotationCorrect = 0;
    wasIturning = 1;
    sample = 0;
  }
 
  else if ((sample >= SAMPLE_PERIOD))
  {
    sample = 0;
    gyro.getEvent(&rotationReadout);
    int difference = rotationReadout.orientation.x - desiredRotation;
    if ((difference < 0 && difference > -180) || difference > 180)
    { //turning left condition
      difference = -(abs(difference - 360) % 360);
    }
    else                                                //turning right condition
    {
      difference = abs(difference + 360) % 360;
    }
    if (difference > MINIMUM_ANGLE)
    {
      rotationCorrect = -ROTATION_CORRECT_MAGNITUDE;
    }
    else if (difference < -MINIMUM_ANGLE)
    {
      rotationCorrect = ROTATION_CORRECT_MAGNITUDE;
    }
    else
    {
      rotationCorrect = 0;
    }
 
    if (wasIturning)
    {
      if (difference == 0)
      {
        wasIturning = 0;
      }
      else
      {
        rotationCorrect = 0;
      }
    }
  }

  Serial.print(rotationCorrect);
  Serial.print("   ");
  Serial.println(desiredRotation);
 
  Serial.print(rotationCorrect);
  Serial.print("   ");
  Serial.println(desiredRotation);

  motor1Drive += rotationCorrect;
  motor2Drive += rotationCorrect;
  motor3Drive += rotationCorrect;
  motor4Drive += rotationCorrect;

  
#endif

  motor4Drive = ((magn * (sin(angle + PI_OVER_4 + motorReverse)) / handicap)
                + (float)(turnHandicap * turnInput) + 90 + motorCorrect);

  motor1Drive = ((magn * (sin(angle + PI_OVER_4 + PI_OVER_2 + motorReverse)) / handicap)                          
                + (float)(turnHandicap * turnInput) + 90 + motorCorrect);

  motor2Drive = ((magn * (sin(angle + PI_OVER_4 + PI_OVER_2 + PI_OVER_2 + motorReverse)) / handicap)              
                + (float)(turnHandicap * turnInput) + 90 + motorCorrect);

  motor3Drive = ((magn * (sin(angle + PI_OVER_4 + PI_OVER_2 + PI_OVER_2 + PI_OVER_2 + motorReverse)) / handicap)  
                + (float)(turnHandicap * turnInput) + 90 + motorCorrect);

#ifdef QB_TRACKING
  motor1Drive -= aimingFactor;
  motor2Drive -= aimingFactor;
  motor3Drive -= aimingFactor;
  motor4Drive -= aimingFactor;
#endif

  if (motor1Drive < 5)motor1Drive = 5;
  else if (motor1Drive > 175)motor1Drive = 175;
  if (motor2Drive < 5)motor2Drive = 5;
  else if (motor2Drive > 175)motor2Drive = 175;
  if (motor3Drive < 5)motor3Drive = 5;
  else if (motor3Drive > 175)motor3Drive = 175;
  if (motor4Drive < 5)motor1Drive = 5;
  else if (motor4Drive > 175)motor4Drive = 175;
 
  if (motor1Drive > motor1Input)motor1Input++;
  else if (motor1Drive < motor1Input)motor1Input--;
  if (motor2Drive > motor2Input)motor2Input++;
  else if (motor2Drive < motor2Input)motor2Input--;
  if (motor3Drive > motor3Input)motor3Input++;
  else if (motor3Drive < motor3Input)motor3Input--;
  if (motor4Drive > motor4Input)motor4Input++;
  else if (motor4Drive < motor4Input)motor4Input--;
 
  motor1.write(motor1Input);
  motor2.write(motor2Input);
  motor3.write(motor3Input);
  motor4.write(motor4Input);
 
#endif
}
 
#ifdef QB_PERIPHERALS
void qbThrowerCtrl()
{

  
  if (PS3.getButtonPress(L2))
  {
    if (PS3.getButtonClick(UP))
    {
      throwOffset += 2;
      PS3.setRumbleOn(5, 255, 5, 0); //vibrate on the left!
    }
    else if (PS3.getButtonClick(DOWN))
    {
      throwOffset -= 2;
      PS3.setRumbleOn(5, 0, 5, 255); //vibrate on the right!
    }
  }
 
  if (PS3.getButtonPress(TRIANGLE))    qbThrower.write(TRIANGLE_THROW);
#ifdef QB_TRACKING
  else if(PS3.getButtonPress(CIRCLE) && (isWRTracking == true)) qbThrower.write(calcThrow); // throw the auto targetted distance
#endif
  else if (PS3.getButtonPress(CIRCLE)) qbThrower.write(CIRCLE_THROW + throwOffset);
  else if (PS3.getButtonPress(CROSS))  qbThrower.write(CROSS_THROW + throwOffset);
  else if (PS3.getButtonPress(SQUARE)) qbThrower.write(SQUARE_THROW + throwOffset) ;
  else if (PS3.getButtonPress(R1))
  {
    qbThrower.write(RELOAD_THROW);
    throwOffset = 0;
  }
  else qbThrower.writeMicroseconds(1500);
}
#endif

#ifdef QB_TRACKING
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
  // This appears to be taken from here https://www.dfrobot.com/wiki/index.php/Positioning_ir_camera
  // also here 							 https://blog.squix.org/2016/05/esp8266-peripherals-indoor-positioning-with-ir-camera.html
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
  //digitalWrite(BLUE_LED, LOW);
  for (i=0; i < 4; i++)
  {
    if (CamX[i] != 1023 && CamX[i] != 0)
    {
      isWRSeen = true;
      //digitalWrite(BLUE_LED,HIGH);
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
        calcThrow = calcThrow+100;
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
#endif

#ifdef QB_TRACKING
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
  if (PS3.getButtonPress(CROSS)) kicker.write(KICKER_POWER);
  else if (PS3.getButtonPress(TRIANGLE)) kicker.write(KICKER_RELOAD);
  else kicker.writeMicroseconds(1500);
}
#endif
 
#ifdef LED_STRIP
void flashLed()
{
  //flash the leds
  digitalWrite(RED_LED, LOW);
  digitalWrite(BLUE_LED, LOW);
  digitalWrite(GREEN_LED, HIGH);
  delay(300);
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(BLUE_LED, HIGH);
  delay(300);
  digitalWrite(BLUE_LED, LOW);
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(RED_LED, HIGH);
  delay(300);
  digitalWrite(RED_LED, LOW);
  digitalWrite(BLUE_LED, LOW);
  digitalWrite(GREEN_LED, HIGH);
  delay(300);
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(BLUE_LED, HIGH);
}
#endif
 
#ifdef CENTER_PERIPHERALS
void centerCtrl()
{
  if (PS3.getButtonClick(TRIANGLE)) centerRelease.write(CENTER_RELEASE_UP);
  else if (PS3.getButtonClick(CROSS)) centerRelease.write(CENTER_RELEASE_DOWN);
}
#endif

#ifdef IR_MAST
void setServo(bool state)
{
  if(state == servoState) return; // requesting servo move to it's existing position
  else if (SERVO_DN == state)
  {
    irMast.write(SERVO_DN_POS);
    servoState = SERVO_DN;
  }
  else if (SERVO_UP == state)
  {
    irMast.write(SERVO_UP_POS);
    servoState = SERVO_UP;
  }
}

void toggleServo()
{
  if (SERVO_UP == servoState)
  {
    irMast.write(SERVO_DN_POS);
    servoState = SERVO_DN;
  }
  else if (SERVO_DN == servoState)
  {
    irMast.write(SERVO_UP_POS);
    servoState = SERVO_UP;
  }
}
#endif
 
//Error handling for what parts of the code are enabled
#ifdef BASIC_DRIVETRAIN
#ifdef OMNIWHEEL_DRIVETRAIN
#error Two drivetrains are enabled!
#endif
#ifdef ROTATION_LOCK
#error Rotation lock is not normally used with a basic drivetrain...
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
#ifdef KICKER_PERIPHERALS
#error Multiple peripherals enabled!
#endif
#endif
#endif
 
#ifdef OMNIWHEEL_DRIVETRAIN
#ifdef DUAL_MOTORS
#error dual motors with an omniwheel? perposterous!
#endif
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

#ifdef QB_TRACKING
#ifndef QB_PERIPHERALS
#error You enabled QB tracking, but no QB peripherals!
#endif
#endif

#ifndef BASIC_DRIVETRAIN
#ifndef OMNIWHEEL_DRIVETRAIN
#warning "You don't have a drivetrain enabled! Don't expect this robot to drive!"
#endif
#endif
