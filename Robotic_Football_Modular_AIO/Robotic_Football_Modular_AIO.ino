#include "src/USB_Host_Shield_Library_20/PS3BT.h"
#include "src/USB_Host_Shield_Library_20/usbhub.h"

//===========Uncomment a LED===========================
#include "Leds/Leds.cpp"
//#include "Leds/OldLeds.cpp"

//===========Uncomment a drive train===================
#include "DriveTrains/BasicDrive.cpp"
//#include "DriveTrains/CenterDrive.cpp"
//#include "DriveTrains/SquareOmniDrive.cpp"

//===========Uncomment for tackle sensor===============
#define TACKLE

//===========Uncomment to choose a Position============
//#define WR
//#define Center
//#define QB
//#define Kicker

//===========Uncomment if not using bag motors=========
//#define OldMotors

//===========Uncomment for debug modes=================
//#define SHOW_CONTROLLER_INPUT
//#define SHOW_EXECUTION_TIME

//=====================================================
//Includes the right peripheral file for specified position
#ifdef WR
  #define PERIPHERAL
  #include "Peripherals/WRPeripheral.cpp"
#endif

#ifdef Center
  #define PERIPHERAL
  #include "Peripherals/CenterPeripheral.cpp"
#endif

#ifdef QB
  #define PERIPHERAL
  #include "Peripherals/QBPeripheral.cpp"
#endif

#ifdef Kicker
  #define PERIPHERAL
  #include "Peripherals/KickerPeripheral.cpp"
#endif

//This just enables and disables the old motors
#ifdef OldMotors
  int motorType = -1;
#else
  int motorType = 1;
#endif
//=====================================================

#define TACKLE_INPUT 6   // Tackle sensor is wired to pin 6
bool hasIndicated = false;
bool stayTackled = false;
int handicap = 3;
bool kidsMode = false;
int newconnect = 0;
int leftX, leftY, rightX, rightY;
float time;
USB Usb;
USBHub Hub1(&Usb);
BTD Btd(&Usb);
PS3BT PS3(&Btd);


void setup() {
  driveSetup(motorType);    //Setup the drive train
  ledsSetup();    //Setup the leds
  flashLeds();    //flash the leds

  #ifdef PERIPHERAL
    peripheralSetup();    //Call the peripheral setup
  #endif

  int newconnect = 0;   // Variable(boolean) for connection to ps3, also activates rumble

  #ifdef TACKLE
    pinMode(TACKLE_INPUT, INPUT);   // Define the tackle sensor pin as an input
  #endif

  Serial.begin(115200);    //Begin Serial Communications
  
  if (Usb.Init() == -1)    // This is for an error message with USB connections
  {
    Serial.print(F("\r\nOSC did not start"));
    while (1);
  }
  Serial.print(F("\r\nPS3 Bluetooth Library Started"));
}


void loop() {
  Usb.Task();   // This updates the input from the PS3 controller
  
  // Run if the controller is connected
  if (PS3.PS3Connected){
    // Changes the LEDs to green and vibrates controller when you first connect
    if (newconnect == 0) {    
      #ifdef TACKLE
        green();
      #else
        blue();
      #endif

      PS3.moveSetRumble(64);
      PS3.setRumbleOn(100, 255, 100, 255); //VIBRATE!!!
      newconnect++;
    }

    #ifdef SHOW_EXECUTION_TIME
      time = micros();
    #endif

    // Press the PS button to disconnect the controller
    if (PS3.getButtonClick(PS)) {
      PS3.disconnect();
      newconnect = 0;
    }

    //====================Get Controller Input=================================
    // Reads and maps joystick values from -90 to 90
    leftX = map(PS3.getAnalogHat(LeftHatX), 0, 255, -90, 90);
    leftY = map(PS3.getAnalogHat(LeftHatY), 0, 255, -90, 90);
    rightX = map(PS3.getAnalogHat(RightHatX), 0, 255, -90, 90);
    rightY = map(PS3.getAnalogHat(RightHatY), 0, 255, -90, 90);

    // Deals with stickness from joysticks
    if (abs(leftX) < 8) leftX = 0;
    if (abs(leftY) < 8) leftY = 0;
    if (abs(rightX) < 8) rightX = 0;
    if (abs(rightY) < 8) rightY = 0;
      
    #ifdef SHOW_CONTROLLER_INPUT
      Serial.print(leftX);    
      Serial.print("\t");
      Serial.print(leftY);    
      Serial.print("\t");
      Serial.print(rightX);    
      Serial.print("\t");
      Serial.print(rightY);    
      Serial.print("\t");
    #endif

    //====================Specify the handicap=================================
    //Toggle in and out of kidsmode
    if (PS3.getButtonClick(START)) {
      if (kidsMode == true) {
        kidsMode = false;
        handicap = 3;
        PS3.setLedRaw(1);               // ON OFF OFF ON
        PS3.setRumbleOn(5, 255, 5, 255);// vibrate both, then left, then right
      } else if (kidsMode == false) {
        kidsMode = true;
        handicap = 7;
        PS3.setLedRaw(9);               // OFF OFF OFF ON
        PS3.setRumbleOn(5, 255, 5, 255);// vibrate both, then left, then right
      }
    }

    if (kidsMode == false) {
      // Press R2 to boost
      if (PS3.getButtonPress(R2)) {
        handicap = 1;
      }
      // Press L2 to slow down
      else if (PS3.getButtonPress(L2)) {
        handicap = 6;
      }
      // Sets default handicap
      else {
        handicap = 3;
      }
    }

    //===============================Adjust motors=============================
    if (PS3.getButtonClick(LEFT)){
      correctMotor(1);
      Serial.println("Left button clicked");
    }
    if (PS3.getButtonClick(RIGHT)){
      correctMotor(-1);
      Serial.println("right button clicked");
    }

    //=================================Tackle Sensor===========================
    #ifdef TACKLE
      // NORMAL OPERATION MODE
      // for the if statement for whether or not
      // tackle is enabled. cool stuff
      if (PS3.getButtonClick(LEFT)) {
        if (stayTackled == true) {
          stayTackled = false;
        } 
        else {
          stayTackled = true;
        }
        PS3.setRumbleOn(30, 255, 30, 255);
      }

      if (!digitalRead(TACKLE_INPUT)) {
        red();
        if (!hasIndicated) {
          PS3.setRumbleOn(10, 255, 10, 255);
          hasIndicated = true;
        }
      }
      else {
        if (stayTackled == false) {
          hasIndicated = false;
          green();
        }
      }
    #endif

    // Drives the robot according to joystick input
    driveCtrl(handicap, leftX, leftY, rightX, rightY);

    #ifdef PERIPHERAL
      peripheral(PS3);    //Call the peripheral
    #endif
    
    #ifdef SHOW_EXECUTION_TIME
      Serial.print("Exe time: ");
      Serial.print(micros() - time);
      Serial.print("\t");
    #endif
  }

  // If the controller is not connected, LEDs blue and stop robot
  if (!PS3.PS3Connected) {
    blue();
    driveStop();
  }
}
