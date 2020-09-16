#include <Ps3Controller.h>
#include <ESP32Servo.h>

//===========Uncomment a LED===========================
#include "Leds/Leds.cpp"
//#include "Leds/OldLeds.cpp"

//===========Uncomment a drive train===================
#include "DriveTrains/BasicDrive.cpp"
//#include "DriveTrains/CenterDrive.cpp"
//#include "DriveTrains/SquareOmniDrive.cpp"

//===========Uncomment for tackle sensor===============
//
#define TACKLE
//===========Uncomment to choose a Peripheral==========
//#define PERIPHERAL
//#include "Peripherals/WRPeripheral.cpp"
//#include "Peripherals/CenterPeripheral.cpp"
//#include "Peripherals/QBPeripheral.cpp"
//#include "Peripherals/KickerPeripheral.cpp"

//==========Uncomment if not using bag motors==========
//#define OldMotors

//This just enables and disables the old motors
#ifdef OldMotors
int motorType = -1;
#else
int motorType = 1;
#endif
//===================================

#define TACKLE_INPUT    35           // Tackle sensor is wired to pin 6
bool hasIndicated = false;
bool stayTackled = false;
int handicap = 3;
bool kidsMode = false;
int newconnect = 0;
int leftX, leftY, rightX, rightY;
ps3_cmd_t cmd;

void setup() {// This is stuff for connecting the PS3 controller.
  Ps3.begin("00:1b:dc:0f:f3:59");
  driveSetup(motorType);//Setup the drive train
  ledsSetup();          //Setup the leds
  flashLeds();          //flash the leds


#ifdef PERIPHERAL
  peripheralSetup();//Call the peripheral setup
#endif

  int newconnect = 0;         // Variable(boolean) for connection to ps3, also activates rumble

#ifdef TACKLE
  pinMode(TACKLE_INPUT, INPUT); // define the tackle sensor pin as an input
#endif
  Serial.begin(9600);       //Begin Serial Communications
  /*
    if (Usb.Init() == -1)       // this is for an error message with USB connections
    {
    Serial.print(F("\r\nOSC did not start"));
    while (1);
    }
    Serial.print(F("\r\nPS3 Bluetooth Library Started"));
  */
}


void loop() {

  //  Usb.Task();                           // This updates the input from the PS3 controller
  if (Ps3.isConnected())                 // run if the controller is connected
  {
    if (newconnect == 0)                // this is the vibration that you feel when you first connect
    {
#ifdef TACKLE
      green();
#else
      blue();
#endif
      cmd.rumble_left_intensity = 0xff;
      cmd.rumble_right_intensity = 0xff;
      cmd.rumble_right_duration = 750;
      cmd.rumble_left_duration = 750;
      newconnect++;
      ps3Cmd(cmd);
    }
    if (Ps3.data.button.ps && newconnect == 1) {
      newconnect = 0;

    }
    //========================================Get Controller Input==========================================
    leftX = map(Ps3.data.analog.stick.lx, -128, 127, -90, 90);    // Recieves PS3
    leftY = map(Ps3.data.analog.stick.ly, -128, 127, -90, 90);    // Recieves PS3
    rightX = map(Ps3.data.analog.stick.rx, -128, 127, -90, 90);   // Recieves PS3
    rightY = map(Ps3.data.analog.stick.ry, -128, 127, -90, 90);   // Recieves PS3
    if (abs(leftX) < 8) leftX = 0;                             // deals with the stickiness
    if (abs(leftY) < 8) leftY = 0;
    if (abs(rightX) < 8) rightX = 0;

    if (Ps3.data.button.start) { //Toggle in and out of kidsmode
      if (kidsMode == true) {
        kidsMode = false;
        handicap = 3;
        //PS3.setLedRaw(1);               // ON OFF OFF ON
        //PS3.setRumbleOn(5, 255, 5, 255);// vibrate both, then left, then right
      } else if (kidsMode == false) {
        kidsMode = true;
        handicap = 7;
        //PS3.setLedRaw(9);               // OFF OFF OFF ON
        //PS3.setRumbleOn(5, 255, 5, 255);// vibrate both, then left, then right
      }
    }
    if (kidsMode == false) {
      if (Ps3.data.button.r2) { //Boost
        handicap = 1;
      }
      else if (Ps3.data.button.l2) { //Slow Down
        handicap = 6;
      } else {
        handicap = 3;
      }
    }
    //==========================================================================

    //=================================Tackle Sensor================================
#ifdef TACKLE
    // NORMAL OPERATION MODE
    // for the if statement for whether or not
    // tackle is enabled. cool stuff
    if (Ps3.data.button.left) {
      if (stayTackled == true) {
        stayTackled = false;
      } else {
        stayTackled = true;
      }
      /*
        Ps3.data.cmd.rumble_right_duration(30);
        Ps3.data.cmd.rumble_right_intensity(255);
        Ps3.data.cmd.rumble_left_duration(30);
        Ps3.data.cmd.rumble_left_intensity(255);
      */
      //PS3.setRumbleOn(30, 255, 30, 255);
    }
    if (!digitalRead(TACKLE_INPUT))
    {
      red();
      if (!hasIndicated) {
        //PS3.setRumbleOn(10, 255, 10, 255);
        hasIndicated = true;
      }
    }
    else
    {
      if (stayTackled == false) {
        hasIndicated = false;
        green();
      }
    }
#endif
    //===============================================================================================

    driveCtrl(handicap, leftX, leftY, rightX, rightY);//Drive the drive train

#ifdef PERIPHERAL
    peripheral(Ps3);//Call the peripheral
#endif
  }
  if (!Ps3.isConnected()) {
    blue();
    driveStop();
  }
}
