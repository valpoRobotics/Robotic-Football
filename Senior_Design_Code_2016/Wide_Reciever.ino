
//Quarterback Control Code 
//Original version was created by Jason Toberman, Chase Greenhagen, and Hayden Hast
//This code is written for an Arduino Mega ADK with a Bluetooth USB Dongle in order
//to connect to the Playstation Controler. Make sure to have the correct libraries
//saved on your computer. 

//This version was created by Tim Krentz


/* ROBOT LAYOUT- BIRD'S EYE VIEW
 *  _______________________________________
 *  |                FRONT                |
 *  | MOTOR 3          ^          MOTOR 2 |
 *  |  PIN 9          / \          PIN 10 |
 *  |                  |                  |
 *  |                  |                  |
 *  |                  |                  |
 *  |                                     |
 *  |                                     |
 *  |                                     |
 *  | MOTOR 4                     MOTOR 1 |
 *  |  PIN 8                        PIN 7 |
 *  |                 BACK                |
 *  ---------------------------------------
*/

//Libraries Included
#include "PS3BT.h"
#include "usbhub.h"
#include <Servo.h>
#include <math.h>

#define HANDICAP 5 //The amount the motor speed is divided by

//change pin inputs here
#define MOTOR1 7
#define MOTOR4 8 
#define MOTOR3 9
#define MOTOR2 10
#define MAST 3
#define TACKLE_PIN 11



//Assign All Variables
Servo motor2; //Left motor
Servo motor4; //Right motor
Servo motor1; //Front motor
Servo motor3; //Back motor
Servo arm1; //Arm motor 
int newconnect = 0; //Variable(boolean) for connection to ps3, also activates rumble
int Drive = 0; //Initial speed before turning calculations
int Turn = 0; //Turn is adjustment to drive for each motor separately to create turns
int Tackle = 1; //Tackle detects if the robot has been tackled
int motorCorrect = 3; // 3 instead of 1 to stop two button up calibration press at beginning 
int motorCorrecty = 1; // Used to help motor correct
int motorCorrectx = 1; // used to help motor correct
int Throw = 1; // Throwing Value to the arm
int circleCorrect = 0; // Value used to vary the circle throwing distance
int crossCorrect = 0; // Value used to vary the cross thowing distnace
bool ledStrip = true;
bool mastState = false;

//This is stuff for connecting the PS3 to USB. WE HAVE NO IDEA HOW THIS WORKS
USB Usb;
USBHub Hub1(&Usb);
BTD Btd(&Usb);
PS3BT PS3(&Btd);

void setup() {
  //Assign motor pin outs
  motor1.attach(MOTOR1,1000,2000);
  motor2.attach(MOTOR2,1000,2000);
  motor3.attach(MOTOR3,1000,2000);
  motor4.attach(MOTOR4,1000,2000);
  
  //Define Pin Mode of Tackle Input Pin
  pinMode(TACKLE_PIN, INPUT);

  //Define Pin Mode of Mast Output Pin
  pinMode(MAST, OUTPUT);
  
  // Initialize LED Strip
//  strip.begin();
//  strip.show(); // Initialize all pixels to 'off'
  
  // Helps Motors not freak out at the begining
  motor1.writeMicroseconds(1500); // keep bot from going full terminator at start up #CODY ie set motors to not mov e
  motor2.writeMicroseconds(1500); // keep bot from going full terminator at start up #CODY ie set motors to not move
  motor3.writeMicroseconds(1500); // keep bot from going full terminator at start up #CODY ie set motors to not mov e
  motor4.writeMicroseconds(1500); // keep bot from going full terminator at start up #CODY ie set motors to not move
//  arm1.writeMicroseconds(1500); // keep bot from going full terminator at start up #CODY ie set motors to not move
//  led_startup_sequence();
  
  //Begin Serial Communications
  Serial.begin(115200);
  if(Usb.Init() == -1) 
  { //I think this is for an error message with USB connections
    Serial.print(F("\r\nOSC did not start"));
    while(1);
  }
  Serial.print(F("\r\nPS3 Bluetooth Library Started"));
}

void loop()
{
  Usb.Task(); //This does something USB related

  if(PS3.PS3Connected || PS3.PS3NavigationConnected) //This only lets the program run if the PS3 controller is connected.
  {
    if(newconnect == 0)
    {
      newconnect++;
      Serial.println("Rumble is on!");
      PS3.moveSetRumble(64);
      PS3.setRumbleOn(100,255,100,255); //VIBRATE!!!
    }
   Tackle = digitalRead(TACKLE_PIN);  //Reads value from tackle sensor
   
   if(Tackle ==1){ //if robot is not tackled, continue to move
   
     // Define Angles
     float FortyFive = M_PI*45.0/180.0;
     float Ninety = M_PI*90.0/180.0;
     
//     //turn LED Strip Green
//     if(ledStrip == true)
//     {
//        motor1.write(92);
//        motor2.write(92);
//        motor3.write(92);
//        motor4.write(92);
//        motor1.writeMicroseconds(192000000); // keep bot from going full terminator at start up #CODY ie set motors to not mov e
//        motor2.writeMicroseconds(192000000); // keep bot from going full terminator at start up #CODY ie set motors to not move
//        motor3.writeMicroseconds(192000000); // keep bot from going full terminator at start up #CODY ie set motors to not mov e
//        motor4.writeMicroseconds(192000000); // keep bot from going full terminator at start up #CODY ie set motors to not move
//        arm1.writeMicroseconds(192000000); // keep bot from going full terminator at start up #CODY ie set motors to not move
//        delayMicroseconds(1000);
//        
//        // Turn LED Strip Green 
////        colorGreen(); // Green 
//     }
//     ledStrip = false; // Update Variable
     
     
     float yOldInput = map(PS3.getAnalogHat(LeftHatY), 0, 255, 90, -90); //Recieves PS3 forward/backward input
     float xOldInput = map(PS3.getAnalogHat(LeftHatX), 0, 255, 90, -90); //Recieves PS3 horizontal input and sets it to an inverted scale of 90 to -90
     float magn = sqrt((xOldInput)*(xOldInput) + (yOldInput)*(yOldInput));
     float theta = atan2(double(yOldInput),double(xOldInput));
     int TurnInput = map(PS3.getAnalogHat(RightHatX), 0, 255, -7,7);//Receives turn value from right joystick
     magn = map(magn, 0, float(90)*sqrt(2.0), 0, 60);
     
     float yvalue =  map(PS3.getAnalogHat(LeftHatY), 0, 255, 0, 180); //Recieves PS3 horizontal input and sets it to an inverted scale o
     
     // This section is to adjust the center off value of the motors
     if(PS3.getButtonClick(UP)) motorCorrect++;
     if(PS3.getButtonClick(DOWN)) motorCorrect--;


     

     
     // This section is to adjust the values of the Circle and Cross
     if(PS3.getButtonClick(LEFT)) circleCorrect--;
     if(PS3.getButtonClick(RIGHT)) circleCorrect++;
     if(PS3.getButtonClick(L2)) crossCorrect--;
     if(PS3.getButtonClick(R2)) crossCorrect++;
//     if(PS3.getButtonClick(L1)) 
//     {
//        crossCorrect=0;
//        circleCorrect=0;
//     }
     
     // This section is for throwing the football or handing off
     if(PS3.getButtonPress(TRIANGLE))Throw = 178;
     else if(PS3.getButtonPress(CIRCLE))Throw = 132 + circleCorrect;
     else if(PS3.getButtonPress(CROSS))Throw = 118 + crossCorrect;
     else if(PS3.getButtonPress(SQUARE))Throw = 101;
     else if(PS3.getButtonPress(R1))Throw = 88;
     else Throw = 90 + motorCorrect;

     int motor4Drive = (magn*(sin(theta+FortyFive))+TurnInput + 90 + motorCorrect);
     int motor1Drive = (magn*(sin(theta + Ninety + FortyFive))+TurnInput+ 90+ motorCorrect);
     int motor2Drive = (magn*(sin(theta + FortyFive+Ninety+Ninety))+TurnInput+ 90+ motorCorrect);
     int motor3Drive = (magn*(sin(theta + FortyFive+Ninety+Ninety+Ninety))+TurnInput+ 90+ motorCorrect);
     
     
     //limit power range of Motors
     if (motor3Drive < 0)motor3Drive = 0;
     if (motor3Drive > 180)motor3Drive = 180;
     if (motor1Drive < 0)motor1Drive = 0;
     if (motor1Drive > 180)motor1Drive = 180;
     if (motor2Drive < 0)motor2Drive = 0;
     if (motor2Drive > 180)motor2Drive = 180;
     if (motor1Drive < 0)motor1Drive = 0;
     if (motor1Drive > 180)motor1Drive = 180;
     
     if (yvalue < 0)motor1Drive = 0;
     if (yvalue > 180)motor1Drive = 180;
     
     // Write Final Values to Motor Controller
     motor1.write(motor1Drive);
     motor3.write(motor3Drive);
     motor4.write(motor4Drive);
     motor2.write(motor2Drive);
//     arm1.write(Throw);

     //Serial.println("on\n");
   }
   
   else // if tackle detected don't move
   {
     motor1.write(92);
     motor2.write(92);
     motor3.write(92);
     motor4.write(92);
     
     //Serial.println("off\n");
     //turn on blue led for two seconds
     if (ledStrip == false )
     {
        motor1.writeMicroseconds(1500); // keep bot from going full terminator at start up #CODY ie set motors to not mov e
        motor2.writeMicroseconds(1500); // keep bot from going full terminator at start up #CODY ie set motors to not move
        motor3.writeMicroseconds(1500); // keep bot from going full terminator at start up #CODY ie set motors to not mov e
        motor4.writeMicroseconds(1500); // keep bot from going full terminator at start up #CODY ie set motors to not move
//        arm1.writeMicroseconds(192000000); // keep bot from going full terminator at start up #CODY ie set motors to not move
        delayMicroseconds(1000);
//        colorBlue();

     }
     ledStrip=true;
     
     //Serial.println("Rumble is on!");
     PS3.moveSetRumble(64);
     PS3.setRumbleOn(5, 255, 5, 255); //VIBRATE!!!
   }
   
  }
}
