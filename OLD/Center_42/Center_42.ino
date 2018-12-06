
//Basic Drive Code for Robots with Victor Pro 888 or 885
//This version was created by Jason Toberman, Aaron Roggow and Hayden Hast
//This code is written for an Arduino Mega ADK with a Bluetooth USB Dongle in order
//to connect to the Playstation Controler. Make sure to have the correct libraries
//saved on your computer.

/*
  Pin 8: Tackle Input
  Pin 9: Servo 1 -- Left Motor
  Pin 10: Servo 2 -- Right Motor
  Pin 11: Servo 3 -- Center Motor
*/

//Include Libraries
#include <PS3BT.h>
#include <usbhub.h>
#include <Servo.h>

//If one of the motors is spinning when stopped, adjust it's offset here
#define LEFT_ADJUST 0
#define RIGHT_ADJUST 0
#define LEFT_FLIP -1
#define RIGHT_FLIP 1

#define TURBO 1
#define HANDICAP 2 //The amount the motor speed is divided by
#define KID_HANDICAP 3
int currentHandicap = HANDICAP;
bool kidMode = false;

//change pin inputs here
#define BLUELED 13
#define GREENLED 12
#define LEFT_MOTOR 9
#define RIGHT_MOTOR 10
#define CENTER_MOTOR 11
#define TACKLE_PIN 6

// Define Variables
Servo leftMotor; //Left motor
Servo rightMotor; //Right motor
Servo centerMotor; //Center motor

int newconnect = 0; //Variable(boolean) for connection to ps3, also activates rumble
int Drive = 0; //Initial speed before turning calculations
int Turn = 0; //Turn is adjustment to drive for each motor separately to create turns
int Tackle = 1; //Tackle detects if the robot has been tackled
int motorCorrect = 0; //This will help center the stop value of the motors
bool ledStrip = true;

//This is stuff for connecting the PS3 to USB.
USB Usb;
USBHub Hub1(&Usb);
BTD Btd(&Usb);
PS3BT PS3(&Btd);

void setup() {
  //Assign motor pin outs
  leftMotor.attach(LEFT_MOTOR, 1000, 2000);
  rightMotor.attach(RIGHT_MOTOR, 1000, 2000);
  centerMotor.attach(CENTER_MOTOR);
  centerMotor.write(70);
  // Set the LED pins as outputs
  pinMode(BLUELED, OUTPUT);
  pinMode(GREENLED, OUTPUT);
  // Set the LED pins into intial untackled state
  digitalWrite(GREENLED, LOW);
  digitalWrite(BLUELED, HIGH);
  delay(500);
  digitalWrite(BLUELED, LOW);
  digitalWrite(GREENLED, HIGH);
  delay(500);
  digitalWrite(GREENLED, LOW);
  digitalWrite(BLUELED, HIGH);
  // Set up the Tackle pin as an input
  pinMode(TACKLE_PIN, INPUT);
  //strip.begin();
  //strip.show(); // Initialize all pixels to 'off'

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
  Usb.Task(); //This does something USB related.

  if (PS3.PS3Connected || PS3.PS3NavigationConnected) //This only lets the program run if the PS3 controller is connected.
  {
    if (newconnect == 0)
    {
      newconnect++;
      Serial.println("Rumble is on!");
      PS3.moveSetRumble(64);
      PS3.setRumbleOn(100, 255, 100, 255); //VIBRATE!!!
      ledStrip = true;
    }
    if (PS3.getButtonClick(PS)) {
      PS3.disconnect();
      newconnect = 0;
    }

    if(PS3.getButtonClick(START))
    {
      kidMode = !kidMode;
      if(kidMode)
      {
        PS3.setLedRaw(9);
        currentHandicap = KID_HANDICAP;
      }
      else
      {
        PS3.setLedRaw(1);
        currentHandicap = HANDICAP;
      }
    }
    
    
    Tackle = digitalRead(TACKLE_PIN);  //Reads value from tackle sensor

    if (Tackle == 1) //if robot is not tackled, continue to move
    {
      int yInput = map(PS3.getAnalogHat(LeftHatY), 0, 255, -90, 90); //Recieves PS3 forward/backward input
      int xInput = map(PS3.getAnalogHat(RightHatX), 0, 255, 90, -90); //Recieves PS3 horizontal input and sets it to an inverted scale of 90 to -90
      // turn LED's into tackled state
      digitalWrite(BLUELED, LOW);
      digitalWrite(GREENLED, HIGH);

      if (abs(yInput) < 8)yInput = 0;
      if (abs(xInput) < 8)xInput = 0;

      //Instead of following some sort of equation to slow down acceleration
      //We just increment the speed by one towards the desired speed.
      //The acceleration is then slowed because of the loop cycle time
      if (Drive < yInput)Drive++; //Accelerates
      else if (Drive > yInput) Drive--; //Decelerates

      if (Turn < xInput) Turn++;
      else if (Turn > xInput) Turn--;

      // Helps get the robot to drive straigt
      if (PS3.getButtonClick(UP)) motorCorrect++;
      if (PS3.getButtonClick(DOWN)) motorCorrect--;

      if (PS3.getButtonClick(CROSS)) // Cross will lower the servo/ snap the ball
      {
        centerMotor.write(120);
      }
      if (PS3.getButtonClick(TRIANGLE)) // Triangle will raise the servo/ hold the ball
      {
        centerMotor.write(70);
      }

      if (PS3.getButtonPress(R2)&&!kidMode)
      {
        currentHandicap = TURBO;
      }
      else if(!kidMode) currentHandicap = HANDICAP;

      int ThrottleL = LEFT_FLIP * ((Drive + Turn) / currentHandicap); //This is the final variable that decides motor speed.
      int ThrottleR = RIGHT_FLIP * ((Drive - Turn) / currentHandicap);

      if (ThrottleL > 90) ThrottleL = 90;
      if (ThrottleR > 90) ThrottleR = 90;

      leftMotor.write((ThrottleL + 90 + motorCorrect)); //Sending values to the speed controllers
      rightMotor.write((ThrottleR + 90 + motorCorrect));
    }

    else // if tackle detected don't move
    {
      leftMotor.writeMicroseconds(1500);
      rightMotor.writeMicroseconds(1500);
      Serial.println("off\n");
      Serial.println("Rumble is on!");
      PS3.moveSetRumble(64);
      PS3.setRumbleOn(10, 255, 10, 255); //VIBRATE!!!
      //turn on blue led for two seconds
      digitalWrite(GREENLED, LOW);
      digitalWrite(BLUELED, HIGH);
    }
  }
  else // if tackle detected don't move
    {
      leftMotor.writeMicroseconds(1500);
      rightMotor.writeMicroseconds(1500);
      //turn on blue led for two seconds
      digitalWrite(GREENLED, LOW);
      digitalWrite(BLUELED, HIGH);
    }
}

