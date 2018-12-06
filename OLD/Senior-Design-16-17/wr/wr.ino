/*
  Software developed for use on Wide Reciever Gen 3
  Developed by Aaron Roggow(17) and John White(17)
  
  Change log:
  Date                   Name                 Notes
  ======                 =====                =====
  JAN/19/17               AWR                 code adapted from Kicker progress from Jan/19/17
  JAN/21/17               AWR                 added servo functionality, reversable drive
*/

#include <PS3BT.h>
#include <usbhub.h>
#include <Servo.h>

//#define DEBUG

//change pin inputs here
#define REDLED        11
#define BLUELED       12
#define GREENLED      13
#define LEFT_MOTOR    9
#define RIGHT_MOTOR   10
#define SERVO_MOTOR   8
#define IRLED         7


#define LEFT_FLIP -1
#define RIGHT_FLIP 1
#define DEADZONE 8
#define SERVO_UP_POS 120
#define SERVO_DN_POS 30
#define SERVO_UP true
#define SERVO_DN false
bool servoState = SERVO_DN;
#define FORWARD 1
#define BACKWARD -1
int bowDirection = FORWARD;

#define LED_STATUS_FORWARD  1
#define LED_STATUS_BACKWARD 9
#define LED_STATUS_KIDMODE  15

#define TURBO         1
#define HANDICAP      3           //The amount the motor speed is divided by
#define KID_HANDICAP  7
int currentHandicap = HANDICAP;
bool kidMode = false;

Servo leftMotor;                  //Left motor
Servo rightMotor;                 //Right motor
Servo servoMotor;                 //Center motor

int newconnect = 0;               //Variable(boolean) for connection to ps3, also activates rumble
int Drive = 0;                    //Initial speed before turning calculations
int Turn = 0;                     //Turn is adjustment to drive for each motor separately to create turns

int motorCorrect = 0;             //This will help center the stop value of the motors
#define TURN_LIMITER 0.6

#define RED   1
#define GREEN 2
#define BLUE  3
int ledColor = BLUE;

//This is stuff for connecting the PS3 to USB.
USB Usb;
USBHub Hub1(&Usb);
BTD Btd(&Usb);
PS3BT PS3(&Btd);

void setup() {
  //Assign motor pin outs
  leftMotor.attach(LEFT_MOTOR,      1000, 2000);
  rightMotor.attach(RIGHT_MOTOR,    1000, 2000);
  servoMotor.attach(SERVO_MOTOR,    1000, 2000);

  stop();

  pinMode(REDLED,   OUTPUT);
  pinMode(BLUELED,  OUTPUT);
  pinMode(GREENLED, OUTPUT);
  pinMode(IRLED,    OUTPUT);

  flashLEDs();

  setServo(SERVO_DN);

  Serial.begin(115200);
  if (Usb.Init() == -1)
  { // this is for an error message with USB connections
    Serial.print(F("\r\nOSC did not start"));
    while (1);
  }
  Serial.print(F("\r\nPS3 Bluetooth Library Started"));
}

void loop() {
  Usb.Task(); //updates buffer of PS3 inputs

  if (PS3.PS3Connected || PS3.PS3NavigationConnected) //This only lets the program run if the PS3 controller is connected.
  {
    if (newconnect == 0)
    {
      newconnect = 1;
      #ifdef DEBUG
      Serial.println("Connection is good!");
      #endif
      PS3.moveSetRumble(64);
      PS3.setRumbleOn(50, 255, 50, 255); //VIBRATE!!!    
      setBowDirection(FORWARD);
      setGreen();
    }
    if (PS3.getButtonClick(PS))
    {
      #ifdef DEBUG
      Serial.println("Disconnect");
      #endif
      kidMode = false;
      PS3.disconnect();
      setBowDirection(FORWARD);
      newconnect = 0;
      setBlue();
      stop();
    }

    if(PS3.getButtonPress(SELECT))
    {
      if(PS3.getButtonClick(START))
      {
        if(!kidMode)
        {
          kidMode = true;
          #ifdef DEBUG
          Serial.print("Entering Kid Mode ");
          #endif
          PS3.setRumbleOff();
          PS3.setLedRaw(LED_STATUS_KIDMODE);
          currentHandicap = KID_HANDICAP;
        }
        else
        {
          kidMode = false;
          #ifdef DEBUG
          Serial.print("Exiting Kid Mode ");
          #endif
          PS3.setRumbleOff();
          PS3.setLedRaw(LED_STATUS_FORWARD);
          currentHandicap = HANDICAP;
        }
      }
    }

    if (PS3.getButtonClick(L1))
    {
      toggleServo();
    }

    if (PS3.getButtonClick(R1))
    {
      toggleBowDirection();
    }
    
    if (PS3.getButtonPress(R2) && !kidMode)
    {
      #ifdef DEBUG
      Serial.print("Turbo! ");
      #endif
      currentHandicap = TURBO;
    }
    else if (!kidMode) currentHandicap = HANDICAP;
    else 
    {
      currentHandicap = KID_HANDICAP;
      #ifdef DEBUG
      Serial.print("Kid Mode! ");
      #endif
    }
  
    int yInput = map(PS3.getAnalogHat(LeftHatY), 0, 255, -90, 90); //Recieves PS3 forward/backward input
    int xInput = map(PS3.getAnalogHat(RightHatX), 0, 255, 90, -90); //Recieves PS3 horizontal input and sets it to an inverted scale of 90 to -90
    setGreen();

    
    if (abs(yInput) < DEADZONE) yInput = 0;
    if (abs(xInput) < DEADZONE) xInput = 0;
    yInput *= TURN_LIMITER;
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

    int ThrottleL =  LEFT_FLIP * (((bowDirection * Drive) + Turn) / currentHandicap); //This is the final variable that decides motor speed.
    int ThrottleR = RIGHT_FLIP * (((bowDirection * Drive) - Turn) / currentHandicap);

    if (ThrottleL > 90) ThrottleL = 90;
    else if(ThrottleL < -90) ThrottleL = -90;
    if (ThrottleR > 90) ThrottleR = 90;
    else if(ThrottleR < -90) ThrottleR = -90;

    #ifdef DEBUG
    Serial.print("ThrottleL: ");
    Serial.print(ThrottleL);
    Serial.print(" ThrottleR: ");
    Serial.print(ThrottleR);
    Serial.print(" motorCorrect: ");
    Serial.print(motorCorrect);
    if(SERVO_DN == servoState) Serial.print(" DN ");
    else if(SERVO_UP == servoState) Serial.print(" UP ");
    Serial.println(" ;");
    #endif
    leftMotor.write((ThrottleL + 90 + motorCorrect)); //Sending values to the speed controllers
    rightMotor.write((ThrottleR + 90 + motorCorrect));
    
  }
      
  else
  {
    Serial.println("Stop");
    stop();
    setBlue();
  }

}

void stop()
{
  leftMotor.writeMicroseconds(1500);
  rightMotor.writeMicroseconds(1500);
  servoMotor.writeMicroseconds(1500);
}

void flashLEDs()
{
  digitalWrite(GREENLED,  LOW);
  digitalWrite(BLUELED,   LOW);
  digitalWrite(REDLED,    HIGH);
  delay(500);
  digitalWrite(REDLED,    LOW);
  digitalWrite(GREENLED,  HIGH);
  delay(500);
  digitalWrite(GREENLED,  LOW);
  digitalWrite(BLUELED,   HIGH);
}

void setGreen()
{
  digitalWrite(REDLED,    LOW);
  digitalWrite(BLUELED,   LOW);
  digitalWrite(GREENLED,  HIGH);
}

void setRed()
{
  digitalWrite(GREENLED,  LOW);
  digitalWrite(BLUELED,   LOW);
  digitalWrite(REDLED,    HIGH);
}

void setBlue()
{
  digitalWrite(GREENLED,  LOW);
  digitalWrite(REDLED,    LOW);
  digitalWrite(BLUELED,   HIGH);
}

void setServo(bool state)
{
  if(state == servoState) return; // requesting servo move to it's existing position
  else if (SERVO_DN == state)
  {
    digitalWrite(IRLED, LOW);
    servoMotor.write(SERVO_DN_POS);
    servoState = SERVO_DN;
  }
  else if (SERVO_UP == state)
  {
    servoMotor.write(SERVO_UP_POS);
    servoState = SERVO_UP;
    digitalWrite(IRLED, HIGH);
  }
}

void toggleServo()
{
  if (SERVO_UP == servoState)
  {
    digitalWrite(IRLED, LOW);
    servoMotor.write(SERVO_DN_POS);
    servoState = SERVO_DN;
  }
  else if (SERVO_DN == servoState)
  {
    servoMotor.write(SERVO_UP_POS);
    servoState = SERVO_UP;
    digitalWrite(IRLED, HIGH);
  }
}

void toggleBowDirection()
{
  if(FORWARD == bowDirection)
  {
    bowDirection = BACKWARD;
    PS3.setRumbleOff();
    PS3.setLedRaw(LED_STATUS_BACKWARD);
    PS3.setRumbleOn(25, 255, 25, 255); //VIBRATE!!!        
  }
  else if(BACKWARD == bowDirection)
  {
    bowDirection = FORWARD;
    PS3.setRumbleOff();
    if(!kidMode) PS3.setLedRaw(LED_STATUS_FORWARD);
    else PS3.setLedRaw(LED_STATUS_KIDMODE);
    PS3.setRumbleOn(25, 255, 25, 255); //VIBRATE!!!      
  }
}

void setBowDirection(int dir)
{
  if(dir = bowDirection) return;
  if(BACKWARD == dir)
  {
    bowDirection = BACKWARD;
    PS3.setRumbleOff();
    PS3.setLedRaw(LED_STATUS_BACKWARD);
    PS3.setRumbleOn(25, 255, 25, 255); //VIBRATE!!!        
  }
  else if(FORWARD == bowDirection)
  {
    bowDirection = FORWARD;
    PS3.setRumbleOff();
    if(!kidMode) PS3.setLedRaw(LED_STATUS_FORWARD);
    else PS3.setLedRaw(LED_STATUS_KIDMODE);
    PS3.setRumbleOn(25, 255, 25, 255); //VIBRATE!!!      
  }
}
