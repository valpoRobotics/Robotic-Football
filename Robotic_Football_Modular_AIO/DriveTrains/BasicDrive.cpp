#include <Servo.h>

Servo leftMotor, rightMotor;    // Define motor objects
static int drive = 0;   // Initial speed before turning calculations
static int turn = 0;    // Turn is adjustment to drive for each motor separately to create turns
static int motorDirection = 1;
static int xInput, yInput, throttleL, throttleR;
static int motorCorrection = 0;
#define LEFT_MOTOR 9    // left motor is wired to pin 9
#define RIGHT_MOTOR 10    // right motor is wired to pin 10
#define MAX_DRIVE 90    // limited because of issues with calibrating victors to full 0-180 range

//#define SHOW_DEBUG_INFO   //Uncomment to print debug info over serial port

/**
 * Sets up the motors by attaching them to the correct pins
 * @param motorType Defines the direction of the motors
 */
void driveSetup(int motorType) {
  /* These lines are attaching the motor objects to their output pins on the arduino
    1000, 2000 refers to the minimum and maximum pulse widths to send to the motors (AKA full forward/reverse)
    1500 represents stop
  */
  leftMotor.attach(LEFT_MOTOR, 1000, 2000);
  rightMotor.attach(RIGHT_MOTOR, 1000, 2000);
  leftMotor.writeMicroseconds(1500);
  rightMotor.writeMicroseconds(1500);
  motorDirection = motorType;
}

/**
 * Drives the robot according to input from 2 joysticks
 * @param handicap Amount of handicap to put on the motors.
 * @param leftX X input from the left joystick.
 * @param leftY Y input from the left joystick.
 * @param rightX X input from the right joystick.
 * @param rightY y input from the right joystick.
 */
void driveCtrl(int handicap, int leftX, int leftY, int rightX, int rightY)
{
  if ((rightY == 0) && (leftX == 0))
  { /* If no input this should ensure that the motors actually stop 
      and skip the rest of the drive function 
      */
    leftMotor.writeMicroseconds(1500);
    rightMotor.writeMicroseconds(1500);
  }

  if (drive < leftY) drive++;    // Accelerates
  else if (drive > leftY) drive--;   // Decelerates

  if (turn < rightX) turn++;    //Turns left?
  else if (turn > rightX) turn--;   //Turns right?
  
  // These are the final variables that decide motor speed
  throttleL = motorDirection * ((drive - turn) / handicap);
  throttleR = -1 * motorDirection * ((drive + turn) / handicap) + motorCorrection;

  // Limiting the max value of the throttle
  if (throttleL > MAX_DRIVE) throttleL = MAX_DRIVE;
  else if (throttleL < -MAX_DRIVE) throttleL = -MAX_DRIVE;
  if (throttleR > MAX_DRIVE) throttleR = MAX_DRIVE;
  else if (throttleR < -MAX_DRIVE) throttleR = -MAX_DRIVE;
  
  // Sending values to the speed controllers
  leftMotor.write(throttleL + 90);   
  rightMotor.write(throttleR + 90);

  #ifdef SHOW_DEBUG_INFO
    Serial.print("Throttle Left: ");
    Serial.print(throttleL);
    Serial.print("Throttle Right: ");
    Serial.print(throttleR);
    Serial.print("\t");
  #endif
}

/**
 * Corrects the motors if the robot does not drive straight
 * @param correction +1 if correction to the left, -1 if correction to the right
 */
void correctMotor(int correction){
  if (motorCorrection > -90 && motorCorrection < 90){
    motorCorrection += correction;
  }
  #ifdef SHOW_DEBUG_INFO
    Serial.print("Motor correction: ");
    Serial.print(motorCorrection);
    Serial.print("\t");
  #endif
}

/**
 * Stops the motors
 */
void driveStop()
{
  leftMotor.writeMicroseconds(1500);
  rightMotor.writeMicroseconds(1500);
}
