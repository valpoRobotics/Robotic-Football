#include <Servo.h>
Servo leftMotor, rightMotor;        // Define motor objects
static int drive = 0;               // Initial speed before turning calculations
static int turn = 0;                // Turn is adjustment to drive for each motor separately to create turns
static int motorDirection = 1;
static int xInput, yInput, throttleL, throttleR;
static float rightAdjust = 1.0;
#define LEFT_MOTOR            9     // left motor is wired to pin 9
#define RIGHT_MOTOR           10    // right motor is wired to pin 10

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
#define MAX_DRIVE             90    // limited because of issues with calibrating victors to full 0-180 range
}

void driveCtrl(int handicap, int leftX, int leftY, int rightX, int rightY)
{
  if ((rightY == 0) && (leftX == 0))
  { // if no input this should ensure that
    // the motors actually stop, and skip the rest
    // of the drive function
    leftMotor.writeMicroseconds(1500);
    rightMotor.writeMicroseconds(1500);
  }

  if (drive < leftY)drive++;                     // Accelerates
  else if (drive > leftY) drive--;               // Decelerates

  if (turn < rightX) turn++;
  else if (turn > rightX) turn--;

  throttleL = motorDirection * ((drive - turn) / handicap);
  // This is the final variable that
  // decides motor speed.
  throttleR = -1 * motorDirection * ((drive + turn) / handicap) * rightAdjust;

  if (throttleL > MAX_DRIVE) throttleL = MAX_DRIVE;
  else if (throttleL < -MAX_DRIVE)throttleL = -MAX_DRIVE;
  if (throttleR > MAX_DRIVE) throttleR = MAX_DRIVE;
  else if (throttleR < -MAX_DRIVE)throttleR = -MAX_DRIVE;

  leftMotor.write(throttleL + 90);   // Sending values to the speed controllers
  rightMotor.write(throttleR + 90);
}

void motorAdjust(float adjust){
  if ((adjust + rightAdjust) > 0.5 && (adjust + rightAdjust) < 1.5){
    rightAdjust = rightAdjust + adjust;
  }
  Serial.print(rightAdjust);

}

void driveStop()
{
  leftMotor.writeMicroseconds(1500);
  rightMotor.writeMicroseconds(1500);
}
