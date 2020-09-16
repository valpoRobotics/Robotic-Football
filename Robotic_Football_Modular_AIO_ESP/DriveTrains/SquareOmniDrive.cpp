#include <Servo.h>
Servo motor1, motor2, motor3, motor4;         // Define motor objects
static int vertical = 0, horizontal = 0, turn = 0;
static int motorDirection = 1;
static int throttleM1, throttleM2, throttleM3, throttleM4;
#define MOTOR1 7    // motor1 is wired to pin 9
#define MOTOR2 8    // motor2 is wired to pin 10
#define MOTOR3 9    // motor3 is wired to pin 11
#define MOTOR4 10   // motor4 is wired to pin 12

void driveSetup(int motorType) {
  /* These lines are attaching the motor objects to their output pins on the arduino
    1000, 2000 refers to the minimum and maximum pulse widths to send to the motors (AKA full forward/reverse)
    1500 represents stop
  */
  motor1.attach(MOTOR1, 1000, 2000);
  motor2.attach(MOTOR2, 1000, 2000);
  motor3.attach(MOTOR3, 1000, 2000);
  motor4.attach(MOTOR4, 1000, 2000);
  motor1.writeMicroseconds(1500);
  motor2.writeMicroseconds(1500);
  motor3.writeMicroseconds(1500);
  motor4.writeMicroseconds(1500);
  motorDirection = motorType;

}

void driveCtrl(int handicap, int leftX, int leftY, int rightX, int rightY)
{
  if ((leftY == 0) && (rightX == 0)){ // if no input this should ensure that
    // the motors actually stop, and skip the rest
    // of the drive function
    motor1.writeMicroseconds(1500);
	motor3.writeMicroseconds(1500);
  }
if((leftX == 0) && (rightX == 0)){
	motor2.writeMicroseconds(1500);
	motor4.writeMicroseconds(1500);
}
  
  if (vertical < leftY) vertical++;         // Accelerates
  else if (vertical > leftY) vertical--;    // Decelerates
  
  if (horizontal < leftX) horizontal++;      // Accelerates
  else if (horizontal > leftX) horizontal--; // Decelerates
  
  if (turn < rightX) turn++;                 // Accelerates
  else if (turn> rightX) turn--;             // Decelerates  
  
  throttleM1 = ((-1*horizontal - turn)  / handicap) + 90;
  throttleM2 = ((vertical - turn)  		/ handicap) + 90;
  throttleM3 = ((horizontal - turn)		/ handicap) + 90;
  throttleM4 = ((-1*vertical - turn)    / handicap) + 90;
  
  if (throttleM1 < 5)throttleM1 = 5;
  else if (throttleM1 > 175)throttleM1 = 175;
  if (throttleM2 < 5)throttleM2 = 5;
  else if (throttleM2 > 175)throttleM2 = 175;
  if (throttleM3 < 5)throttleM3 = 5;
  else if (throttleM3 > 175)throttleM3 = 175;
  if (throttleM4 < 5)throttleM4 = 5;
  else if (throttleM4 > 175)throttleM4 = 175;
  
  motor1.write(throttleM1);
  motor2.write(throttleM2);
  motor3.write(throttleM3);
  motor4.write(throttleM4);
}

void driveStop()
{
  motor1.writeMicroseconds(1500);
  motor2.writeMicroseconds(1500);
  motor3.writeMicroseconds(1500);
  motor4.writeMicroseconds(1500);
  
}
