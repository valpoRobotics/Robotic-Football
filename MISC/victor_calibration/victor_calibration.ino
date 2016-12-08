/*
   This program is designed to calibrate victors.
   Alternatively, it can also be used to run a single motor for a specified period of time at a chosen speed.

   Setup:
   1. Wire the motor and 12V battery to the victor. DO THIS BEFORE CONNECTING A PWM CABLE TO THE VICTOR!

    Proper wiring between the arduino and the victor to be tested:
    2. Connect the black wire from the PWM cable to one of the GND pins on the arduino.
    3. Connect the white wire from the PWM cable to PIN 9 on the arduino.

   4. Upload this program to the arduino and remove power from the arduino.
   5. Wire motor to victor.
   6. Connect a PWM cable to the victor, with the white wire facing outwards.

   Program written by Matt Bull Oct 2015.
*/

#include <Servo.h>
#include <avr/sleep.h>
#define VICTOR_PIN 9 // the pin that the victor's signal line is attached to
#define ledPin 13

Servo victor;
int i;
int maxForward = 180; // Choose a value from 1501 to 2000. Larger number is faster. Set to 0 to skip running in forward.
int maxReverse = 0; // Choose a value from 1499 to 1000. Smaller number is faster. Set to 0 to skip running in reverse.
int increment = 1;
int atSpeed = 1500; //Time motor will stay at max speed. (in milliseconds)
int pause = 500; //Time between motor stopping and reversing. (in milliseconds)
int continuous = 0; //set equal to 1 in order to create an endless loop
int ledState = HIGH;

void setup()
{
  victor.attach(VICTOR_PIN, 1000, 2000);
  Serial.begin(115200); //This is necessary to be able to view the print statements in the serial monitor
  // initialize digital pin 13 as an output. Pin 13 controls the LED on the arduino itself.
  pinMode(ledPin, OUTPUT);
}

void loop()
{
  victor.write(90); // drive STOP
  digitalWrite(ledPin, LOW);
  delay(5000); // ensure there is 3 seconds before drive starts
  digitalWrite(ledPin, HIGH);

  // Calls the forward function if maxForward is within limits.
  if (maxForward > 90 && maxForward < 181)
  {
    forward();
  }
  else {
    Serial.print("\r\n Forward skipped: Outside of acceptable forward values.");
  }

  // Calls the reverse function if maxReverse is within limits.
  if (maxReverse < 90 && maxReverse >= 0)
  {
    Serial.print("\r\n Begin Reverse.");
    reverse();
  }
  else {
    Serial.print("\r\n Reverse skipped: Outside of acceptable reverse values.");
  }

  // Hold motor in stopped state.
  victor.write(90); // drive STOP
  Serial.print("\r\n Stop");
  digitalWrite(ledPin, LOW);
  if (continuous != 1) {};
  //  if (continuous == 0) { //this bit of code puts the arduino to sleep. Since there is nothing to wake it up it is a more power friendly while(true){} loop
  //    cli(); //disables all interrupts
  //    sleep_enable(); //basically says to allow sleep mode
  //    sleep_cpu(); //actually starts sleep mode
  //  }
}

void forward(void)
{
  for ( i = 90; i < maxForward; i += increment) //Slow ramp to set max forward speed.
  {
    victor.write(i);
    delay(15);
  }
  Serial.print("\r\n Forward");
  delay(atSpeed); // max forward for set time.

  for ( i = maxForward; i > 90; i -= increment) //slow to stop
  {
    victor.write(i);
    delay(15);
  }
  delay(pause); //allow motor to coast to a stop (If jumper on the victor is set to the left position the motor will stop immedietly)
}

void reverse(void)
{
  for ( i = 90; i > maxReverse; i -= increment) // Slow ramp up to set reverse speed
  {
    victor.write(i);
    delay(15);
  }
  Serial.print("\r\n Reverse");
  delay(atSpeed); // Run for set time at set max reverse.

  for ( i = maxReverse; i < 90; i += increment) // Slow ramp down to full stop.
  {
    victor.write(i);
    delay(15);
  }
  delay(pause); //Allow motor to coast to stop
}
