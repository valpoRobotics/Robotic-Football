#include <Servo.h>
#include <PS3BT.h>
#include <usbhub.h>

#define KICKER_MOTOR          5     	// Kicker motor is wired to pin 5
										//these are the speeds for kicking and reload the kicker foot
#define KICKER_POWER          175   
#define KICKER_RELOAD         85
Servo kicker;                       	// Define motor object for the kicker motor

void peripheralSetup(){
	kicker.attach(KICKER_MOTOR);
	kicker.writeMicroseconds(1500);
}
  
void peripheral(PS3BT PS3){
	if (PS3.getButtonPress(CROSS)) kicker.write(KICKER_POWER);
	else if (PS3.getButtonPress(TRIANGLE)) kicker.write(KICKER_RELOAD);
	else kicker.writeMicroseconds(1500);
}