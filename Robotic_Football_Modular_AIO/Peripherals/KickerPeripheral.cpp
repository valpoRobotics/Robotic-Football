#include <Servo.h>
#include "src/USB_Host_Shield_Library_20/PS3BT.h"
#include "src/USB_Host_Shield_Library_20/usbhub.h"

#define KICKER_MOTOR          5     	// Kicker motor is wired to pin 5
										//these are the speeds for kicking and reload the kicker foot
#define KICKER_POWER          2000   
#define KICKER_RELOAD         1000
Servo kicker;                       	// Define motor object for the kicker motor

void peripheralSetup(){
	kicker.attach(KICKER_MOTOR);
	kicker.writeMicroseconds(1500);
}
  
void peripheral(PS3BT PS3){
	if (PS3.getButtonPress(CROSS)){
		kicker.writeMicroseconds(KICKER_POWER);
	}else if (PS3.getButtonPress(TRIANGLE)){
		kicker.writeMicroseconds(KICKER_RELOAD);
	}else{
		kicker.writeMicroseconds(1500);
	}
}