#include <Servo.h>
#include <PS3BT.h>
#include <usbhub.h>
#define CENTER_RELEASE        5     // the ball release servo is wired to pin 5
#define CENTER_RELEASE_DOWN   120   // these are the angles between 0 and 180 to set servo for releasing and holding the ball
#define CENTER_RELEASE_UP     70    
Servo centerRelease;                // define servo object for ball release 
  
void peripheralSetup(){
	centerRelease.attach(CENTER_RELEASE);   // attach ball release servo to its pin
	centerRelease.write(CENTER_RELEASE_UP);
}
  
void peripheral(PS3BT PS3){
	if (PS3.getButtonClick(TRIANGLE)) centerRelease.write(CENTER_RELEASE_UP);
	else if (PS3.getButtonClick(CROSS)) centerRelease.write(CENTER_RELEASE_DOWN);
}