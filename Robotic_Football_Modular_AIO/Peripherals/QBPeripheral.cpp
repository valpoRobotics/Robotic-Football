#include <Servo.h>
#include <PS3BT.h>
#include <usbhub.h>

#define QB_THROWER            5
#define TRIANGLE_THROW        175
#define CIRCLE_THROW          125
#define CROSS_THROW           108
#define SQUARE_THROW          102
#define RELOAD_THROW          88
int throwOffset = 0;                //used to adjust strength of cross and circle throws
Servo qbThrower;


void peripheralSetup(){
	qbThrower.attach(QB_THROWER, 1000, 2000);
	qbThrower.writeMicroseconds(1500);
}
  
void peripheral(PS3BT PS3){
	if (PS3.getButtonPress(TRIANGLE))    qbThrower.write(TRIANGLE_THROW);
	else if (PS3.getButtonPress(CIRCLE)) qbThrower.write(CIRCLE_THROW + throwOffset);
	else if (PS3.getButtonPress(CROSS))  qbThrower.write(CROSS_THROW + throwOffset);
	else if (PS3.getButtonPress(SQUARE)) qbThrower.write(SQUARE_THROW + throwOffset) ;
	else if (PS3.getButtonPress(R1))
	{
		qbThrower.write(RELOAD_THROW);
		throwOffset = 0;
	}
	else qbThrower.writeMicroseconds(1500);
}