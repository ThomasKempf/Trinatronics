map U16 Controlword as inout 0x6040:00
map U16 Statusword as input 0x6041:00
map U32 Inputs as input 0x60FD:00
map U32 Outputs as inout 0x60FE:01
map S08 ModesOfOperation as output 0x6060:00
map S08 ModesOfOperationDisplay as input 0x6061:00
map S16 AnalogInput as input 0x3220:01
map S32 TargetPosition as output 0x607A:00
map U32	ProfileVelocity as output 0x6081:00

#include "wrapper.h"
#include "nanotec.h"

int ZeroPosition = 0;

void ReferenceDrive()
{
	int Velocity = -10;

	ModesOfOperation(3);
	od_write(0x60FF, 0x00, Velocity);
	EnableOperation();
	while (DigitalInput(1) != true)
	{
		yield();
	}
	od_write(0x60FF, 0x00, 0);
	ZeroPosition = od_read(0x6064, 0x00);
}

void user()
{
	int Velocity = -10;
	int Amplitude = 18000; // amplitude in degre
	int StartPosition = 18000;
	float MultiplicationFactor = 1;
	
	int NextTargetPosition = 0;
	od_write(0x60A8, 0x00, 0xFE410000); //control faktor of position FF 10^-1 FA 10^-6
	
	ReferenceDrive();	
	ModesOfOperation(1);
	AbsoluteMovement();
	ChangeSetPointImmediately(true);
	Out.ProfileVelocity=600;
	NextTargetPosition = ZeroPosition + (StartPosition * MultiplicationFactor);
	Out.TargetPosition = NextTargetPosition;
	NewSetPoint(true);
	sleep(10);
	NewSetPoint(false);
	while (TargetReached() != true)
	{
		yield();
	}
	
	Quickstop();
	od_write(0x2300, 0x00, 0x0);
}