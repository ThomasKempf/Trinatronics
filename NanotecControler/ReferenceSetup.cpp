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


void map(int ModbusAdresse,int Index,int Nanoj)
{
	if(((od_read(ModbusAdresse, 0x00) != Index) && (od_read(ModbusAdresse, 0x00) !=9)) || (od_read(ModbusAdresse, Index) != Nanoj))		//if entry not mapped yet
		{
		
			od_write(ModbusAdresse, 0x00, 0x00);				//deactivate mapping
			yield();
			od_write(ModbusAdresse, Index, Nanoj);			//set mapping entry
			yield();
			od_write(ModbusAdresse, 0x00, Index);				//activate mapping -> 8 mapped TxPDO objects now
			yield();
			
			od_write(0x1010, 0x01, 1702257011);			//save all command
			yield();
			while(od_read(0x1010, 0x01) != 1)			//wait for save done
			{
				yield();
			}
			od_write(0x2800, 0x01, 0x746F6F62);			//reboot
			yield();
		}
}


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


void GoNextPosition(int PositionOrder)
{
	int MultiplicationFactor = 1;
	int NextTargetPosition = 0;
	od_write(0x6083, 0x00, 0XFFFFFFF);//acceleration
	od_write(0x6084, 0x00, 0XFFFFFFF);//deceleration
	ModesOfOperation(1);
	AbsoluteMovement();
	ChangeSetPointImmediately(true);
	Out.ProfileVelocity=4000;
	
	
	NextTargetPosition = ZeroPosition + (PositionOrder * MultiplicationFactor);
	Out.TargetPosition = NextTargetPosition;
	NewSetPoint(true);
	yield();
	NewSetPoint(false);
}


void StopMotor()
{
	Quickstop();
	od_write(0x2300, 0x00, 0x0);
	Shutdown();
}


void user()
{
	map(0x3602,0x07,0x25000120); // controler status    adresse 5008
	map(0x3502,0x08,0x24000120); // Order				adresse 6010
	map(0x3502,0x09,0x24000220); // Setpoint			adresse 6012
	
	
	od_write(0x60A8, 0x00, 0xFE410000); //control faktor of position FF 10^-1 FA 10^-6
	
	ReferenceDrive();	
	
	while (true)
	{
		int order = od_read(0x2400,0x01);
		int NextPose = od_read(0x2400,0x02);
		
		if (order == 3)
		{
			GoNextPosition(NextPose);
		}
		else if (order == 1)
		{
			StopMotor();
		}
		else
		{
			yield();
		}
	}
}