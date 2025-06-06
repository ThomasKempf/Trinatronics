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

void map(int ModbusAdresse,int Index,int Nanoj)
{
	if( (od_read(ModbusAdresse, 0x00) != Index) || (od_read(ModbusAdresse, Index) != Nanoj) )		//if entry not mapped yet
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


void user()
{
	map(0x3502,0x08,0x24000120);
	map(0x3602,0x07,0x25000120);
	int Velocity = -100;
	while(true)
	{
		if (od_read(0x2400,0x01) == 6)
		{
			ModesOfOperation(3);
			od_write(0x60FF, 0x00, Velocity);
			EnableOperation();
			sleep(2000);
			od_write(0x60FF, 0x00, 0);
			sleep(2000);
			od_write(0x2500, 0x01, 10);
			sleep(500);
			od_write(0x2500, 0x01, 0);	
		}
		yield();
	}
}