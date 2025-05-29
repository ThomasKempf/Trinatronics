#ifndef MOTORCONTROLER_H
#define MOTORCONTROLER_H
#include <ModbusMaster.h>

void setupModbus();
void PrintModbusMessage(int8_t SlaveID,int16_t Index,int8_t Function,bool Succes);
void ReadorWrite(ModbusMaster* node,uint8_t SlaveID,uint16_t Index);
void ReadAllControler();
void UpdateOrder();
int DefineSetPoint(char SlaveID);
void WriteAllControler();
void UpdateControlerStatus();

#endif