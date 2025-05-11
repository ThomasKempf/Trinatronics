#include "Parameter.h"
#include <Arduino.h>

ModbusMaster node1;// slave x
ModbusMaster node2;// slave y
ModbusMaster node3;// slave rv
ModbusMaster node4;// slave rh


int SetPoint = 0; // new SetPoint of the controler
char Order = 4; // 1 ManualMove, 3 AutoMode , 4 ManualMode
char ControlerReadStatus[] = { 0, 0, 0, 0 };
int SimulationStatus = 4; // simulation state of all the controler


void setupModbus()
{
  Serial1.begin(19200, SERIAL_8E1); //TX18 RX19 8 bits de données, parité paire, 1 bit d'arrêt
  node1.begin(1, Serial1);
  node2.begin(2, Serial1);
  node3.begin(3, Serial1);
  node4.begin(4, Serial1);
}


void printModbusMessage(int8_t SlaveID,int16_t Index,int8_t Function,bool Succes)
{
  Serial.print("Slave:" );
  Serial.print(SlaveID);
  Serial.print(", Index:" );
  Serial.print(Index);
  Serial.print(", Function:");
  Serial.print(Function);
  if (Function == 6)
  {
    Serial.print(", WriteValue :");
    Serial.print(WriteValue);
  }
  if (Succes)
  {
    Serial.print(", IndexValue :");
    Serial.println(ReadValue);
  }
  else
  {
    Serial.println(", Error");
  }
}


uint16_t readorWrite(ModbusMaster* node,uint8_t SlaveID,uint8_t Function,uint16_t Index)
{
  uint16_t Result;
  uint8_t Error;

  if (Function == 3)
  {
    Result = node->readHoldingRegisters(Index, 2);
  }
  else if (Function == 6)
  {
    Result = node->writeSingleRegister(Index, WriteValue);
  }

  if (Result == node->ku8MBSuccess) // succes
  { 
    ReadValue = node->getResponseBuffer(0);
    printModbusMessage(SlaveID,Index,Function,true);
    return 0;
  } 
  else // error
  { 
    printModbusMessage(SlaveID,Index,Function,false);
    return Result;
  }
}


int DefineSetPoint(char SlaveID)
{
    int SetPointValue[] = { x, y, rh, rv };
    SetPoint = SetPointValue[SlaveID-1];
    Serial.print(SetPoint);
    return SetPoint;
}


void WriteAllControler()
{
  Serial.print("Setpoint: ");
  for (int SlaveID = 1; SlaveID <= 4; SlaveID++) //slave ID 1 -> 4
  {
    SetPoint = DefineSetPoint(SlaveID);
    error = readorWrite(&node5,5,3,5000);
    delay(1); // simul write into slave SetPoint and Order
  }
  Serial.println(" ");
}


void ReadControler(char SlaveID)
{
  delay(1); // simul read eadControlerStatus
  ControlerReadStatus[SlaveID-1] = SimulationStatus;
}


// Read the status from the controller and compare it with the order
void ReadAllControler()
{
  char NbrOfCorrectStatus = 0;
  for (int SlaveID = 1; SlaveID <= 4; SlaveID++) //slave ID 1 -> 4
  {
    ReadControler(SlaveID);
    if (ControlerReadStatus[SlaveID-1] == 0 or ControlerReadStatus[SlaveID-1] == 2) // if conection are fail or reference are active, break the function
    {
      ControlerStatus = ControlerReadStatus[SlaveID-1];
      return;
    }
    else if (ControlerReadStatus[SlaveID-1] == Order) // the controler have the same State with the Order
    {
      NbrOfCorrectStatus = NbrOfCorrectStatus + 1;
    }
  }
  if (NbrOfCorrectStatus == 4)
  {
    ControlerStatus = Order; // all the controler have the good State
  }
}


// Update the order given to the controller
void UpdateOrder()
{
  if (AutoMode == true and ManualMove == false)
  {
    Order = 3; // AutoMode
  }
  else if (AutoMode == false and ManualMove == false)
  {
    Order = 4; // manualMode
  }
  else if (ManualMove == true)
  {
    Order = 1; // ManualMove
  }
}


// Update controller data: write values, read responses, and update controller status
void UpdateControlerStatus() 
{
  UpdateOrder();
  Serial.print("order: ");
  Serial.println(Order, DEC);
  WriteAllControler();
  do {
    ReadAllControler();
  } while (ControlerStatus != Order and ControlerStatus != 2 and ControlerStatus != 0);
  Serial.print("ControlerStatus:");
  Serial.println(ControlerStatus, DEC);
}
