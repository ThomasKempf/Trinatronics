#include "Parameter.h"
#include <Arduino.h>
#include <ModbusMaster.h>


ModbusMaster node[] = {ModbusMaster(),ModbusMaster(),ModbusMaster(),ModbusMaster()};// slave instance x, y, rh, rv
uint16_t IndexSetPoint = 6010;
uint16_t IndexOrder = 6012;
uint16_t IndexStatus = 5008;
uint16_t WriteValue = 0;
uint8_t ReadValue = 0;

int SetPoint = 0; // new SetPoint of the controlers
uint8_t Order = 4; // 1 ManualMove, 3 AutoMode , 4 ManualMode, 5 Reference
uint8_t ControlerReadStatus[] = { 0, 0, 0, 0 };// 0 erro , 1 Motor Off , 2 Reference , 3 Enable , 4 Manuel Mode
uint8_t NumberOfControler = 4;
bool RequereReference = false;

void setupModbus()
{
  Serial1.begin(19200, SERIAL_8E1); //TX18 RX19 8 bits de données, parité paire, 1 bit d'arrêt
  for (uint8_t SlaveID = 1; SlaveID <= NumberOfControler; SlaveID++)
  {
    node[SlaveID-1].begin(SlaveID, Serial1);
  }
}


void PrintModbusMessage(int8_t SlaveID,int16_t Index,int8_t Function,bool Succes)
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


void ReadorWrite(ModbusMaster* node,uint8_t SlaveID,uint16_t Index)
{
  uint16_t Result;
  uint8_t Function; // Modbus Function

  if(Index == IndexSetPoint)
  {
    WriteValue = SetPoint;
    Function = 6;
    Result = node->writeSingleRegister(Index, WriteValue);
  }
  else if (Index == IndexOrder)
  {
    WriteValue = Order;
    Function = 6;
    Result = node->writeSingleRegister(Index, WriteValue);
  }
  else if (Index == IndexStatus)
  {
    Function = 3;
    Result = node->readHoldingRegisters(Index, 2);
  }

  if (Result == node->ku8MBSuccess) // succes
  { 
    ReadValue = node->getResponseBuffer(0);
    PrintModbusMessage(SlaveID,Index,Function,true);
    if (Index == IndexStatus)
    {
      ControlerReadStatus[SlaveID-1] = ReadValue;
    }
  } 
  else // error
  { 
    PrintModbusMessage(SlaveID,Index,Function,false);
  }
  delay(100); // delay to become the slave message
}


int DefineSetPoint(char SlaveID)
{
    int SetPointValue[] = { x, y, rh, rv };
    SetPoint = SetPointValue[SlaveID-1];
    return SetPoint;
}


void WriteAllControler()
{
  for (int SlaveID = 1; SlaveID <= NumberOfControler; SlaveID++) //slave ID 1 -> 4
  {
    SetPoint = DefineSetPoint(SlaveID);
    ReadorWrite(&node[SlaveID-1],SlaveID,IndexSetPoint); // write_new setpoint
    ReadorWrite(&node[SlaveID-1],SlaveID,IndexOrder);
    delay(200); // delay between two slaver
  }
}


// Read the status from the controller and compare it with the order
void ReadAllControler()
{
  char NbrOfCorrectStatus = 0;
  for (int SlaveID = 1; SlaveID <= NumberOfControler; SlaveID++) //slave ID 1 -> 4
  {
    ReadorWrite(&node[SlaveID-1],SlaveID,IndexStatus); // read_new ControlerStatus
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
  if (NbrOfCorrectStatus == NumberOfControler)
  {
    ControlerStatus = Order; // all the controler have the good State
  }
}


void referenceProtocol()
{
  uint8_t runingOrder[] = { 3, 4, 1, 2 }; // rh rv x y
  int refSetPoint [] = { 0, 18000, 450, 200};
  for (int SlaveID = 1; SlaveID <= NumberOfControler; SlaveID++)
  {
    Order = 5; // Reference
    ReadorWrite(&node[runingOrder[SlaveID-1]-1],runingOrder[SlaveID-1],IndexOrder);
    Order = 3; // AutoMode
    SetPoint = refSetPoint[SlaveID-1]; // go to refSetPoint
    ReadorWrite(&node[runingOrder[SlaveID-1]-1],runingOrder[SlaveID-1],IndexSetPoint);
    ReadorWrite(&node[runingOrder[SlaveID-1]-1],runingOrder[SlaveID-1],IndexOrder);
    do 
    {
      delay(200); // wait end of the Reference
      ReadorWrite(&node[runingOrder[SlaveID-1]-1],runingOrder[SlaveID-1],IndexStatus);
    } while (ControlerReadStatus[runingOrder[SlaveID-1]-1] != 3 && ControlerReadStatus[runingOrder[SlaveID-1]-1] != 0);//control that ref is end
  }
  RequereReference = false;
}


// Update the order given to the controller
void UpdateOrder()
{
  if (AutoMode == true and ManualMove == false and RequereReference == false)
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
    RequereReference = true;
  }
  else if (AutoMode == true and ManualMove == false and RequereReference == false)
  {
    Order = 5; // Reference
    RequereReference = false;
  }
}


// Update controller data: write values, read responses, and update controller status
void UpdateControlerStatus() 
{
  UpdateOrder();
  Serial.print("order: ");
  Serial.println(Order);
  WriteAllControler();
  do {
    ReadAllControler();
  } while (ControlerStatus != Order and ControlerStatus != 2);
  Serial.print("ControlerStatus:");
  Serial.println(ControlerStatus);
}
