#include "Parameter.h"
#include <Arduino.h>
#include <ModbusMaster.h>


ModbusMaster node[] = {ModbusMaster(),ModbusMaster(),ModbusMaster(),ModbusMaster()};// slave instance x_moteur, y_moteur, rh, rv
uint16_t IndexSetPoint = 6012;
uint16_t IndexOrder = 6010;
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
  // Serial.print("Slave:" );
  // Serial.print(SlaveID);
  // Serial.print(", Index:" );
  // Serial.print(Index);
  // Serial.print(", Function:");
  // Serial.print(Function);
  if (Function == 6)
  {
    // Serial.print(", WriteValue :");
    // Serial.print(WriteValue);
  }
  if (Succes)
  {
    // Serial.print(", IndexValue :");
    // Serial.println(ReadValue);
  }
  else
  {
    // Serial.println(", Error");
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
    // Serial.print("Read value: ");
    // Serial.println(ReadValue);
    PrintModbusMessage(SlaveID,Index,Function,true);
    ControlerReadStatus[SlaveID-1] = ReadValue;
    // Serial.print("controler status in read ");
    // Serial.print( ControlerReadStatus[SlaveID-1]);
    // Serial.print(" / slave ID: ");
    // Serial.println(SlaveID);
  } 
  else // error
  { 
    PrintModbusMessage(SlaveID,Index,Function,false);
  }
 delay(5); // delay to become the slave message
}


int DefineSetPoint(char SlaveID)
{
    int SetPointValue[] = { x_moteur, y_moteur, rh, rv };
    SetPoint = SetPointValue[SlaveID-1];
    return SetPoint;
}


void WriteAllControler()
{
  for (int SlaveID = 1; SlaveID <= NumberOfControler; SlaveID++) //slave ID 1 -> 4
  {
    // Serial.print("write controler: ");
    // Serial.println(SlaveID - 1);
    SetPoint = DefineSetPoint(SlaveID);
    ReadorWrite(&node[SlaveID-1],SlaveID,IndexSetPoint); // write_new setpoint
    ReadorWrite(&node[SlaveID-1],SlaveID,IndexOrder);
    // Serial.println("write finish");
    delay(15); // delay between two slaver
  }
}


// Read the status from the controller and compare it with the order
void ReadAllControler()
{
  char NbrOfCorrectStatus = 0;
  for (int SlaveID = 1; SlaveID <= NumberOfControler; SlaveID++) //slave ID 1 -> 4
  {
    // Serial.print("read controler: ");
    // Serial.println(SlaveID - 1);
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
    delay(15); // delay between two slaver
    // Serial.println("finish read");
  }
  if (NbrOfCorrectStatus == NumberOfControler)
  {
    ControlerStatus = Order; // all the controler have the good State
  }
}


void referenceProtocol()
{
  uint8_t runingOrder[] = { 3, 4, 1, 2 }; // rh rv x_moteur y_moteur
  int refSetPoint [] = {0, 18000, 10, 200};
  delay(100);
  for (int controler = 1; controler <= NumberOfControler; controler++)
  {
    uint8_t SlaveID = runingOrder[controler-1];
    // Serial.print("make reference controler: ");
    // Serial.println(SlaveID);
    Order = 5; // Reference
    ReadorWrite(&node[SlaveID-1],SlaveID,IndexOrder);
    delay(100);
    ReadorWrite(&node[SlaveID-1],SlaveID,IndexStatus);
    if (ControlerReadStatus[SlaveID-1] == 2)
    {
      Order = 3; // AutoMode
      SetPoint = refSetPoint[controler-1]; // go to refSetPoint
      ReadorWrite(&node[SlaveID-1],SlaveID,IndexSetPoint);
      ReadorWrite(&node[SlaveID-1],SlaveID,IndexOrder);
      do 
      {
        delay(100); // wait end of the Reference
        ReadorWrite(&node[SlaveID-1],SlaveID,IndexStatus);
        // Serial.print("controler status: ");
        // Serial.print(ControlerReadStatus[SlaveID-1]);
        // Serial.print(" / slave ID: ");
      // Serial.println(SlaveID);
      } while (ControlerReadStatus[SlaveID-1] != 3);//control that ref is end
    }
    // Serial.println("finish reference");
    delay(100); // ensure that are finish
  }
  RequereReference = false;
  Order = 3; // AutoMode
  SetPoint = 450;
  ReadorWrite(&node[1-1],1,IndexSetPoint);
  ReadorWrite(&node[1-1],1,IndexOrder);
  delay(200); // ensure that are finish
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
  else if (AutoMode == true and ManualMove == false and RequereReference == true)
  {
    Order = 5; // Reference
    RequereReference = false;
  }
}


// Update controller data: write values, read responses, and update controller status
void UpdateControlerStatus() 
{
  // Serial.println("update Controler Order");
  UpdateOrder();
  // Serial.print("order: ");
  // Serial.println(Order);
  WriteAllControler();
  do {
    ReadAllControler();
  } while (ControlerStatus != Order and ControlerStatus != 2 and  ControlerStatus != 0);
  // Serial.print("ControlerStatus:");
  // Serial.println(ControlerStatus);
}
