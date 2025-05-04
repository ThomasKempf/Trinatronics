#include "Parameter.h"
#include <Arduino.h>


int SetPoint = 0;
char Order = 4; // 1 ManuelMove, 3 AutoMode , 4 ManualMode
char ControlerReadStatus[] = { 0, 0, 0, 0 };
int SimulationStatus = 4; // simulation state of all the controler


void DefineSetPoint(char SlaveID)
{
    int SetPointValue[] = { x, y, rh, rv };
    SetPoint = SetPointValue[SlaveID-1];
    Serial.print(SetPoint);
}


void WriteAllControler()
{
  Serial.print("Setpoint: ");
  for (int SlaveID = 1; SlaveID <= 4; SlaveID++) //slave ID 1 -> 4
  {
    DefineSetPoint(SlaveID);
    delay(1); // simul write into slave SetPoint and Order
  }
  Serial.println(" ");
}


void ReadControler(char SlaveID)
{
  delay(1); // simul read eadControlerStatus
  ControlerReadStatus[SlaveID-1] = SimulationStatus;
}


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
    else if (ControlerReadStatus[SlaveID-1] == Order)
    {
      NbrOfCorrectStatus = NbrOfCorrectStatus + 1;
    }
    else if (ControlerReadStatus[SlaveID-1] == 1 or ControlerReadStatus[SlaveID-1] == 3 or ControlerReadStatus[SlaveID-1] == 4)// if we have an other state, change global state do not have a 0
    {
      ControlerStatus = 10;// Different State in the four controler
    }
  }
  if (NbrOfCorrectStatus == 4)
  {
    ControlerStatus = Order;
  }
}


void UpdateOrder()
{
  if (AutoMode == true and ManualMove == false)
  {
    Order = 3;
  }
  else if (AutoMode == false and ManualMove == false)
  {
    Order = 4;
  }
  else if (ManualMove == true)
  {
    Order = 1;
  }
}


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
