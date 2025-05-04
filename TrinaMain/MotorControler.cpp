#include "Parameter.h"
#include <Arduino.h>


int SetPoint = 0; // new SetPoint of the controler
char Order = 4; // 1 ManualMove, 3 AutoMode , 4 ManualMode
char ControlerReadStatus[] = { 0, 0, 0, 0 };
int SimulationStatus = 4; // simulation state of all the controler


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
