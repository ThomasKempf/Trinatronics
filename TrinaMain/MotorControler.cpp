#include "Parameter.h"
#include <Arduino.h>


int SetPoint = 0;


void DefineSetPoint(char SlaveID)
{
  int SetPointValue[] = { x, y, rh, rv };
  SetPoint = SetPointValue[SlaveID-1];
}


void WriteControler(char SlaveID)
{
  delay(1); // simul write into slave
}


void UpdateControlerStatus() 
{
  for (int SlaveID = 1; SlaveID <= 4; SlaveID++) //slave ID 1 -> 4
  {
    DefineSetPoint(SlaveID);
    Serial.println(SetPoint);

    WriteControler(SlaveID);

  }
}
