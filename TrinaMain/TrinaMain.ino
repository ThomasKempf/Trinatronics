#include "Parameter.h"
#include "MotorControler.h"



void setup() 
{
  // put your setup code here, to run once:
  Serial.begin(9600);//USB
  setupModbus();
}

// example ho to use UpdateControlerStatus
void loop() 
{
  Serial.begin(9600);
  void referenceProtocol();
  // define the parameter
  x = 3;
  y = 4;
  rh = 5;
  rv = 6;
  AutoMode = false;
  ManualMove = false;

  UpdateControlerStatus();
  delay(2000);
}
