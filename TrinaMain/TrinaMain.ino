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
  x = 3; // mm
  y = 4;
  rh = 5; // °*10^-2
  rv = 6;
  AutoMode = false; // Quand t'es en mode auto il se déplace
  ManualMove = false; // Quand il y a un déplacement en cours (c'est quand c'est appuyé sur les boutons)

  UpdateControlerStatus();
  delay(2000);
}
