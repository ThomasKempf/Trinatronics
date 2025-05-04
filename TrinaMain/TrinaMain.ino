#include "Parameter.h"
#include "MotorControler.h"

void setup() 
{
  // put your setup code here, to run once:

}

void loop() 
{
  Serial.begin(9600);
  x = 3;
  y = 4;
  rh = 5;
  rv = 6;
  UpdateControlerStatus();
  delay(2000);
}
