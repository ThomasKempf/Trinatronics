#include "Parameter.h"
#include "MotorControler.h"

void setup() 
{
  // put your setup code here, to run once:

}

void loop() 
{
  Serial.begin(9600);
  UpdateControlerStatus();
  delay(2000);
}
