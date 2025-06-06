#include <ModbusMaster.h>

ModbusMaster node[] = {ModbusMaster(),ModbusMaster()};

uint16_t ReadValue;
uint16_t WriteValue;


void setupModbus()
{
  Serial1.begin(19200, SERIAL_8E1); //TX18 RX19 8 bits de données, parité paire, 1 bit d'arrêt
  for (uint8_t SlaveID = 1; SlaveID <= 2; SlaveID++)
  {
    node[SlaveID-1].begin(3, Serial1);
  }
  int8_t error = 0;
  WriteValue = 3;
  error = readorWrite(&node[0],4,6,6010);
  delay(200); // Attendre avant la prochaine lecture
  WriteValue = 9000;
  error = readorWrite(&node[0],4,6,6012);
  delay(200); // Attendre avant la prochaine lecture
  error = readorWrite(&node[0],4,3,5008);
}


void setup() 
{
  Serial.begin(9600);//USB
  setupModbus();
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


void loop() 
{
  delay(200); // Attendre avant la prochaine lecture

}
