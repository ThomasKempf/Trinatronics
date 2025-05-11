#include <ModbusMaster.h>

ModbusMaster node[] = {ModbusMaster(),ModbusMaster()};

uint16_t ReadValue;
uint16_t WriteValue;

//void setupModbusConnection()

void setupModbus()
{
  Serial1.begin(19200, SERIAL_8E1); //TX18 RX19 8 bits de données, parité paire, 1 bit d'arrêt
  for (uint8_t SlaveID = 1; SlaveID <= 2; SlaveID++)
  {
    node[SlaveID-1].begin(4+SlaveID, Serial1);
  }
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
  int8_t error = 0;
  WriteValue = 6;
  error = readorWrite(&node[0],5,3,5000);
  delay(2000); // Attendre avant la prochaine lecture
  error = readorWrite(&node[0],5,6,6008);
  delay(2000); // Attendre avant la prochaine lecture
  error = readorWrite(&node[1],3,3,5000);
  delay(2000); // Attendre avant la prochaine lecture
  error = readorWrite(&node[1],3,6,6008);
  delay(2000); // Attendre avant la prochaine lecture
}
