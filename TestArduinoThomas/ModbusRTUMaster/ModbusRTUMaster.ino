
#include <ModbusMaster.h>
#include <SoftwareSerial.h>
//Rx/Tx is hooked up to pins 5, 3
SoftwareSerial mySerial(5, 3); // RX, TX
ModbusMaster node;
void setup() {
  // Modbus communication runs at 9600 baudrate
  Serial.begin(9600);
  mySerial.begin(9600);
  // Modbus slave ID 1
  node.begin(5, mySerial);
}
void loop()
{
uint8_t result;
uint16_t statusword = 0;

// Lire le registre 0x6041 (Statusword)
result = node.readHoldingRegisters(0x6041, &statusword);
if (result == node.ku8MBSuccess)
{
  Serial.print("ReadHoldingRegisters: ");
  Serial.println(statusword, HEX);
  delay(1000);
}
else
{
  Serial.print("lecture echec\n");
}
delay(1000);

}

