// Premier programme : Retours des valeurs de position détectées par les caméras.
// Objectif : test et prise des valeurs pour la calibration

#include <Pixy2I2C.h>

Pixy2I2C pixy1;
Pixy2I2C pixy2;

#include <Wire.h>

void setup() {
  Wire.begin();
  Serial.begin(115200);
  Serial.println(">>> DÉMARRAGE <<<");
  pixy1.init(0x53);

  pixy2.init(0x54);

  pinMode(13, OUTPUT);
}

void loop() {
  int r1 = pixy1.ccc.getBlocks();
  int r2 = pixy2.ccc.getBlocks();

  if (r1 > 0 && pixy1.ccc.numBlocks > 0 && r2 > 0 && pixy2.ccc.numBlocks > 0) {
    digitalWrite(13, HIGH);

    Serial.print("Cam1 (x,y): ");
    Serial.print(pixy1.ccc.blocks[0].m_x);
    Serial.print(", ");
    Serial.print(pixy1.ccc.blocks[0].m_y);

    Serial.print("| Cam2 (x,y): ");
    Serial.print(pixy2.ccc.blocks[0].m_x);
    Serial.print(", ");
    Serial.println(pixy2.ccc.blocks[0].m_y);

  } else {
    digitalWrite(13, LOW);
    Serial.println("Aucune détection.");
  }

  delay(200);
}
