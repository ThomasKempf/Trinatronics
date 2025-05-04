#include "Parameter.h"

void setup() {
    Serial.begin(9600);
    x = 42;
    y = 4;
    Serial.println("Setup terminé");
}

void loop() {
    Serial.println(x);
    Serial.println(y);
    Serial.println(ManualMove);
    delay(1000);
}