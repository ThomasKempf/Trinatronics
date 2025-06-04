#include "Parameter.h"
#include "MotorControler.h"

unsigned long previousMillis = 0;
int state = 0;

void setup() 
{
  Serial.begin(115200);
  setupModbus();
  referenceProtocol();

  Serial.println("Demarrage du systeme de test des moteurs...");
}

void loop() 
{
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= 10000 || state == 0) {
    previousMillis = currentMillis;
    
    if (state == 0) {
      // Première commande
      x = 100; // mm
      y = 100;
      rh = 0; // °*10^-2
      rv = 9000;
      AutoMode = true; // Quand t'es en mode auto il se déplace
      ManualMove = false; // Quand il y a un déplacement en cours (c'est quand c'est appuyé sur les boutons)
      Serial.println("==> Envoi de la commande 1");
    }
    else if (state == 1) {
      // Deuxième commande
      x = 700; // mm
      y = 200;
      rh = 9000; // °*10^-2
      rv = 27000;
      AutoMode = true; // Quand t'es en mode auto il se déplace
      ManualMove = false; // Quand il y a un déplacement en cours (c'est quand c'est appuyé sur les boutons)
      UpdateControlerStatus();
      Serial.println("==> Envoi de la commande 2");
    }
    else {
      Serial.println("==> Fin du test. Boucle figée.");
      while (true); // Arrêt du programme
    }

    AutoMode = true;
    ManualMove = false;

    // Mise à jour du contrôle
    UpdateControlerStatus();

    // Affichage des paramètres actuels
    Serial.print("x = "); Serial.println(x);
    Serial.print("y = "); Serial.println(y);
    Serial.print("rh = "); Serial.println(rh);
    Serial.print("rv = "); Serial.println(rv);
    Serial.print("AutoMode = "); Serial.println(AutoMode);
    Serial.print("ManualMove = "); Serial.println(ManualMove);
    Serial.print("ControlerStatus = "); Serial.println(ControlerStatus);
    Serial.println("-----------------------------");

    state++;
  }
}
