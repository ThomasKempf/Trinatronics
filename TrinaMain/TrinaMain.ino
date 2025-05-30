// #include "Parameter.h"
// #include "MotorControler.h"



// void setup() 
// {
//   // put your setup code here, to run once:
//   Serial.begin(9600);//USB
//   setupModbus();
// }

// // example ho to use UpdateControlerStatus
// void loop() 
// {
//   Serial.begin(9600);
//   referenceProtocol();
//   // define the parameter
//   x = 3; // mm
//   y = 4;
//   rh = 5; // °*10^-2
//   rv = 6;
//   AutoMode = false; // Quand t'es en mode auto il se déplace
//   ManualMove = false; // Quand il y a un déplacement en cours (c'est quand c'est appuyé sur les boutons)

//   UpdateControlerStatus();
//   delay(2000);
// }

#include "Parameter.h"
#include "MotorControler.h"

unsigned long previousMillis = 0;
int state = 0;

void setup() 
{
  Serial.begin(9600);
  setupModbus();
  referenceProtocol();
  // Initialisation à une valeur de départ
  x = 100;
  y = 200;
  rh = 1000;
  rv = 5000;
  UpdateControlerStatus();
  AutoMode = true;
  ManualMove = false;

  Serial.println("Demarrage du systeme de test des moteurs...");
  delay(500);
}

void loop() 
{
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= 30000 || state == 0) {
    previousMillis = currentMillis;
    
    if (state == 0) {
      // Première commande
      x = 300;
      y = 150;
      rh = 4500;   // 45°
      rv = 2000;   // 20°
      Serial.println("==> Envoi de la commande 1");
      UpdateControlerStatus();
    }
    else if (state == 1) {
      // Deuxième commande
      x = 800;
      y = 400;
      rh = 9000;   // 90°
      rv = 5000;   // 50°
      Serial.println("==> Envoi de la commande 2");
      UpdateControlerStatus();
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
