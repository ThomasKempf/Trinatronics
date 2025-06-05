#include "Parameter.h"
#include "MotorControler.h"

// DÉCLARATION DES BROCHES
const int pinOnOff = 25;            // SWITCH : Interrupteur ON/OFF physique
const int pinButtonMode = 27;       // BP : Bouton poussoir de changement de mode
const int pinLedButton = 26;        // BP : LED intégrée au bouton
const int pinRelay = 24;            // LED : Sortie relais (commande lampe)
const int pinManualMove = 29;       // BP : Bouton poussoir de mouvement manuel

bool lastButtonState = HIGH;        // BP : État précédent du bouton mode pour détection de front
int mode = 0;                       // BP : 0 = Auto (LED allumée), 1 = Manuel (LED éteinte)
unsigned long lastDebounceTime = 0; // BP : Pour anti-rebond
const unsigned long debounceDelay = 50; // BP : Délai de stabilisation des appuis (ms)

unsigned long previousMillis = 0; // Pour le test des moteurs
bool lastStateOn = false; // Pour relancer le test moteur après switch sur on

// ----- SETUP -----

void setup() {
  pinMode(pinOnOff, INPUT_PULLUP);      // SWITCH défini en entrée
  pinMode(pinButtonMode, INPUT_PULLUP); // BP défini en entrée
  pinMode(pinLedButton, OUTPUT);        // BP : LED définie en sortie
  pinMode(pinRelay, OUTPUT);            // LED : Relais défini en sortie
  pinMode(pinManualMove, INPUT_PULLUP); // BP : Mode manuel


  Serial.begin(115200);                 // Initialisation du port série
  Serial.println("--- Démarrage du système | Test boutons ---");

}

void loop() {

  bool systemOn = (digitalRead(pinOnOff) == HIGH); // SWITCH : Lecture de l'état de l'interrupteur ON/OFF
  digitalWrite(pinRelay, systemOn ? LOW : HIGH); // LED : Si le système est ON, activation du relais | Attention ! Le relais est inversé, la lampe est allumée lorsque LOW est envoyé.
  unsigned long currentMillis = millis();

  if (systemOn) {
    if (!lastStateOn){
      lastStateOn = true;
      previousMillis = currentMillis;
    }
    bool currentButtonState = digitalRead(pinButtonMode); //BP : Lecture
    if (currentButtonState != lastButtonState && millis() - lastDebounceTime > debounceDelay) { 
      if (currentButtonState == LOW) {
        mode = 1 - mode;  // Si état changé (front) + après délai de stabilisation, inversion du mode (manuel/auto)
      }
      lastDebounceTime = millis(); // Mémorisation temps de changement
    }
    lastButtonState = currentButtonState; // Mise à jour état précédent
    
    bool manualMoveButtonPressed = (digitalRead(pinManualMove) == LOW);
    if (manualMoveButtonPressed) {
      ManualMove = true;
      AutoMode = false;
      mode = 1; // Passage en mode manuel
    } else {
      if (mode == 0) {
        ManualMove = false; // On autorise l'arrêt du mouvement manuel que si on est en mode auto
      }
    }

    if (mode==0){
      AutoMode = true;
    }
    else{
      AutoMode = false;
    }

    digitalWrite(pinLedButton, (mode == 0) ? HIGH : LOW); // BP : Activation de la LED témoin si mode = 0 (Auto)
  }
  else {
    digitalWrite(pinLedButton, LOW);
    lastStateOn = false;
    mode=0;
  }
  // AFFICHAGE DES PARAMÈTRES
    Serial.print("État système : ");
    Serial.print(systemOn ? "ON" : "OFF");
    Serial.print(mode == 0 ? " | Automatique" : " | Manuel");
    Serial.print(" Pin manualMove : ");
    Serial.print(digitalRead(pinManualMove));
    if (systemOn){
      Serial.print(" | Temps depuis switch sur ON (ms) = ");
      Serial.print(currentMillis - previousMillis);
    }
    Serial.println("");
}





