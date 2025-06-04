// Déclaration des broches
const int pinOnOff = 25;            // Interrupteur ON/OFF physique
const int pinButtonMode = 27;       // Bouton poussoir de changement de mode
const int pinLedButton = 26;        // LED intégrée au bouton (ou témoin de mode)
const int pinRelay = 24;            // Sortie relais (commande lampe)

bool lastButtonState = HIGH;        // État précédent du bouton mode pour détection de front
int mode = 0;                       // 0 = Auto (LED allumée), 1 = Manuel (LED éteinte)
unsigned long lastDebounceTime = 0; // Pour anti-rebond
const unsigned long debounceDelay = 50; // Délai de stabilisation des appuis (ms)

void setup() {
  pinMode(pinOnOff, INPUT_PULLUP);      // Interrupteur câblé avec résistance pullup interne
  pinMode(pinButtonMode, INPUT_PULLUP); // Bouton poussoir aussi en pullup
  pinMode(pinLedButton, OUTPUT);        // Sortie LED témoin
  pinMode(pinRelay, OUTPUT);            // Sortie relais

  Serial.begin(115200);                 // Initialisation du port série
  Serial.println("--- Démarrage du système ---");
}

void loop() {
  // Lecture de l'état de l'interrupteur ON/OFF (LOW si activé)
  bool systemOn = (digitalRead(pinOnOff) == HIGH);

  // Si le système est ON, activation du relais
  digitalWrite(pinRelay, systemOn ? LOW : HIGH);

  if (systemOn) {
    // Lecture de l'état actuel du bouton mode
    bool currentButtonState = digitalRead(pinButtonMode);

    // Si état changé (front) + après délai de stabilisation
    if (currentButtonState != lastButtonState && millis() - lastDebounceTime > debounceDelay) {
      if (currentButtonState == LOW) { // Appui détecté
        mode = 1 - mode;  // Inversion du mode : 0 <-> 1
        Serial.print("Mode changé : ");
        Serial.println(mode == 0 ? "Automatique" : "Manuel");
      }
      lastDebounceTime = millis(); // Mémorisation temps de changement
    }
    lastButtonState = currentButtonState; // Mise à jour état précédent

    // Activation de la LED si mode = 0 (Auto)
    digitalWrite(pinLedButton, (mode == 0) ? HIGH : LOW);
  } else {
    // Système OFF → LED éteinte
    digitalWrite(pinLedButton, LOW);
  }

  // Affichage des paramètres
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 200) {
    Serial.print("État système: ");
    Serial.print(systemOn ? "ON" : "OFF");
    Serial.print(" | Mode: ");
    Serial.print(mode == 0 ? "Auto" : "Manuel");
    Serial.print(" | Lampe: ");
    Serial.println(systemOn ? "ON" : "OFF");
    lastPrint = millis();
  }
}