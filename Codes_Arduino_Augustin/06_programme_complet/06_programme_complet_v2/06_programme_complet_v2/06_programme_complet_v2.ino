#include <Pixy2I2C.h>
#include <Wire.h>
#include "Parameter.h"
#include "MotorControler.h"

// extern uint8_t ControlerStatus; // 0 erro / 1 Motor Off / 2 Reference / 3 Enable / 4 Manuel Mode
// extern int x; // lateral translation, 5 à 900 mm, vers la droite
// extern int y; // depth translation, 5 à 380 mm, vers l'opérateur
// extern int rh; // horizontal rotation , -395 à 9100 °*10^-2, 0 étant vers le bas, 90 vers l'avant.
// extern int rv; // vertical translation, 1055 à 34050 °*10^-2, 1055 étant du centre passant au dessus de l'épaule droite, 34050 passant au dessus de l'épaule gauche

// STRUCTURES DE BASE
struct Vect2D {float x, y;};
struct Vect3D {float x, y, z;};
struct Matrice {float m[3][2];};
Pixy2I2C pixy1;
Pixy2I2C pixy2;

// DÉCLARATION DES BROCHES
const int pinOnOff = 25;            // SWITCH : Interrupteur ON/OFF physique
const int pinButtonMode = 27;       // BP : Bouton poussoir de changement de mode
const int pinLedButton = 26;        // BP : LED intégrée au bouton
const int pinRelay = 24;            // LED : Sortie relais (commande lampe)

bool lastButtonState = HIGH;        // BP : État précédent du bouton mode pour détection de front
int mode = 0;                       // BP : 0 = Auto (LED allumée), 1 = Manuel (LED éteinte)
unsigned long lastDebounceTime = 0; // BP : Pour anti-rebond
const unsigned long debounceDelay = 50; // BP : Délai de stabilisation des appuis (ms)

unsigned long previousMillis = 0; // Pour le test des moteurs
bool lastStateOn = false; // Pour relancer le test moteur après switch sur on

// DONNÉES DE CALIBRATION
// Ces données sont à copier-coller dans le .py, en modifiant les valeurs de position des points en fonction de la calibration.
// Le .py donnera alors les coefficients d'homographie. 
// CTRL + : pour enlever et remettre les //

// pixy = "_1"
// # Points dans l'image (pixels)
// pts_image = np.array([
//     [297, 176],
//     [161, 13],
//     [135, 160],
//     [80, 38]
// ], dtype=np.float32)
// # Points réels sur le plan de travail (en mm)
// pts_reels = np.array([
//     [45, 470],
//     [1095, 520],
//     [335, 45],
//     [1095, 45]
// ], dtype=np.float32)

// Coefficients d'homographie pour la Pixy1
float h11_1 = -3.2572961359;
float h12_1 = -1.4564009859;
float h13_1 = 940.2694375514;
float h21_1 = 1.1898411359;
float h22_1 = 0.5256002376;
float h23_1 = -79.6320609977;
float h31_1 = -0.0039950470;
float h32_1 = 0.0024970885;
float h33_1 = 1.0000000000;

// pixy = "_2"
// # Points dans l'image (pixels)
// pts_image = np.array([
//     [116, 195],
//     [134, 12],
//     [241, 198],
//     [222, 14]
// ], dtype=np.float32)
// # Points réels sur le plan de travail (en mm)
// pts_reels = np.array([
//     [1095, 570],
//     [45, 570],
//     [1095, 45],
//     [265, 45]
// ], dtype=np.float32)

// Coefficients d'homographie pour la Pixy2
float h11_2 = -374.5941610032;
float h12_2 = -686.5371505189;
float h13_2 = 57550.5113986114;
float h21_2 = 325.6971439239;
float h22_2 = 6.6487038854;
float h23_2 = -90335.9323810888;
float h31_2 = -0.4605637470;
float h32_2 = -0.3071092850;
float h33_2 = 1.0000000000;

// POSITIONS FIXES DES CAMÉRAS
const Vect3D C1 = {0, 0, 60};  // Caméra gauche - Pixy1
const Vect3D C2 = {60, 0, 60};  // Caméra droite - Pixy2

// DÉFINITION DES FONCTIONS
void transformer(float x, float y,
                 float h11, float h12, float h13,
                 float h21, float h22, float h23,
                 float h31, float h32,
                 float &X, float &Y) {
  float denom = h31 * x + h32 * y + 1.0;
  if (denom != 0) {
    X = (h11 * x + h12 * y + h13) / denom;
    Y = (h21 * x + h22 * y + h23) / denom;
  } else {
    X = Y = -1.0; // erreur
  }
}

// DROITES VECTORIELLES : CALCUL DE LA POSITION X Y Z
Vect3D vecteur_directeur(const Vect3D& origine, float X, float Y) {
  return {X - origine.x, Y - origine.y, -origine.z}; // direction vers le plan Z = 0
}
Vect3D vectoriel(const Vect3D& D1, const Vect3D& D2) {
  Vect3D v = {D1.y * D2.z - D1.z * D2.y, D1.z * D2.x - D1.x * D2.z, D1.x * D2.y - D1.y * D2.x};
  return {v};
}
float scalaire(const Vect3D& D1, const Vect3D& D2) {
  return {D1.x * D2.x + D1.y * D2.y + D1.z * D2.z};
}
Vect3D point_plus_proche(const Vect3D& C1, const Vect3D& D1, const Vect3D& C2, const Vect3D& D2) {

  Vect3D r = {C2.x - C1.x, C2.y - C1.y, C2.z - C1.z}; // Vecteur entre les 2 caméras (C1 vers C2)

  Vect3D n = vectoriel(D1,D2); // Produit vectoriel des 2 vecteurs directeurs D1 et D2
  Vect3D n1 = vectoriel(n,D2);
  Vect3D n2 = vectoriel (n,D1);

  float lambda = scalaire(r,n1)/scalaire(D1,n1);
  float mu = scalaire(r,n2)/scalaire(D2,n2);

  Vect3D P1 = {C1.x + lambda*D1.x, C1.y + lambda*D1.y, C1.z + lambda*D1.z};
  Vect3D P2 = {C2.x + mu*D2.x, C2.y + mu*D2.y, C2.z + mu*D2.z};

  return {(P1.x + P2.x)/2, (P1.y + P2.y)/2, (P1.z + P2.z)/2};
}

// ----- SETUP -----

void setup() {
  // Wire.begin();
  pinMode(pinOnOff, INPUT_PULLUP);      // SWITCH défini en entrée
  pinMode(pinButtonMode, INPUT_PULLUP); // BP défini en entrée
  pinMode(pinLedButton, OUTPUT);        // BP : LED définie en sortie
  pinMode(pinRelay, OUTPUT);            // LED : Relais défini en sortie

  Serial.begin(115200);                 // Initialisation du port série
  setupModbus();
  referenceProtocol();
  Serial.println("--- Démarrage du système ---");

  pixy1.init(0x53);
  pixy2.init(0x54);
}

void loop() {

  bool systemOn = (digitalRead(pinOnOff) == HIGH); // SWITCH : Lecture de l'état de l'interrupteur ON/OFF
  digitalWrite(pinRelay, systemOn ? LOW : HIGH); // LED : Si le système est ON, activation du relais | Attention ! Le relais est inversé, la lampe est allumée lorsque LOW est envoyé.
  int pixy_detection = 0; // Nombre de détection par les Pixy (0, 1, 2 ou 3 si les 2 pixy détectent l'objet)
  float X1 = 0.0, Y1 = 0.0, X2 = 0.0, Y2 = 0.0; // Poistions réelles sur la surface détectées par les Pixy 1 et 2
  Vect3D P={0.0,0.0,0.0}; // Position réelle (x,y,z) calculée si détection par les 2 Pixy
  unsigned long currentMillis = millis();
  Serial.println("depart boucle");
  if (systemOn) {
    if (!lastStateOn){
      Serial.println("!lastStateOn");
      lastStateOn = true;
      previousMillis = currentMillis;
    }
    Serial.println("read pinButtonMode");
    bool currentButtonState = digitalRead(pinButtonMode); //BP : Lecture
    if (currentButtonState != lastButtonState && millis() - lastDebounceTime > debounceDelay) { 
      if (currentButtonState == LOW) {
        mode = 1 - mode;  // Si état changé (front) + après délai de stabilisation, inversion du mode (manuel/auto)
      }
      lastDebounceTime = millis(); // Mémorisation temps de changement
    }
    lastButtonState = currentButtonState; // Mise à jour état précédent

    digitalWrite(pinLedButton, (mode == 0) ? HIGH : LOW); // BP : Activation de la LED témoin si mode = 0 (Auto)

    Serial.println("pixi 1");
    pixy1.ccc.getBlocks();
    if (pixy1.ccc.numBlocks) {
      pixy_detection = 1;
      int x1 = pixy1.ccc.blocks[0].m_x;
      int y1 = pixy1.ccc.blocks[0].m_y;
      transformer(x1, y1, h11_1, h12_1, h13_1, h21_1, h22_1, h23_1, h31_1, h32_1, X1, Y1);
      // X1 et Y1 sont les positions réelles sur la surface de la table détectées par la Pixy1
    }
    Serial.println("pixi 2");
    pixy2.ccc.getBlocks();
    if (pixy2.ccc.numBlocks) {
      if (pixy_detection == 1){
        pixy_detection = 3;
      }
      else if (pixy_detection == 0)
      {
        pixy_detection = 2;
      }
      int x2 = pixy2.ccc.blocks[0].m_x;
      int y2 = pixy2.ccc.blocks[0].m_y;
      transformer(x2, y2, h11_2, h12_2, h13_2, h21_2, h22_2, h23_2, h31_2, h32_2, X2, Y2);
      // X2 et Y2 sont les positions réelles sur la surface de la table détectées par la Pixy2
    }

    if (pixy_detection == 3){
      Serial.println("pixi 2 1");
      // Serial.print(" | Écart (x,y) = ("); Serial.print(X2 - X1); Serial.print(","); Serial.print(Y2 - Y1); Serial.print(")");
      // Serial.print(" | Moyenne (x,y) = ("); Serial.print(min(X2,X1)+abs(X2 - X1)/2); Serial.print(","); Serial.print(min(Y2,Y1)+abs(Y2 - Y1)/2); Serial.print(")");
      Vect3D D1 = vecteur_directeur(C1, X1, Y1);
      Vect3D D2 = vecteur_directeur(C2, X2, Y2);
      P = point_plus_proche(C1, D1, C2, D2);
      // Serial.print(" | Intersection approx_moteur= (");
      // Serial.print(P.x); Serial.print(", "); Serial.print(P.y); Serial.print(", "); Serial.print(P.z); Serial.print(")");
    }

    if ((currentMillis - previousMillis)%10000 <= 5000) { 
      Serial.println("(currentMillis - previousMillis)%10000 <= 5000");
      x_moteur = 200;
      y_moteur = 250;
      rh = 3000;
      rv = 10000;
      AutoMode = true; // Mode automatique : il se déplace
      ManualMove = false; // Lorsqu'il y a un mouvement manuel en cours (les 2 boutons sont enclenchés)
    }
    else {
      Serial.println("else (currentMillis - previousMillis)%10000 <= 5000");
      x_moteur= 700;
      y_moteur = 200;
      rh = 1500;
      rv = 27000;
      AutoMode = true; // Mode automatique : il se déplace
      ManualMove = false; // Lorsqu'il y a un mouvement manuel en cours (les 2 boutons sont enclenchés)
      UpdateControlerStatus();
      }

      AutoMode = true;
      ManualMove = false;

      // Mise à jour du contrôle
      Serial.println("go to update controler");
      UpdateControlerStatus();

      // Affichage des paramètres actuels
      Serial.print("x_moteur = "); Serial.print(x_moteur);
      Serial.print(" | y_moteur = "); Serial.print(y_moteur);
      Serial.print(" | rh = "); Serial.print(rh);
      Serial.print(" | rv = "); Serial.print(rv);
      Serial.print(" | AutoMode = "); Serial.print(AutoMode);
      Serial.print(" | ManualMove = "); Serial.print(ManualMove);
      Serial.print(" | ControlerStatus = "); Serial.print(ControlerStatus);
      Serial.println("");
  }
  else {
    digitalWrite(pinLedButton, LOW);
    lastStateOn = false;
  }
  Serial.println("affiche paramètre");
  // AFFICHAGE DES PARAMÈTRES
  static unsigned long lastPrint = 0; // La boucle est gérée pour tourner "au plus vite", donc on ajoute une régulation temporelle pour l'affichage
  if (millis() - lastPrint > 400) {
    Serial.print("État système: ");
    Serial.print(systemOn ? "ON" : "OFF");
    Serial.print(" | ");
    Serial.print(mode == 0 ? "Automatique" : "Manuel");
    if (pixy_detection%2 == 1){
      Serial.print(" | Pixy 1 - Surface ("); Serial.print(X1); Serial.print(","); Serial.print(Y1); Serial.print(")");
    }
    if (pixy_detection > 1){
      Serial.print(" | Pixy 2 - Surface ("); Serial.print(X2); Serial.print(","); Serial.print(Y2); Serial.print(")");
    }
    if (pixy_detection == 3){
      Serial.print(" | Position absolue estimée = (");
      Serial.print(P.x); Serial.print(", "); Serial.print(P.y); Serial.print(", "); Serial.print(P.z); Serial.print(")");
    }
    if (systemOn){
      Serial.print(" | Temps depuis switch sur ON (ms) = ");
      Serial.print(currentMillis - previousMillis);
    }

    Serial.println("");
    lastPrint = millis();
    Serial.println("boucle fini");
  }
}