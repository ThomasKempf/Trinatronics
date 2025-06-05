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
const int pinManualMove = 29;       // BP : Bouton poussoir de mouvement manuel

bool lastButtonState = HIGH;        // BP : État précédent du bouton mode pour détection de front
int mode = 0;                       // BP : 0 = Auto (LED allumée), 1 = Manuel (LED éteinte)
unsigned long lastDebounceTime = 0; // BP : Pour anti-rebond
const unsigned long debounceDelay = 50; // BP : Délai de stabilisation des appuis (ms)

unsigned long previousMillis = 0; // Pour le test des moteurs
bool lastStateOn = false; // Pour relancer le test moteur après switch sur on
int compteur_ManualMove = 0;

const int decalage_x = 200; // Décalage entre x réel et x moteur
const int decalage_y = 100; // Décalage entre y réel et y moteur

const int x_moteur_min = 6;
const int x_moteur_max = 899;
const int y_moteur_min = 100;
const int y_moteur_max = 379;

const int rh_min = -394;
const int rh_max = 2000;
const int rv_min = 1056;
const int rv_max = 34049;

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
const Vect3D C1 = {0, 0, 700};  // Caméra gauche - Pixy1
const Vect3D C2 = {1000, 0, 700};  // Caméra droite - Pixy2

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

void updateMotorPositionFromDetection(float X1, float Y1, float X2, float Y2, int pixy_detection, Vect3D P) {
  float x_target = 0.0;
  float y_target = 0.0;

  if (pixy_detection == 1) {
    x_target = X1;
    y_target = Y1;
  } else if (pixy_detection == 2) {
    x_target = X2;
    y_target = Y2;
  } else if (pixy_detection == 3) {
    x_target = P.x;
    y_target = P.y;
    // x_target = (X1 + X2) / 2.0;
    // y_target = (Y1 + Y2) / 2.0;
  } else {
    return; // Aucune détection
  }

  // Position moteur = position détectée - décalage
  float x_moteur_calc = x_target - decalage_x;
  float y_moteur_calc = y_target - decalage_y;

  // Distance entre position actuelle (en repère réel) et cible
  float dx = x_target - (x_moteur + decalage_x);
  float dy = y_target - (y_moteur + decalage_y);
  float distance = sqrt(dx * dx + dy * dy);

  Serial.print("dx = "); Serial.print(dx);
  Serial.print(" | dy = "); Serial.print(dy);
  Serial.print(" | distance = "); Serial.println(distance);

  if (distance > 200) {
    x_moteur = constrain((int)x_moteur_calc, x_moteur_min, x_moteur_max);
    Serial.print("x_moteur : "); Serial.println(x_moteur);
    y_moteur = constrain((int)y_moteur_calc, y_moteur_min, y_moteur_max);
    Serial.print("y_moteur : "); Serial.println(y_moteur);
  }

  // === CALCUL DES ANGLES DE ROTATION ===
  float angle_v_rad = atan2(dy, dx);
  float angle_v_deg = (angle_v_rad * 180.0 / PI -90);
  int angle_v_deg100 = ((int)(angle_v_deg * 100))%36000;

  Serial.print("angle_v_rad = "); Serial.print(angle_v_rad);
  Serial.print(" rad | angle_v_deg = "); Serial.print(angle_v_deg);
  Serial.print("° | angle_v_deg100 = "); Serial.println(angle_v_deg100);

  float horizontal_distance = sqrt(dx * dx + dy * dy);
  float height = 1000.0; // hauteur fixe de la lampe
  float angle_h_rad = atan2(horizontal_distance, height);
  float angle_h_deg = angle_h_rad * 180.0 / PI;
  int angle_h_deg100 = (int)(angle_h_deg * 100);

  Serial.print("horizontal_distance = "); Serial.print(horizontal_distance);
  Serial.print(" | height = "); Serial.print(height);
  Serial.print(" | angle_v_rad = "); Serial.print(angle_v_rad);
  Serial.print(" rad | angle_v_deg = "); Serial.print(angle_v_deg);
  Serial.print("° | angle_v_deg100 = "); Serial.println(angle_v_deg100);

  // Application des contraintes
  rh = constrain(angle_h_deg100, rh_min, rh_max);
  rv = constrain(angle_v_deg100, rv_min, rv_max);

  Serial.print("rh final : "); Serial.println(rh);
  Serial.print("rv final : "); Serial.println(rv);
  Serial.print("UpdateControler : ("); Serial.print(x_moteur); Serial.print(","); Serial.print(y_moteur); Serial.print(","); Serial.print(rh); Serial.print(","); Serial.print(rv); Serial.println(")");
}

// ----- SETUP -----

void setup() {
  // Wire.begin();
  pinMode(pinOnOff, INPUT_PULLUP);      // SWITCH défini en entrée
  pinMode(pinButtonMode, INPUT_PULLUP); // BP défini en entrée
  pinMode(pinLedButton, OUTPUT);        // BP : LED définie en sortie
  pinMode(pinRelay, OUTPUT);            // LED : Relais défini en sortie
  pinMode(pinManualMove, INPUT_PULLUP); // BP : Mode manuel


  Serial.begin(115200);                 // Initialisation du port série
  setupModbus();
  referenceProtocol();
  Serial.println("--- Démarrage du système | Programme général ---");

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

  Serial.println("Départ boucle");

  if (systemOn) {
    if (!lastStateOn){
      Serial.println("Système allumé");
      lastStateOn = true;
      previousMillis = currentMillis;
    }
    Serial.println("Read pinButtonMode");
    bool currentButtonState = digitalRead(pinButtonMode); //BP : Lecture
    if (currentButtonState != lastButtonState && millis() - lastDebounceTime > debounceDelay) { 
      if (currentButtonState == LOW) {
        mode = 1 - mode;  // Si état changé (front) + après délai de stabilisation, inversion du mode (manuel/auto)
      }
      lastDebounceTime = millis(); // Mémorisation temps de changement
    }
    lastButtonState = currentButtonState; // Mise à jour état précédent
    
    Serial.println("Read pinManualMove");
    // GESTION DU BOUTON MANUEL (pinManualMove)
    int currentManualMoveButtonState = digitalRead(pinManualMove);

    // Détection de front descendant (début d'appui)
    if (currentManualMoveButtonState == 0) {
      compteur_ManualMove= compteur_ManualMove+1;
    }
    else{
      compteur_ManualMove=0;
    }

    // Détection de front montant (relâchement)
    if (compteur_ManualMove>15) {
      ManualMove = true;
      mode = 1;
      Serial.println("ManualMove ACTIVÉ (>300ms)");
    } else {
      ManualMove = false;
    }
    if (mode==0){
      AutoMode = true;
    }
    else{
      AutoMode = false;
    }

    digitalWrite(pinLedButton, (mode == 0) ? HIGH : LOW); // BP : Activation de la LED témoin si mode = 0 (Auto)

    Serial.println("Lecture Pixy 1");
    pixy1.ccc.getBlocks();
    Serial.println("Lecture Pixy 1 terminée");
    if (pixy1.ccc.numBlocks) {
      pixy_detection = 1;
      int x1 = pixy1.ccc.blocks[0].m_x;
      int y1 = pixy1.ccc.blocks[0].m_y;
      transformer(x1, y1, h11_1, h12_1, h13_1, h21_1, h22_1, h23_1, h31_1, h32_1, X1, Y1);
      // X1 et Y1 sont les positions réelles sur la surface de la table détectées par la Pixy1
    }
    Serial.println("Lecture Pixy 2");
    pixy2.ccc.getBlocks();
    Serial.println("Lecture Pixy 2 terminée");
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
      Serial.println("Pixy 1 et 2 ont détecté : définition du point 3D");
      // Serial.print(" | Écart (x,y) = ("); Serial.print(X2 - X1); Serial.print(","); Serial.print(Y2 - Y1); Serial.print(")");
      // Serial.print(" | Moyenne (x,y) = ("); Serial.print(min(X2,X1)+abs(X2 - X1)/2); Serial.print(","); Serial.print(min(Y2,Y1)+abs(Y2 - Y1)/2); Serial.print(")");
      Vect3D D1 = vecteur_directeur(C1, X1, Y1);
      Vect3D D2 = vecteur_directeur(C2, X2, Y2);
      P = point_plus_proche(C1, D1, C2, D2);
      Serial.print(" | Intersection approx_moteur= (");
      Serial.print(P.x); Serial.print(", "); Serial.print(P.y); Serial.print(", "); Serial.print(P.z); Serial.print(")");
    }

    if (pixy_detection > 0) {
      updateMotorPositionFromDetection(X1, Y1, X2, Y2, pixy_detection, P);
      UpdateControlerStatus();
    }

    // Affichage des paramètres actuels
    Serial.print("Moteurs : x_moteur = "); Serial.print(x_moteur);
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
  // AFFICHAGE DES PARAMÈTRES
    Serial.print("État système : ");
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
    Serial.println("Fin de boucle");
}