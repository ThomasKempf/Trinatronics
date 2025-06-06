#include <Pixy2I2C.h>
#include "Parameter.h"
#include "MotorControler.h"
#include <avr/wdt.h>

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

const int decalage_x = 60; // Décalage entre x réel et x moteur
const int decalage_y = 200; // Décalage entre y réel et y moteur

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
//     [102, 25],
//     [167,36],
//     [212,162],
//     [130,162]
// ], dtype=np.float32)
// # Points réels sur le plan de travail (en mm)
// pts_reels = np.array([
//     [973, 120],
//     [970,460],
//     [273,453],
//     [300,165]
// ], dtype=np.float32)

float h11_1 = -1.0798538009;
float h12_1 = -2.9432463037;
float h13_1 = 1030.8508606266;
float h21_1 = 3.8044260968;
float h22_1 = 0.1205507281;
float h23_1 = -286.5894296226;
float h31_1 = -0.0022924263;
float h32_1 = 0.0041783661;
float h33_1 = 1.0000000000;

// pixy = "_2"
// # Points dans l'image (pixels)
// pts_image = np.array([
//     [171,39],
//     [110,44],
//     [161, 195],
//     [61, 173]
// ], dtype=np.float32)
// # Points réels sur le plan de travail (en mm)
// pts_reels = np.array([
//     [300,165],
//     [300,455],
//     [943,110],
//     [938,445]
// ], dtype=np.float32)

// Coefficients d'homographie pour la Pixy2
float h11_2 = 2.4369801649;
float h12_2 = 16.0262851207;
float h13_2 = -421.2770749030;
float h21_2 = -8.1612838871;
float h22_2 = -0.3955343935;
float h23_2 = 1752.2647951977;
float h31_2 = 0.0044061106;
float h32_2 = 0.0080716394;
float h33_2 = 1.0000000000;

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
    x_target = (X1 + X2) / 2.0;
    y_target = (Y1 + Y2) / 2.0;
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

  if (distance > 500) {
    x_moteur = constrain((int)x_moteur_calc, x_moteur_min, x_moteur_max);
    Serial.print("x_moteur : "); Serial.println(x_moteur);
    y_moteur = constrain((int)y_moteur_calc, y_moteur_min, y_moteur_max);
    Serial.print("y_moteur : "); Serial.println(y_moteur);
  }

  // === CALCUL DES ANGLES DE ROTATION ===
  float angle_v_rad = atan2(dy, dx);
  float angle_v_deg = (angle_v_rad * 180.0 / PI +90);
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

  Application des contraintes
  rh = constrain(angle_h_deg100, rh_min, rh_max);
  rv = constrain(angle_v_deg100, rv_min, rv_max);
  rh = 0;
  rv = 18000;

  Serial.print("rh final : "); Serial.println(rh);
  Serial.print("rv final : "); Serial.println(rv);
  Serial.print("UpdateControler : ("); Serial.print(x_moteur); Serial.print(","); Serial.print(y_moteur); Serial.print(","); Serial.print(rh); Serial.print(","); Serial.print(rv); Serial.println(")");
}


// ----- SETUP -----

void setup() {
  pinMode(pinOnOff, INPUT_PULLUP);      // SWITCH défini en entrée
  pinMode(pinButtonMode, INPUT_PULLUP); // BP défini en entrée
  pinMode(pinLedButton, OUTPUT);        // BP : LED définie en sortie
  pinMode(pinRelay, OUTPUT);            // LED : Relais défini en sortie
  pinMode(pinManualMove, INPUT_PULLUP); // BP : Mode manuel


  Serial.begin(115200);                 // Initialisation du port série
  setupModbus();
  referenceProtocol();
  Serial.println("--- Démarrage du système | Programme général ---");

  pixy1.init(0x54);
  delay(20)
  pixy2.init(0x53);
  wdt_enable(WDTO_2S);
  Serial.println("Pixy initialisées");
}

void loop() {
  wdt_reset();
  bool systemOn = (digitalRead(pinOnOff) == HIGH); // SWITCH : Lecture de l'état de l'interrupteur ON/OFF
  digitalWrite(pinRelay, systemOn ? LOW : HIGH); // LED : Si le système est ON, activation du relais | Attention ! Le relais est inversé, la lampe est allumée lorsque LOW est envoyé.
  int pixy_detection = 0; // Nombre de détection par les Pixy (0, 1, 2 ou 3 si les 2 pixy détectent l'objet)
  float X1 = 0.0, Y1 = 0.0, X2 = 0.0, Y2 = 0.0; // Poistions réelles sur la surface détectées par les Pixy 1 et 2
  Vect3D P={0.0,0.0,0.0}; // Position réelle (x,y,z) calculée si détection par les 2 Pixy
  unsigned long currentMillis = millis();

  // Serial.println("Départ boucle");

  if (systemOn) {
    if (!lastStateOn){
      lastStateOn = true;
      previousMillis = currentMillis;
    }
    // Serial.println("Read pinButtonMode");
    bool currentButtonState = digitalRead(pinButtonMode); //BP : Lecture
    if (currentButtonState != lastButtonState && millis() - lastDebounceTime > debounceDelay) { 
      if (currentButtonState == LOW) {
        mode = 1 - mode;  // Si état changé (front) + après délai de stabilisation, inversion du mode (manuel/auto)
      }
      lastDebounceTime = millis(); // Mémorisation temps de changement
    }
    lastButtonState = currentButtonState; // Mise à jour état précédent
    
    // Serial.println("Read pinManualMove");
    // GESTION DU BOUTON MANUEL (pinManualMove)
    bool manualPressed = !digitalRead(pinManualMove);

    if (manualPressed) {
      ManualMove = true;
      AutoMode = false;
      mode = 1;
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
    if (mode==0){
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
      delay(30);
      Serial.println("Lecture Pixy 2");
      Serial.print(pixy2.ccc.getBlocks());
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
        Serial.print("Pixy 1 et 2 ont détecté quelque chose :");
        Serial.print(" Moyenne (x,y) = ("); Serial.print(min(X2,X1)+abs(X2 - X1)/2); Serial.print(","); Serial.print(min(Y2,Y1)+abs(Y2 - Y1)/2); Serial.print(")");
        Vect3D D1 = vecteur_directeur(C1, X1, Y1);
        Vect3D D2 = vecteur_directeur(C2, X2, Y2);
        P = point_plus_proche(C1, D1, C2, D2);
      }
    }

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
    Serial.println(pixy_detection);
    if (pixy_detection%2 == 1){
      Serial.print(" | Pixy 1 - Surface ("); Serial.print(X1); Serial.print(","); Serial.print(Y1); Serial.print(")");
    }
    if (pixy_detection > 1){
      Serial.print(" | Pixy 2 - Surface ("); Serial.print(X2); Serial.print(","); Serial.print(Y2); Serial.print(")");
    }
    // if (pixy_detection == 3){
    //   Serial.print(" | Position absolue estimée = (");
    //   Serial.print(P.x); Serial.print(", "); Serial.print(P.y); Serial.print(", "); Serial.print(P.z); Serial.print(")");
    // }
    if (systemOn){
      Serial.print(" | Temps depuis switch sur ON (ms) = ");
      Serial.print(currentMillis - previousMillis);
    }
    Serial.println("");
    // Serial.println("Fin de boucle");
}