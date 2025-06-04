#include <Pixy2I2C.h>
#include <Wire.h>

// STRUCTURES DE BASE

struct Vect2D {float x, y;};
struct Vect3D {float x, y, z;};
struct Matrice {float m[3][2];};

Pixy2I2C pixy1;
Pixy2I2C pixy2;

// DONNÉES DE CALIBRATION : PIXY 1

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

// Coefficients d'homographie pour la caméra 1 (Pixy1)
float h11_1 = -3.0351983945;
float h12_1 = -4.3786490204;
float h13_1 = 1766.1399529285;
float h21_1 = 5.7492101108;
float h22_1 = -2.3268680892;
float h23_1 = -315.7513522178;
float h31_1 = 0.0002456224;
float h32_1 = 0.0057779114;
float h33_1 = 1.0000000000;

// DONNÉES DE CALIBRATION : PIXY 2

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

// Coefficients d'homographie pour la caméra 2 (Pixy2)
float h11_2 = 5.4297639615;
float h12_2 = 14.9604748676;
float h13_2 = -828.4476902040;
float h21_2 = -10.2207775396;
float h22_2 = 1.2830470768;
float h23_2 = 2350.6284267266;
float h31_2 = 0.0051780135;
float h32_2 = 0.0045239961;
float h33_2 = 1.0000000000;

// POSITIONS FIXES DES CAMÉRAS (mm)

const Vect3D C1 = {0, 0, 60};  // Caméra gauche
const Vect3D C2 = {60, 0, 60};  // Caméra droite

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

  Vect3D B1 = {C1.x + lambda*D1.x, C1.y + lambda*D1.y, C1.z + lambda*D1.z};
  Vect3D B2 = {C2.x + mu*D2.x, C2.y + mu*D2.y, C2.z + mu*D2.z};

  return {(B1.x + B2.x)/2, (B1.y + B2.y)/2, (B1.z + B2.z)/2};
}

// INITIALISATION

void setup() {
  // Wire.begin();
  Serial.begin(115200);
  Serial.println(">>> DÉMARRAGE <<<");

  pixy1.init(0x53);
  pixy2.init(0x54);

  pinMode(13, OUTPUT);  // LED diagnostic
}

void loop() {
  pixy1.ccc.getBlocks();
  float X1, Y1;
  if (pixy1.ccc.numBlocks) {
    int x1 = pixy1.ccc.blocks[0].m_x;
    int y1 = pixy1.ccc.blocks[0].m_y;
    transformer(x1, y1, h11_1, h12_1, h13_1, h21_1, h22_1, h23_1, h31_1, h32_1, X1, Y1);
    Serial.print("Pixy1 : ");
    Serial.print("x = "); Serial.print(X1);
    Serial.print(" mm, y = "); Serial.print(Y1); Serial.print(" mm");
  }

  pixy2.ccc.getBlocks();
  if (pixy2.ccc.numBlocks) {
    int x2 = pixy2.ccc.blocks[0].m_x;
    int y2 = pixy2.ccc.blocks[0].m_y;
    float X2, Y2;
    transformer(x2, y2, h11_2, h12_2, h13_2, h21_2, h22_2, h23_2, h31_2, h32_2, X2, Y2);
    Serial.print(" | Pixy2 : ");
    Serial.print("x = "); Serial.print(X2);
    Serial.print(" mm, y = "); Serial.print(Y2); Serial.print(" mm");
    Serial.print(" | Écart (x,y) = ("); Serial.print(X2 - X1); Serial.print(","); Serial.print(Y2 - Y1); Serial.print(")");
    Serial.print(" | Moyenne (x,y) = ("); Serial.print(min(X2,X1)+abs(X2 - X1)/2); Serial.print(","); Serial.print(min(Y2,Y1)+abs(Y2 - Y1)/2); Serial.print(")");

    Vect3D D1 = vecteur_directeur(C1, X1, Y1);
    Vect3D D2 = vecteur_directeur(C2, X2, Y2);

    Vect3D P = point_plus_proche(C1, D1, C2, D2);

    Serial.print(" | Intersection approx = (");
    Serial.print(P.x); Serial.print(", "); Serial.print(P.y); Serial.print(", "); Serial.print(P.z); Serial.print(")");
  }

  Serial.println("");

  delay(20);
}


