#include "Parameter.h"

uint8_t ControlerStatus = 0; // 0 erro / 1 Motor Off / 2 Reference / 3 Enable / 4 Manuel Mode
bool ManualMove = false;
bool AutoMode = false;  //hand manual move from the Camera
int x_moteur = 0; // lateral translation , max 900 min 5 mm
int y_moteur = 0; // depth translation , max 380 min 5 mm  
int rh = 0; // horizontal rotation , max 9500 min 5 ° 10^-2
int rv = 0; // vertical translation , max 33000 min 5 ° 10^-2
