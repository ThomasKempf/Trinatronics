#ifndef PARAMETER_H
#define PARAMETER_H
#include <stdint.h>


// Déclaration des variables globales
extern uint8_t ControlerStatus; // 0 erro / 1 Motor Off / 2 Reference / 3 Enable / 4 Manuel Mode
extern int x_moteur; // lateral translation , max 900 min 5 mm
extern int y_moteur; // depth translation , max 380 min 5 mm 
extern int32_t rh; // horizontal rotation , max 950 min 5 ° 10^-2
extern int32_t rv; // vertical translation , max 33000 min 1000 ° 10^-2
extern bool ManualMove; //hand manual move from the Camera
extern bool AutoMode;

#endif