#ifndef PARAMETER_H
#define PARAMETER_H
#include <stdint.h>


// Déclaration des variables globales
extern uint8_t ControlerStatus;
extern int x; // lateral translation
extern int y; // depth translation
extern int rh; // horizontal rotation
extern int rv; // vertical translation
extern bool ManualMove; //hand manual move from the Camera
extern bool AutoMode;

#endif