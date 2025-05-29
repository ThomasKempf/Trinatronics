#ifndef PARAMETER_H
#define PARAMETER_H
#include <stdint.h>


// Déclaration des variables globales
<<<<<<< HEAD
extern char ControlerStatus; // Variable de renvoi qui me dit dans quel mode je suis.
extern int x; // lateral translation mm
extern int y; // depth translation
extern int rh; // horizontal rotation °*10^-2
extern int rv; // vertical rotation
=======
extern uint8_t ControlerStatus; // 0 erro / 1 Motor Off / 2 Reference / 3 Enable / 4 Manuel Mode
extern int x; // lateral translation , max 900 min 5 mm
extern int y; // depth translation , max 900 min 5 mm   MAX IS NOT CORRECT DANGER
extern int rh; // horizontal rotation , max 9500 min 5 ° 10^-2
extern int rv; // vertical translation , max 33000 min 5 ° 10^-2
>>>>>>> preparation_nanotec_arduino
extern bool ManualMove; //hand manual move from the Camera
extern bool AutoMode;

#endif