#ifndef PARAMETER_H
#define PARAMETER_H

// Déclaration des variables globales
extern char ControlerStatus; // Variable de renvoi qui me dit dans quel mode je suis.
extern int x; // lateral translation mm
extern int y; // depth translation
extern int rh; // horizontal rotation °*10^-2
extern int rv; // vertical rotation
extern bool ManualMove; //hand manual move from the Camera
extern bool AutoMode;

#endif