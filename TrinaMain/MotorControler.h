#ifndef MOTORCONTROLER_H
#define MOTORCONTROLER_H


void ReadControler(char SlaveID);
void ReadAllControler();
void UpdateOrder();
int DefineSetPoint(char SlaveID);
void WriteAllControler();
void UpdateControlerStatus();

#endif