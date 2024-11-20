#include "comms.h"

#if USE_ARDUINO_H
#include "Servo.h"
#else
#include "FakeServo.h"
#endif

/**
* Gestisce l'insieme di tratti partendo da una serie di istruzioni
* e, tramite i risultati dei singoli tratti, viene comandato il servomotore.
* 
* @param ci Le istruzioni da utilizzare per la legge di moto
* @param s Il servo motore da controllare
*/
void setteTratti(Instructions ci, Servo *s);
/*
Tratto a jerk costante positivo e accelerazione.
*/
InfoTratto tratto1(InfoTratto tratto0, double jerk, Servo *s);
/*
Tratto a jerk costante nullo e accelerazione costante positiva.
*/
InfoTratto tratto2(InfoTratto tratto1, double jerk, Servo *s);
/*
Tratto a jerk costante negativo e decelerazione.
*/
InfoTratto tratto3(InfoTratto tratto2, double jerk, Servo *s);
/*
Tratto a jerk costante nullo e accelerazione nulla
*/
InfoTratto tratto4(InfoTratto tratto3, double jerk, Servo *s);
/*
Tratto a jerk costante negativo e decelerazione.
*/
InfoTratto tratto5(InfoTratto tratto4, double jerk, Servo *s);
/*
Tratto a jerk costante nullo e accelerazione costante negativa.
*/
InfoTratto tratto6(InfoTratto tratto5, double jerk, Servo *s);
/*
Tratto a jerk costante positivo e accelerazione.
*/
InfoTratto tratto7(InfoTratto tratto6, double jerk, Servo *s);