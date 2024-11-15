#pragma once
#include "comms.h"
#if USE_ARDUINO_H 
#include <Servo.h>
#else
#include "FakeServo.h"
#endif

/**
* Gestisce la legge di moto trapezoidale.
* 
* @param ci Le istruzioni per eseguire la legge di moto.
* @param s Il Servo motore da controllare.
*/
void treTratti(Instructions &ci, Servo &s);

