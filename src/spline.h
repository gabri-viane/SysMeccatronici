#pragma once
#include "comms.h"

#if USE_ARDUINO_H
#include <Servo.h>
#else
#include "FakeServo.h"
#endif

#include <vector>

/**
* Funzione che serve per eseguire l'interpolazione tra punti utilizzando la tecnica spline cubica.
* Necessita di due vettori di uguali dimensioni che contengono i punti e il corrispondente tempo.
* 
* @param tempi Un vettore contenente la lista di tempi dei vari punti simulati
* @param punti Un vettore contenente la lista di posizione dei vari punti simulati
* @param s Il servo motore da controllare
*/
void spline(std::vector<double> &tempi, std::vector<double> &punti, Servo *s);