#pragma once

#ifndef SERVO_FAKE
#define SERVO_FAKE 1

#include <stdio.h>

#include "FakeArduino.h"
/**
* Classe che simula la libreria "Servo" di Arduino in modo che il codice possa
* essere compilato per piattaforme Windows senza aver necessità di utilizzare
* le librerie di arduino
*/
class Servo {
public:
	/**
	* Simula la connessione ad un servomotore
	* 
	* @param Il pin a cui è connesso il servo motore
	*/
	void attach(int);
	/*
	* Simula la scrittura del valore ad un servomotore
	* 
	* @param L'angolo in gradi a cui portare il servo motore.
	*/
	void write(double);
};

#endif