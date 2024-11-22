#pragma once
#if !USE_ARDUINO_H
#include "FakeServo.h"

/**
* Questa classe viene compilata solo se su un dispositivo Windows e serve per 
* gestire le interazioni dell'utente nella console (input dei dati e mostrare
* il testo)
* 
* @class
*/
class Menu
{
private:
	Servo s;
	int pin;

public:
	/**
	* Crea un nuovo gestore del menu
	*/
	Menu();

	/**
	* Gestisce il ciclo del menu.
	*/
	void startMenu();
	/**
	* Gestisce le richieste per eseguire una simulazione
	* di una legge di moto trapezoidale.
	*/
	void handleTreTratti();
	/**
	* Gestisce le richieste per eseguire una simulazione
	* di una legge di moto sette tratti.
	*/
	void handleSetteTratti();
	/**
	* Gestisce le richieste per eseguire una simulazione
	* tramite spline.
	*/
	void handlerSplineCubica();
};

#endif

