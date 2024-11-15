
#include "comms.h"

#include "Menu.h"

#if !USE_ARDUINO_H
#include <iostream>
#include <string>
#include <vector>
#include "three.h"
#include "seven.h"
#include "spline.h"
#include "FakeServo.h"
#include "matlab.h"


#if ENABLE_ARDUINO_COMM
#include "communication.h"
#endif

Menu::Menu() {
	pin = 0;
}

void Menu::startMenu() {
	bool toDo = false;
	s = Servo();
	unsigned short pin = 0;
	//Queste linee di codice chiedono all'utente di inserire un pin valido per il servo motore
	//In questo caso si considerano i pin PWM di un Arduino UNO.
	do {
		if (toDo) {
			std::cout << "Inserisci un valore valido per il PIN (5,6,9,10,11,12)\n\n";
		}
		std::cout << "Pin del servo: ";
		std::cin >> pin;
		if (pin != 5 && pin != 6 && pin != 9 && pin != 10 && pin != 11 && pin != 12) {
			toDo = true;
		}
		else {
			toDo = false;
		}
	} while (toDo);
	s.attach(pin);
	this->pin = pin;

	//Queste linee di codice si occupano di mostrare il menù e richiedere
	//all'utente di scegliere un'operazione 
	while (true) {
		std::cout << "Scegli operazione:\n\t1) Tre tratti\n\t2) Sette tratti\n\t3) Spline\n\t0) Esci\nInserisci scelta: ";
		char v = -1;
		std::cin >> v;
		switch (v) {
		case '1':
			handleTreTratti();
			break;
		case '2':
			handleSetteTratti();
			break;
		case '3':
			handlerSplineCubica();
			break;
		case '0':
			std::cout << "Uscita...\n";
			return;
		default:
			std::cout << "\n\n\tOperazione non valida...\n\n";
			break;
		}
	}
}

/*
Membro statico per gestire l'inserimento dei vari parametri in modo da generare
le istruzioni iniziali.
*/
static Instructions generateInstructions(bool sette_tratti) {
	CondizioniIniziali ci{ 0,0,0 };
	std::cout << "\n------------------\nCONDIZIONI INIZIALI\n------------------\n";
	std::cout << "\nAngolo iniziale (°): ";
	std::cin >> ci.angolo_inizio;
	std::cout << "\nVelocita' iniziale (°/s): ";
	std::cin >> ci.velocita_inizio;
	if (sette_tratti) {
		std::cout << "\nAccelerazione iniziale (°/s^2): ";
		std::cin >> ci.accelerazione_inizio;
	}

	Lambdas lmbs{};
	std::cout << "\n------------------\nCOEFF. TRATTI\n------------------\n";
	if (sette_tratti) {
		std::cout << "\nCoeff. tratti acc. lineare : ";
		std::cin >> lmbs.increm_lin;
	}
	std::cout << "\nCoeff. acc.cost. T1: ";
	std::cin >> lmbs.cost_1;
	std::cout << "\nCoeff. acc.cost. T2: ";
	std::cin >> lmbs.cost_2;
	std::cout << "\nCoeff. acc.cost. T3: ";
	std::cin >> lmbs.cost_3;

	if ((lmbs.cost_1 + lmbs.cost_2 + lmbs.cost_3 + lmbs.increm_lin * 4) != 1.0) {
		std::cout << "\n!! I coefficienti scelti non hanno somma pari ad 1 !!\n";
	}

	Instructions inst{ ci,0,lmbs,0 };
	std::cout << "\n------------------\nDATI\n------------------\n";
	std::cout << "\nDelta angolo da percorrere (°): ";
	std::cin >> inst.delta_angolo;
	std::cout << "\nTempo di percorrenza (ms): ";
	std::cin >> inst.tempo_tot_ms;

	return inst;
}

void Menu::handleTreTratti() const {
#if MATLAB_COMPILE
	std::cout << "Inserisci il nome del file MATLAB da generare: ";
	std::string nome_file;
	do {
		std::cin >> nome_file;
	} while (nome_file.empty());
	nome_file += ".m";
	MatLAB* M = createInstance(nome_file.c_str());
#endif

	Instructions inst = generateInstructions(false);

	treTratti(inst, s);
#if MATLAB_COMPILE
	M->writeData();
	M->flushData();
	M->addPlotting("3-Tratti");
	delete M;
#endif
#if ENABLE_ARDUINO_COMM
	CommDataInstruction* cdi = requestServoInstructionData(this->pin, inst.ci, inst.delta_angolo,
		inst.lambdas, inst.tempo_tot_ms, LawType::TRE_TRATTI);
	sendServoInstruction("COM4", cdi);
#endif

	std::cout << "\n\n-------------------------\n\tCOMPLETATO\n-------------------------\n\n";
}

void Menu::handleSetteTratti() const {
#if MATLAB_COMPILE
	std::cout << "Inserisci il nome del file MATLAB da generare: ";
	std::string nome_file;
	do {
		std::cin >> nome_file;
	} while (nome_file.empty());
	nome_file += ".m";
	MatLAB* M = createInstance(nome_file.c_str());
#endif

	Instructions inst = generateInstructions(true);

	setteTratti(inst, s);
#if MATLAB_COMPILE
	M->writeData();
	M->flushData();
	M->addPlotting("7-Tratti");
	delete M;
#endif
#if ENABLE_ARDUINO_COMM
	CommDataInstruction* cdi = requestServoInstructionData(this->pin, inst.ci, inst.delta_angolo,
		inst.lambdas, inst.tempo_tot_ms, LawType::SETTE_TRATTI);
	sendServoInstruction("COM4", cdi);
#endif

	std::cout << "\n\n-------------------------\n\tCOMPLETATO\n-------------------------\n\n";
}


void Menu::handlerSplineCubica() const {
#if MATLAB_COMPILE
	std::cout << "Inserisci il nome del file MATLAB da generare: ";
	std::string nome_file;
	do {
		std::cin >> nome_file;
	} while (nome_file.empty());
	nome_file += ".m";
	MatLAB* M = createInstance(nome_file.c_str());
#endif
#if ENABLE_ARDUINO_COMM
	Point puntiComm[MAX_LENGTH_POINTS]{};
#endif

	std::vector<double> tempi;
	std::vector<double> punti;
	int index = 0;
	double t{ 0 }, p{ 0 };
	bool toDo = true;
	std::cout << "inserisci le coppie tempo-spazio per costruire la spline.\n(Inserisci un numero negativo nel tempo\nper interrompere l'inserimento)\n\n";
	while (toDo) {
		std::cout << "Tempo (" << index << ") :";
		std::cin >> t;
		toDo = !(t < 0);
		if (toDo) {
			std::cout << "Spazio (" << index << ") :";
			std::cin >> p;
			tempi.push_back(t);
			punti.push_back(p);
			puntiComm[index] = { t,p };
			index++;
		}
	}
	spline(tempi, punti, s);
#if MATLAB_COMPILE
	M->writeData();
	M->flushData();
	M->addPlotting("Spline Cubica");
	delete M;
#endif
#if ENABLE_ARDUINO_COMM
	CommDataInstruction* cdi = requestServoInstructionData(this->pin, index, puntiComm);
	sendServoInstruction("COM4", cdi);
#endif

	std::cout << "\n\n-------------------------\n\tCOMPLETATO\n-------------------------\n\n";
}
#endif