#include "communication.h"

#include "comms.h"

#if SIMULATE_COMMUNICATION
#include "Simulation.h"
#include "three.h"
#include "seven.h"
#include "spline.h"
#include <vector>
#endif

#if USE_ARDUINO_H
#include <Arduino.h>
#include "three.h"
#include "seven.h"
#include "spline.h"
#include <vector>
#else
#include <Windows.h>
#include <thread>
#include <iostream>
#include <string>

constexpr const char* COM1 = "COM1";
constexpr const char* COM2 = "COM2";
constexpr const char* COM3 = "COM3";
constexpr const char* COM4 = "COM4";
#endif

// Questa sezione viene compilata solo per arduino o in caso di simulazione
// della connessione (per testare se la comunicazione funziona correttamente, 
// ovvero dell'invio dei dati)
#if USE_ARDUINO_H || SIMULATE_COMMUNICATION
/**
* Funzione statica: compilata solo nello scopo di questo file
* 
* Controlla che tipologia di istruzione ha ricevuto e interpreta in modo
* corretto i dati che ha ricevuto.
*/
static void executeInstruction(CommInstruction* inst) {
	if (inst->sizeReceived > 2) {
		switch (inst->message[0]) {
		case CommRequestType::SERVO_WRITE: //Se la richiesta è di tipo SERVO_WRITE (ovvero comando servo)
			inst->commType = CommRequestType::SERVO_WRITE;
			inst->inst.swi = parseServoWrite(inst);
			break;
		case CommRequestType::SERVO_READ: //Se la richiesta è di tipo SERVO_READ (ovvero lettura servo)
			inst->commType = CommRequestType::SERVO_READ;
			break;
		case CommRequestType::SENSOR_READ: //Se la richiesta è di tipo SENSOR_READ (ovvero lettura sensore)
			inst->commType = CommRequestType::SENSOR_READ;
			break;
		default:
			inst->commType = CommRequestType::NULL_REQ;
			// Istruzione non valida
			return;
		}
	}
}

void parseInstruction(CommInstruction* inst,Servo &s) {
	char data;
	switch (inst->status) {
	case STATUS_VOID:
		if (Serial.available()) {
			data = Serial.read();
			if (data == STARTING_STREAM_CHAR) {
				inst->status = STATUS_START_READING;
			}
		}
		break;
	case STATUS_START_READING:
		if (Serial.available()) {
			data = Serial.read();
			if (data == STARTING_STREAM_CHAR) {
				inst->status = STATUS_READING;
			}
			else {
				inst->status = STATUS_VOID;
			}
		}
		break;
	case STATUS_READING:
		if (Serial.available()) {
			data = Serial.read();
			if (data == CLOSING_STREAM_CHAR) {
				inst->status = STATUS_SHOULD_COMPLETE;
			}
			inst->message[inst->sizeReceived] = data;
			inst->sizeReceived++;
		}
		break;
	case STATUS_SHOULD_COMPLETE:
		if (Serial.available()) {
			data = Serial.read();
			if (data == CLOSING_STREAM_CHAR) {
				inst->status = STATUS_COMPLETED;
			}
			else {
				inst->status = STATUS_READING;
			}
			inst->message[inst->sizeReceived] = data;
			inst->sizeReceived++;
		}
		break;
	case STATUS_COMPLETED:
		executeInstruction(inst);
		switch (inst->inst.swi->law) {
		case TRE_TRATTI:
			treTratti(*inst->inst.swi->Dati_Tre_Sette_Tratti, s);
			break;
		case SETTE_TRATTI:
			setteTratti(*inst->inst.swi->Dati_Tre_Sette_Tratti, s);
			break;
		case SPLINE:
			executeSplineConversion(inst, s);
			break;
		}
		inst->status = STATUS_VOID;
		memset((void*)inst->message, (char)0, inst->sizeReceived);
		inst->sizeReceived = 0;
		break;
	}
}

void executeSplineConversion(CommInstruction* inst, Servo &s) {
	Point *currentPoint = &(inst->inst.swi->Dati_Spline->points[0]);
	std::vector<double> tempi;
	std::vector<double> spazio;
	for (short i = 0; i < MAX_LENGTH_POINTS; i++, currentPoint++) {
		tempi.push_back(currentPoint->time);
		spazio.push_back(currentPoint->position);
	}
	spline(tempi, spazio, s);
}

ServoWriteInst* parseServoWrite(CommInstruction* ci) {
	ServoWriteInst *result {nullptr};
	switch (ci->message[2]) {
	case LawType::TRE_TRATTI:
		result = new ServoWriteInst(ci->message[1], LawType::TRE_TRATTI);
		memcpy((void*)(result->Dati_Tre_Sette_Tratti),(void*)(&(ci->message[3])),sizeof(Instructions));
		break;
	case LawType::SETTE_TRATTI:
		result = new ServoWriteInst(ci->message[1], LawType::SETTE_TRATTI);
		memcpy((void*)(result->Dati_Tre_Sette_Tratti),(void*)(&(ci->message[3])),sizeof(Instructions));
		break;
	case LawType::SPLINE:
		result = new ServoWriteInst(ci->message[1], LawType::SPLINE);
		memcpy((void*)(result->Dati_Spline),(void*)(&(ci->message[3])),sizeof(Instructions));
		break;
	}
	return result;
}
#endif

#if !USE_ARDUINO_H

static bool WriteComPort(const char* portSpecifier, const char* data, const size_t sizeToWrite) {
	DCB dcb;
	DWORD byteswritten;
	HANDLE hPort = CreateFileA(portSpecifier, GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
	if (hPort == INVALID_HANDLE_VALUE) {
		if (GetLastError() == ERROR_FILE_NOT_FOUND) {
			std::cout << "Errore: serial port does not exist (" << GetLastError() << ")" << std::endl;
		}
		else {
			std::cout << "Errore: some other error occurred (" << GetLastError() << ")" << std::endl;
		}
	}
	if (!GetCommState(hPort, &dcb)) {
		std::cout << "Errore:" << GetLastError() << std::endl;
		return false;
	}
	dcb.BaudRate = CBR_9600;    // 9600 Baud
	dcb.ByteSize = 8;           // 8 data bits
	dcb.Parity = NOPARITY;      // no parity
	dcb.StopBits = ONESTOPBIT;  // 1 stop
	dcb.fDtrControl = DTR_CONTROL_DISABLE;
	if (!SetCommState(hPort, &dcb)) {
		return false;
	}
	PurgeComm(hPort, PURGE_RXCLEAR | PURGE_TXCLEAR);

	bool retVal = WriteFile(hPort, data, sizeToWrite, &byteswritten, NULL);
	CloseHandle(hPort);  // close the handle
	return retVal;
}

CommDataInstruction* requestServoInstructionData(int pin, CondizioniIniziali ci, double delta_angolo, Lambdas lmbds, unsigned long long tempo_tot_ms, LawType type) {
	if (type == SPLINE) {
		return nullptr;
	}
	ServoWriteInst* swi = new ServoWriteInst(pin, type);
	swi->Dati_Tre_Sette_Tratti->ci = ci;
	swi->Dati_Tre_Sette_Tratti->delta_angolo = delta_angolo;
	swi->Dati_Tre_Sette_Tratti->lambdas = lmbds;
	swi->Dati_Tre_Sette_Tratti->tempo_tot_ms = tempo_tot_ms;

	CommDataInstruction* inst = new CommDataInstruction();
	inst->swi = swi;
	return inst;
}

CommDataInstruction* requestServoInstructionData(int pin, int size, Point points[]) {
	if (size > MAX_LENGTH_POINTS) {
		size = MAX_LENGTH_POINTS;
	}
	ServoWriteInst* swi = new ServoWriteInst(pin, LawType::SPLINE);
	memcpy((void*)&(swi->Dati_Spline->points), points, sizeof(Point) * size);
	swi->Dati_Spline->size = size;

	CommDataInstruction* inst = new CommDataInstruction();
	inst->swi = swi;
	return inst;
}

//Elimina in automatico la CommDataInstrucion creata
void sendServoInstruction(const char* portSpecifier, CommDataInstruction* inst) {
	// STRUTTURA BYTE DATI
	// Richiesta di scrittura
	//  0: 1
	// Pin del servo
	//  1: pin
	// Legge da usare
	//  2: legge di moto
	// Istruzioni di scrittura
	// successivi bytes
	char instruction[MAX_MESSAGE_LENGTH]{}; 
	instruction[0] = CommRequestType::SERVO_WRITE;
	instruction[1] = inst->swi->servoPin;
	instruction[2] = inst->swi->law;

	unsigned int size{ 3 };
	// SE LA LEGGE E' SPLINE:
	if (inst->swi->law == SPLINE) {
		// 6: lunghezza della lista di dati che devi inviare (il numero di punti)
		instruction[3] = inst->swi->Dati_Spline->size;
		size++;

		// Calcolo al dimensione totale delle cose da copiare
		size += instruction[3] * sizeof(Point); //Ovvero quanti byte occupano i punti

		for (int i = 0; i < inst->swi->Dati_Spline->size; i++) {
			// Copio i punti nell'array
			memcpy((void*)instruction[i * sizeof(Point)],		// inserisco il puntatore alla posizione corrente dell'array da inviare
				(void*) &(inst->swi->Dati_Spline->points[i]),	// il puntatore al punto da copiare
				sizeof(Point));									// La dimensione da copiare: ovvero il punto
		}
	}
	else if (inst->swi->law == COMANDO_DIRETTO) {
		instruction[3] = inst->swi->directDrive;
		size++;
	}
	else {
		memcpy((void*)&(instruction[3]), (void*)(inst->swi->Dati_Tre_Sette_Tratti), sizeof(Instructions));
		size += sizeof(Instructions);
	}
#if SIMULATE_COMMUNICATION
	char message[MAX_MESSAGE_LENGTH]{};
	message[0] = STARTING_STREAM_CHAR;
	message[1] = STARTING_STREAM_CHAR;
	message[2 + size] = CLOSING_STREAM_CHAR;
	message[3 + size] = CLOSING_STREAM_CHAR;
	//Ricopio i dati del messaggio tra i caratteri di inizio e fine della trasmissione
	memcpy((void*)&(message[2]), (void*)&(instruction[0]), size);
	Serial.insertBuffer(message, size+4);
#else
	WriteComPort(portSpecifier, instruction, size);
#endif // SIMULATE_COMMUNICATION

	delete inst;
}

bool communicateData(const char* portSpecifier, char* str, const unsigned int size)
{
	DCB commProps;
	DWORD bytesScritti;
	HANDLE handleStream = CreateFileA(portSpecifier, GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
	if (handleStream == INVALID_HANDLE_VALUE) {
		if (GetLastError() == ERROR_FILE_NOT_FOUND) {
			std::cout << "Error: serial port does not exist (" << GetLastError() << ")" << std::endl;
		}
		else {
			std::cout << "Error: some other error occurred (" << GetLastError() << ")" << std::endl;
		}
		return false;
	}
	//Prendo le proprietà della porta scelta e le salvo in commProps
	if (!GetCommState(handleStream, &commProps)) {
		std::cout << "Error retreiving commState: error n." << GetLastError() << std::endl;
		return false;
	}
	//Aggiorno le impostazioni della porta per comunicare con arduino
	commProps.BaudRate = CBR_9600;    // 9600 Baud
	commProps.ByteSize = 8;           // 8 data bits
	commProps.Parity = NOPARITY;      // no parity
	commProps.StopBits = ONESTOPBIT;  // 1 stop
	commProps.fDtrControl = DTR_CONTROL_DISABLE;
	//Imposto le nuove proprità per la porta scelta
	if (!SetCommState(handleStream, &commProps)) {
		std::cout << "Error setting commState: error n." << GetLastError() << std::endl;
		return false;
	}
	//Pulisco i dati del buffer di ricezione e di invio
	PurgeComm(handleStream, PURGE_RXCLEAR | PURGE_TXCLEAR);

	const unsigned short resto = (size + 4) % MAX_BUFFER_SIZE;
	const unsigned short cicli = (size + 4 - resto) / MAX_BUFFER_SIZE;
	//Scrivo i valori di inizio e fine trasmissione nel messaggio
	char message[MAX_MESSAGE_LENGTH]{};
	message[0] = STARTING_STREAM_CHAR;
	message[1] = STARTING_STREAM_CHAR;
	message[2 + size] = CLOSING_STREAM_CHAR;
	message[3 + size] = CLOSING_STREAM_CHAR;
	//Ricopio i dati del messaggio tra i caratteri di inizio e fine della trasmissione
	memcpy((void*)&(message[2]), (void*)&(str[0]), size);

	//Buffer che invio di volta in volta alla scheda
	char buffer[MAX_BUFFER_SIZE]{};
	//Per il numero di cicli da fare invio il buffer
	for (int i = 0; i < cicli; i++) {
		memcpy((void*)(&buffer[0]), (void*)(&message[i * MAX_BUFFER_SIZE]), MAX_BUFFER_SIZE);
		bool retVal = WriteFile(handleStream, buffer, MAX_BUFFER_SIZE, &bytesScritti, NULL);
		if (!retVal) {
			return false;
		}
		//Faccio una pausa per permettere ad arduino di ricevere ed elaborare parte del 
		// messaggio per liberare il buffer
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
	//Ora devo inviare i dati rimanenti se ho un resto.
	if (resto > 0) {
		memcpy((void*)(&buffer[0]), (void*)(&message[cicli * MAX_BUFFER_SIZE]), resto);
		bool retVal = WriteFile(handleStream, buffer, resto, &bytesScritti, NULL);
		if (!retVal) {
			return false;
		}
	}

	CloseHandle(handleStream);  // close the handle
	return true;
}

/*
void TestCommunication() {
	CommDataInstruction* cdi = requestServoInstructionData(3, { 0,0,0 }, 180, { 0.2, 0.6, 0, 0 }, 5000, LawType::TRE_TRATTI);
	sendServoInstruction(COM4, cdi);
	// ReadComPort();
}*/

#endif

ServoWriteInst::ServoWriteInst(char Pin, LawType lt) {
	this->servoPin = Pin;
	this->directDrive = 0;
	this->law = lt;
	if (lt == SPLINE) {
		this->Dati_Spline = new SplineInstructionData();
		this->Dati_Tre_Sette_Tratti = nullptr;
	}
	else {
		this->Dati_Tre_Sette_Tratti = new Instructions();
		this->Dati_Spline = nullptr;
	}
}

ServoWriteInst::~ServoWriteInst() {
	if (this->Dati_Spline) {
		delete this->Dati_Spline;
	}
	if (this->Dati_Tre_Sette_Tratti) {
		delete this->Dati_Tre_Sette_Tratti;
	}
}

CommDataInstruction::CommDataInstruction() {
	swi = nullptr;
}

CommDataInstruction::~CommDataInstruction() {
	if (swi) {
		delete swi;
		swi = nullptr;
	}
}

CommInstruction::CommInstruction(char *buffer)
{
	this->commType = CommRequestType::NULL_REQ;
	this->sizeReceived = 0;
	this->status = STATUS_VOID;
	this->message = buffer;
}

CommInstruction::~CommInstruction() {
	if (this->inst.swi) {
		delete this->inst.swi;
		this->inst.swi = nullptr;
	}
}
