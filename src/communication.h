#pragma once
#ifndef COMMUNICATION_INCLUDED
#define COMMUNICATION_INCLUDED 1

#include <vector>

#include "comms.h"

typedef struct ServoWriteInst {
	Instructions* Dati_Tre_Sette_Tratti;
	SplineInstructionData* Dati_Spline;
	char servoPin;
	unsigned short directDrive;
	LawType law;

	ServoWriteInst(char Pin, LawType lt);
	~ServoWriteInst();
};

enum CommRequestType {
	NULL_REQ = 0,
	SERVO_WRITE = 1,
	SERVO_READ = 2,
	SENSOR_READ = 3
};

typedef struct CommDataInstruction {
	ServoWriteInst* swi;

	CommDataInstruction();
	~CommDataInstruction();
};

//Macchina a stati per la lettura del messaggio in arrivo
#define STATUS_VOID 0				//Indica che se si riceve il carattere  qualsiasi si passa a STATUS_START_READING
#define STATUS_START_READING 1      //Indica che se si riceve un carattere qualsiasi si passa a STATUS_START_READING
#define STATUS_READING 2			//Indica che se il prossimo carattere è CLOSING_STREAM_CHAR si passa a STATUS_SHOULD_COMPLETE
#define STATUS_SHOULD_COMPLETE 3 	//Indica che se il prossimo carattere è CLOSING_STREAM_CHAR si passa a STATUS_COMPLETE
#define STATUS_COMPLETED 4			//In questo stato il messaggio è stato completamente ricevuto e pronto per essere interpretato

typedef struct CommInstruction {
	char status;
	char* message;
	unsigned short sizeReceived;
	CommDataInstruction inst;
	CommRequestType commType;
	CommInstruction(char* buffer);
	~CommInstruction();
};
//Parte arduino oppure simulazione
#if USE_ARDUINO_H || SIMULATE_COMMUNICATION
/**
* Deve essere chiamata ciclicamente nella funzione "loop" di arduino.
* Si occupa di gestire i dati in ingresso nel buffer della seriale per controllare
* se i dati in ingressi sono una richiesta di comando o lettura.
* 
* Il funzionamento è simile ad una macchina a stati: inizialmente, se non si stanno ricevendo
* dei dati si è nello stato STATUS_VOID. La comunicazione sulla serial deve iniziare con due 
* byte STARTING_STREAM_CHAR, una volta ricevuti si passa nello stato STATUS_READING. Per concludere
* la comunicazione e eseguire il comando ricevuto bisogna inviare due byte in sequenza 
* CLOSING_STREAM_CHAR.
* 
* @param CommInstruction* inst : richiede che alla struttura venga assegnato un valore a message con 
		un buffer già predisposto.
* @param Servo s
*/
void parseInstruction(CommInstruction* inst, Servo &s);

void executeSplineConversion(CommInstruction* inst, Servo &s);

ServoWriteInst* parseServoWrite(CommInstruction* ci);
#endif
//Parte solo PC
#if !USE_ARDUINO_H
#include <string>

CommDataInstruction* requestServoInstructionData(int pin, CondizioniIniziali ci, double delta_angolo, Lambdas lmbds, unsigned long long tempo_tot_ms, LawType type);

CommDataInstruction* requestServoInstructionData(int pin, int size, Point points[]);

void sendServoInstruction(const char* portSpecifier, CommDataInstruction* inst);

bool communicateData(const char* portSpecifier, char* str, const unsigned int size);

//void TestCommunication(std::string s);
#endif

#endif
