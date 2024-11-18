#pragma once
#ifndef COMMUNICATION_INCLUDED
#define COMMUNICATION_INCLUDED 1

#include <vector>

#include "comms.h"

/*
* Rappresentea l'istruzione da inviare ad Arduino byte per byte. Una volta che viene ricevuta
* dalla scheda viene deserializzata nuovamente in questa struttura.
* Rappresenta in modo univoco un'unica legge di moto alla volta.
* Gestisce la distruzione delle istanze create per "Dati_Tre_Sette_tratti" e "Dati_Spline"
*/
struct ServoWriteInst {
    //il puntatore alla struct che può contenere i dati di una legge di moto Sette tratti o Tre tratti
    Instructions* Dati_Tre_Sette_Tratti;
    //il puntatore alla struct che contiene i dati per costruire una spline
    SplineInstructionData* Dati_Spline;

    //Da notare che solo una delle due precedenti sarà != nullptr

    //Il pin del servomotore a cui applicare la legge di moto
    char servoPin;
    //Rappresenta il valore che viene imposto al servo motore se la legge è "COMANDO_DIRETTO"
    unsigned short directDrive;
    //Legga da utilizzare per comandare il servo motore: in base a quella scelta
    //vengono istanziati e gestiti correttamente i puntatiri di "Dati_Tre_Sette_tratti" e "Dati_Spline"
    LawType law;

    //Inizializza una nuova struct con la legge di moto specificata
    ServoWriteInst(char Pin, LawType lt);
    //Distruggi tutte le istanze create sulla heap
    ~ServoWriteInst();
};

/*
* Rappresenta la tipologia di messaggio inviabile ad Ardunio.
*/
enum CommRequestType {
    //Istruzione nulla : non fare nulla
    NULL_REQ = 0,
    //Istruzione di scrittura servomotore : il PC sta inviando dati serializzati di una struct ServoWriteInst
    SERVO_WRITE = 1,
    //Istruzione di lettura servomotore : il PC chiede ad arduino a che angolo è posizionato un certo servo
    SERVO_READ = 2,
    //Istruzione di lettura generica di un sensore
    SENSOR_READ = 3
};

/*
* Rappresenta la struttura di dati che viene inviata e ricevuta. Per ora, nell'ambito di questo
* progetto viene utilizzato solamente la scrittura a servo motore (CommRequestType::SERVO_WRITE)
*/
struct CommDataInstruction {
    //Puntatore da instanziare a mano sulla heap
    ServoWriteInst* swi;
    //Costruttore della struct
    CommDataInstruction();
    //Distruttore: distrugge in modo autonomo l'istanza puntata da "swi": VERIFICARE DI AVER INSTANZIATO 
    //CORRETTAMENTE E DI NON UTILIZZARE IL PUNTATORE DOPO AVER DISTRUTTI IL DATO
    ~CommDataInstruction();
};

// Macchina a stati per la lettura del messaggio in arrivo
#define STATUS_VOID 0             // Indica che se si riceve il carattere  qualsiasi si passa a STATUS_START_READING
#define STATUS_START_READING 1    // Indica che se si riceve un carattere qualsiasi si passa a STATUS_START_READING
#define STATUS_READING 2          // Indica che se il prossimo carattere è CLOSING_STREAM_CHAR si passa a STATUS_SHOULD_COMPLETE
#define STATUS_SHOULD_COMPLETE 3  // Indica che se il prossimo carattere è CLOSING_STREAM_CHAR si passa a STATUS_COMPLETE
#define STATUS_COMPLETED 4        // In questo stato il messaggio è stato completamente ricevuto e pronto per essere interpretato

/*
* Struttura che rappresenta gestisce i dati in arrivo su Arduino.
*/
struct CommInstruction {
    //Status corrente della macchina a stati
    char status;
    //Puntatore al buffer dove salvare il messaggio
    char* message;
    //Dimensione del messaggio ricevuta e salvata nel buffer
    unsigned short sizeReceived;
    //Dimensione da ricevere del messaggio
    unsigned char size;
    //Dati dell'istruzione richiesta dal PC
    CommDataInstruction inst;
    //Istruzione richiesta dal PC
    CommRequestType commType;
    //Inizializza la struttura con un puntatore al buffer dove salvare il messaggio
    //Ogni volta che il messaggio è stato completamente ricevuto il buffer viene svuotato
    CommInstruction(char* buffer);
    //Distruttore
    ~CommInstruction();
};
// Parte arduino oppure simulazione
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
bool parseInstruction(CommInstruction* inst, Servo* s, unsigned short* tempoRicevuto);

/*
* Serve per eseguire una conversione dei dati in entrata ad un'istruzione per 
* eseguire una legge di moto spline.
* 
* @warn L'implementazione non è stata testata.
*/
void executeSplineConversion(CommInstruction* inst, Servo s);

/*
* Gestisce la conversione dei dati ricevuti nell'effettiva legge di moto. Ovvero copia i byte rievuti
* in una struttura corretta che rappresenta la legge di moto.
* 
* @param CommInstruction* ci Il puntatore alla zona di memoria dove c'è l'istruzione da copiare nella
*               struct
* @return ServoWriteInst* Crea una nuova istanza CHE DOVRA' ESSERE GESTITA E DISTRUTTA alla struttura
*               della legge di moto copiata.
*/
ServoWriteInst* parseServoWrite(CommInstruction* ci);
#endif
// Parte solo PC
#if !USE_ARDUINO_H
#include <string>

/*
* Partendo dai dati iniziali di una legge di moto Tre tratti o Sette tratti 
* crea un l'istruzione da inviare tramite la funzione "sendServoInstruction".
* 
* @param int pin Il pin del servo da controllare
* @param CondizioniIniziali ci La struct contenente tutti i dati delle condizioni iniziali della
*               legge di moto
* @param double delta_angolo Il delta dell'angolo da percorrere (in gradi, non radianti)
* @param Lambdas lmbds I lambda dei tratti della legge di moto scelta
* @param unsigned long long tempo_tot_ms Il tempo di esecuzione della legge di moto (per percorrere il delta_angolo)
* @param LawType type Che legge di moto rappresentano i dati inseriti
* 
* @return CommDataInstruction* Una nuova struttura allocata sulla heap che DEVE ESSERE GESTITA (in modo automatico
se passato il valore alla funzione "sendServoInstruction")
*/
CommDataInstruction* requestServoInstructionData(int pin, CondizioniIniziali ci, double delta_angolo, Lambdas lmbds, unsigned long long tempo_tot_ms, LawType type);
/*
* Partendo dai dati iniziali di una legge di moto Spline
* crea un l'istruzione da inviare tramite la funzione "sendServoInstruction".
*
* @param int pin Il pin del servo da controllare
* @param int size Il numero di punti che sono presenti nell'array "points"
* @param Point points[] Array di punti da utilizzare per costruire la spline.
* 
* @return CommDataInstruction* Una nuova struttura allocata sulla heap che DEVE ESSERE GESTITA (in modo automatico
se passato il valore alla funzione "sendServoInstruction")
*/
CommDataInstruction* requestServoInstructionData(int pin, int size, Point points[]);

/*
* Invia l'istruzione alla porta COM specificata. L'istruzione dovrebbe essere creata con la funzione
* "requestServoInstructionData" per evitare problemi di gestione dei dati. Questa funzione si occupa
* solamente di serializzare come sequenza di byte i dati e inviarli con la funzione "communicateData".
* 
* @param const char* portSpecifier La porta da utilizzare per la comunicazione seriale (es. "COM1")
* @param CommDataInstruction* inst L'istruzione che viene gestita e inviata.
*/
void sendServoInstruction(const char* portSpecifier, CommDataInstruction* inst);

/*
* Invia i dati alla porta COM specificata. Necessita di un array di byte e la dimensione da inviare.
* Utilizza un protocollo ad-hoc che specifica 2 byte iniziali, un byte che rappresenta il numero di
* byte che si stanno inviando e il messaggio conculuso con 2 byte di chiusura.
* 
* Se il messaggio è troppo lungo (supera il valore MAX_BUFFER_SIZE) viene suddiviso in più parti per
* permettere al buffer della seriale di Arduino di ricevere i dati senza perderli. (a causa della 
* differente velocità di comunicazione e ricezione).
* 
* @param const char* portSpecifier La porta da utilizzare per la comunicazione seriale (es. "COM1")
* @param char* str Buffer da cui leggere i dati e inviarli.
* @param const unsigned int size Dimensione del buffer da inviare (al massimo MAX_MESSAGE_LENGTH)
*/
bool communicateData(const char* portSpecifier, char* str, const unsigned int size);

/*
* Utilizzata solo per testare la comunicazione
*/
void TestCommunication();
#endif

#endif
