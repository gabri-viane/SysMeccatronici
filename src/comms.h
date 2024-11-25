#pragma once

/************************************
*       IMPOSTAZIONI MODIFICABILI   *
*************************************/

//Se impostato ad 1 abilita la compilazione del progetto per PC, altrimenti per Arduino
#define COMPILE_FOR_PC 1

//Permette di simulare la comunicazione con Arduino, ovvero se arduino non è connesso al computer
//ma si vuole ugualmente testare il programma e l'invio dei dati.
//Modificare il valore 0 o 1 se si vuole disabilitare o abilitare questa impostazione.
#define ENABLE_COMM_SIMULATION 0

//Se abilitato include tutta la gestione nella parte PC e nella parte arduino per la compilazione del
//codice di comunicazione e elaborazione dei comandi
#define ENABLE_ARDUINO_COMM 0

//Per simulare il funzionamento con il plugin wokwi impostare ad 1 questa define
//inoltre il valore di COMPILE_FOR_PC deve essere 0
#define ENABLE_WOKWI_SIMULATION 0

// Definisce il massimo numero di punti per costruire la spline cubica
#define MAX_LENGTH_POINTS 10

//Rappresenta il delta di tempo in millisecondi utilizzato per il controllo/simulazione
#ifndef TIME_CONST_MS
#define TIME_CONST_MS 5
#endif
//Rappresenta la massima dimensione del buffer di ricezione e di invio in bytes
#ifndef MAX_BUFFER_SIZE
#define MAX_BUFFER_SIZE 64
#endif
//Rappresenta la massima dimensione del messaggio (4 bytes sono privati: 2 di inizio e 2 di termine del messaggio)
#ifndef MAX_MESSAGE_LENGTH
#define MAX_MESSAGE_LENGTH 256
#endif
//Rappresenta il carattere di chiusura messaggio (devono essere inviati in sequenza 2 caratteri)
#ifndef STARTING_STREAM_CHAR
#define STARTING_STREAM_CHAR (char)1
#endif
//Rappresenta il carattere di chiusura messaggio (devono essere inviati in sequenza 2 caratteri)
#ifndef CLOSING_STREAM_CHAR
#define CLOSING_STREAM_CHAR (char)4
#endif

/*****************************************
*     FINE IMPOSTAZIONI MODIFICABILI     *
******************************************/


//Serve per utilizzare le librerie di Arduino, altrimenti utilizza quelle simulate (FakeArduino.h, FakeServo.h)
//Per compilare questa parte mettere il valore a 1
#define USE_ARDUINO_H !COMPILE_FOR_PC

#define SIMULATE_COMMUNICATION (ENABLE_COMM_SIMULATION && COMPILE_FOR_PC)

//Per compilare il codice che utilizza la parte matlab mettere il valore a 1
#define MATLAB_COMPILE (COMPILE_FOR_PC && !SIMULATE_COMMUNICATION)
//Per compilare il codice che utilizza la parte di Arduino mettere il valore a 1
#define ARDUINO_COMPILE !COMPILE_FOR_PC

#define WOKWI_SIMULATION ENABLE_WOKWI_SIMULATION && !COMPILE_FOR_PC

//Rappresenta il delta di tempo in secondi utilizzato per il controllo/simulazione
#ifndef TIME_CONST_S
#define TIME_CONST_S TIME_CONST_MS/1000.0
#endif

#if USE_ARDUINO_H
#include <Servo.h>
#else
#include "FakeServo.h"
#endif

//Se compilo per PC disabilito gli include non necessari delle API Windows
#if COMPILE_FOR_PC
#define WIN32_LEAN_AND_MEAN
#endif

/*****************************************
* INIZIO DEFINIZIONE DI STRUTTURE COMUNI *
******************************************/

/*
angolo_inizio ; velocita_inizio ; accelerazione_inizio
In questo caso come in altri sono stati sostituiti i valori float con
char (ovvero interi di 8bit) in modo da risparmiare spazio poiché i Servo
motori generalmente in dotazione non accettano frazioni di angolo come 
comando.
*/
struct CondizioniIniziali {
    float angolo_inizio; //float
    float velocita_inizio; //float
    float accelerazione_inizio; //double
};

/*
. -----
./     \
/       \______
.              \        /
.               \______/

Accelerazioni dei tratti:
lin  1 lin  2  lin  3   lin
*/

/*
(Riferirsi alla rappresentazione qua sopra per capire cosa rappresentano
i vari valori)
Costanti di tempo dei tratti
increm_lin ; cost_1 ; cost_2 ; cost_3 ;

I coefficienti vengono successivamente divisi per 10:
un coefficiente di 0.2 deve essere salvato come 2.

Questo per permettere l'invio in modo più semplice ad arduino.
*/
struct Lambdas {
    float increm_lin; //float
    float cost_1; //float
    float cost_2; //float
    float cost_3; //float
};

/*
Istruzioni per gestire la sette tratti
*/
struct Instructions {
    // Condizioni iniziali da cui partire
    CondizioniIniziali ci;
    // I coefficienti di tempo dei tratti
    Lambdas lambdas;
    // Lo spazio da percorrere oppure posizione da raggiungere se la legge è COMANDO_DIRETTO
    long delta_angolo;
    // Tempo totale richiesto per eseguire la legge di moto
    unsigned int tempo_tot_ms;
};

/*
Informazioni da cui un tratto parte
*/
struct InfoTratto {
    // Spazio d'inizio
    double spazio0;
    // Velocità d'inizio
    double veloctia0;
    // Accelerazione d'inizio
    double accelerazione0;
    // Tempo totale di calcolo a cui inizia il tratto
    double tempo0;
    // Tempo che deve durare il tratto corrente
    double delta_t;
};

//Struttura utilizzata per rappresentare un punto
struct Point {
    double time;
    double position;
};

//Rappresenta la struttura che può essere inviata ad Arduino tramite
//la comunicazione. La struttura viene serializzata e inviata come sequenza
//di byte.
struct SplineInstructionData {
    //Array che contiene i punti per ricostruire la spline cubica
    Point points[MAX_LENGTH_POINTS];
    //Inidica quanti punti sono presenti nell'array "points"
    unsigned char size;
};

//Enum che rappresenta le tipologie di comandi utilizzabili su servo motore
//La funzione COMANDO_DIRETTO è semplice e non è stata implementata ma viene
//già supportata dal sistema di comunicazione dei dati
enum LawType {
	TRE_TRATTI = 3, SETTE_TRATTI = 7, SPLINE = 6, COMANDO_DIRETTO = 8
};

//Non utilizzata: serve per automatizzare delle parti di esecuzione del codice
//di comunicazione nel caso si volesse espandere questo progetto
typedef void (*LawFunction)(Instructions inst, Servo s);