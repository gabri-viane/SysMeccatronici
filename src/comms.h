#pragma once
//Rappresenta il delta di tempo in secondi utilizzato per il controllo/simulazione
#ifndef TIME_CONST_S
#define TIME_CONST_S 0.005
#endif
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
#define STARTING_STREAM_CHAR 01
#endif
//Rappresenta il carattere di chiusura messaggio (devono essere inviati in sequenza 2 caratteri)
#ifndef CLOSING_STREAM_CHAR
#define CLOSING_STREAM_CHAR 04
#endif


#define COMPILE_FOR_PC 1

//Serve per utilizzare le librerie di Arduino, altrimenti utilizza quelle simulate (FakeArduino.h, FakeServo.h)
//Per compilare questa parte mettere il valore a 1
#define USE_ARDUINO_H !COMPILE_FOR_PC

#define SIMULATE_COMMUNICATION 1 && COMPILE_FOR_PC

//Per compilare il codice che utilizza la parte matlab mettere il valore a 1
#define MATLAB_COMPILE COMPILE_FOR_PC && !SIMULATE_COMMUNICATION
//Per compilare il codice che utilizza la parte di Arduino mettere il valore a 1
#define ARDUINO_COMPILE !COMPILE_FOR_PC
//Se abilitato include tutta la gestione nella parte PC e nella parte arduino per la compilazione del
//codice di comunicazione e elaborazione dei comandi
#define ENABLE_ARDUINO_COMM 1

// Definisce il massimo numero di punti per costruire la spline cubica
#define MAX_LENGTH_POINTS 10

#if USE_ARDUINO_H
#include <Servo.h>
#else
#include "FakeServo.h"
#endif

#if COMPILE_FOR_PC
#define WIN32_LEAN_AND_MEAN
#endif

/*
angolo_inizio ; velocita_inizio ; accelerazione_inizio
*/
struct CondizioniIniziali {
    float angolo_inizio;
    float velocita_inizio;
    double accelerazione_inizio;
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
Costanti di tempo dei tratti
increm_lin ; cost_1 ; cost_2 ; cost_3 ;
*/
struct Lambdas {
    float increm_lin;
    float cost_1;
    float cost_2;
    float cost_3;
};

/*
Istruzioni per gestire la sette tratti
*/
struct Instructions {
    // Condizioni iniziali da cui partire
    CondizioniIniziali ci;
    // Tempo totale richiesto per eseguire la legge di moto
    unsigned long long int tempo_tot_ms;
    // I coefficienti di tempo dei tratti
    Lambdas lambdas;
    // Lo spazio da percorrere oppure posizione da raggiungere se la legge è COMANDO_DIRETTO
    double delta_angolo;
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

struct Point {
    double time;
    double position;
};

struct SplineInstructionData {
    Point points[MAX_LENGTH_POINTS];
    unsigned char size;
};


enum LawType {
	TRE_TRATTI = 3, SETTE_TRATTI = 7, SPLINE = 6, COMANDO_DIRETTO = 8
};

typedef void (*LawFunction)(Instructions inst, Servo s);