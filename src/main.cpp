#include "seven.h"
#include "spline.h"
#include "three.h"

#if USE_ARDUINO_H
#include <Arduino.h>
#include <Servo.h>

#include <vector>

#include "comms.h"
#include "communication.h"
#define SERVO_PIN 5
Servo s = Servo();

#if ENABLE_ARDUINO_COMM
char buffer[MAX_MESSAGE_LENGTH];
CommInstruction* ci = nullptr;
unsigned short tempo = 0;
#endif

void setup() {
    s.attach(SERVO_PIN);
    Serial.begin(9600);
#if ENABLE_ARDUINO_COMM
    ci = new CommInstruction(buffer);
#endif
    s.write(0);
}

#if WOKWI_SIMULATION

CondizioniIniziali tre_tratti_ci = {0, 0, 0};
Lambdas tre_tratti_lmb = {0, 3, 4, 3};
Instructions tre_tratti_inst = {tre_tratti_ci, tre_tratti_lmb, 1, 5, (unsigned char)180};

CondizioniIniziali sette_tratti_ci = {0, 0, 0};
Lambdas sette_tratti_lmb = {1, 2, 2, 2};
Instructions sette_tratti_inst = {sette_tratti_ci, sette_tratti_lmb, 1, 5, (unsigned char)180};

std::vector<double> tempi = {0, 1, 3, 5, 8, 15};
std::vector<double> punti = {0, 50, 140, 70, 120, 180};
#endif

void loop() {
#if ENABLE_ARDUINO_COMM
    if (parseInstruction(ci, &s, &tempo)) {
        // Qui ci va il codice quando viene ricevuto e completato con successo un comando
    }
#endif
#if WOKWI_SIMULATION
    // Test TreTratti
    s.write(tre_tratti_ci.angolo_inizio);
    delay(500);
    treTratti(tre_tratti_inst, &s);
    delay(5000);
    // Test SetteTratti
    s.write(sette_tratti_ci.angolo_inizio);
    delay(500);
    setteTratti(sette_tratti_inst, &s);
    delay(5000);
    // Test Spline
    s.write(punti[0]);
    delay(500);
    spline(tempi, punti, &s);
    delay(5000);
#endif
}

#else

#include <iostream>
#if MATLAB_COMPILE
#include "matlab.h"
#endif
#if SIMULATE_COMMUNICATION
#include <thread>

#include "Simulation.h"
#endif
#include "FakeArduino.h"
#include "Menu.h"
#include "communication.h"

#if SIMULATE_COMMUNICATION
std::atomic<bool> endOfMenu(false);

// Serve come funzione da eseguire in un thread separato su PC
// per simulare un arduino che riceve e legge i dati.
static void arduinoSimulator(Servo* s) {
    char msg[MAX_MESSAGE_LENGTH]{};
    CommInstruction* ci = new CommInstruction(msg);
    while (!endOfMenu) {
        parseInstruction(ci, s);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    delete ci;
}
#endif

int main() {
#if SIMULATE_COMMUNICATION
    TestCommunication();
    Servo s = Servo();
    s.attach(4);
    std::thread arduino(arduinoSimulator, &s);
#endif

    // Creo un nuovo menù
    Menu m = Menu();
    // Faccio partire il ciclo di gestione
    m.startMenu();
    // TestCommunication();

#if SIMULATE_COMMUNICATION
    endOfMenu = true;
    arduino.join();
#endif
    return 0;
}
#endif