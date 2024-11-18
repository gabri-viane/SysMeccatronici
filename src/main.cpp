#include "seven.h"
#include "spline.h"
#include "three.h"

#if USE_ARDUINO_H
#include <Arduino.h>
#include <Servo.h>

#include <vector>

#include "communication.h"
#define SERVO_PIN 5
Servo s = Servo();

char buffer[MAX_MESSAGE_LENGTH];
CommInstruction* ci = nullptr;
unsigned short tempo = 0;

void setup() {
    s.attach(SERVO_PIN);
    Serial.begin(9600);
    ci = new CommInstruction(buffer);
    s.write(0);
}

void loop() {
#if ENABLE_ARDUINO_COMM
    if (parseInstruction(ci, &s, &tempo)) {
        //Qui ci va il codice quando viene ricevuto e completato con successo un comando
    }
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

//Serve come funzione da eseguire in un thread separato su PC
//per simulare un arduino che riceve e legge i dati.
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