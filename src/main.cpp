#include "seven.h"
#include "spline.h"
#include "three.h"

#if USE_ARDUINO_H
#include <Arduino.h>
#include <Servo.h>

#include <vector>

#include "communication.h"
#define SERVO_PIN 5
Servo s;


char buffer[MAX_MESSAGE_LENGTH];
CommInstruction* ci = nullptr;

void setup() {
	s.attach(SERVO_PIN);
	Serial.begin(9600);
	ci = new CommInstruction(buffer);
}

void loop() {
#if ENABLE_ARDUINO_COMM
	if (Serial.available() > 0) {
		parseInstruction(ci, s);
	}else {
		delay(50);
	}
#endif
}

#else

#include <iostream>
#if MATLAB_COMPILE
#include "matlab.h"
#endif
#if SIMULATE_COMMUNICATION
#include "communication.h"
#include "Simulation.h"
#include <thread>
#endif
#include "FakeArduino.h"
#include "Menu.h"


#if SIMULATE_COMMUNICATION
std::atomic<bool> endOfMenu(false);

static void arduinoSimulator(Servo s) {
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
#ifdef SIMULATE_COMMUNICATION
	//TestCommunication("Prova");
	Servo s;
	s.attach(4);
	std::thread arduino(arduinoSimulator, s);
#endif

	// Creo un nuovo menù
	Menu m = Menu();
	// Faccio partire il ciclo di gestione
	m.startMenu();

#ifdef SIMULATE_COMMUNICATION
	endOfMenu = true;
	arduino.join();
#endif
	return 0;
}
#endif