#include "comms.h"

#if !USE_ARDUINO_H
#include "FakeArduino.h"
#include "FakeServo.h"

#include <iostream>

void delay(double time) {
	std::cout << "Waiting for: " << time << std::endl;
}

void Servo::attach(int i) {
	std::cout << "Attaching to " << i << std::endl;
}

void Servo::write(double i) {
	std::cout << " " << i;
}

#endif