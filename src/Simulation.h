#pragma once
#ifndef SIMULATION_INCLUDED
#define SIMULATION_INCLUDED 1

#include "comms.h"

#if SIMULATE_COMMUNICATION
/*
* Serve per simulare la comunicazione quando non si sta utilizzando arduino.
* Viene usata allo stesso modo della seriale di arduino.
*/
class Serial_ {
private:
	char buffer[MAX_MESSAGE_LENGTH]{};
	int size;
public:
	Serial_();
	char read();
	bool available() const;
	void insertBuffer(char* buffer,int size);
};
//Seriale che simula quella di arduino.
//Viene poi definita nel file .cpp
extern Serial_ Serial;
#endif
#endif // !SIMULATION_INCLUDED

