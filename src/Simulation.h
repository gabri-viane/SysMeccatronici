#pragma once
#ifndef SIMULATION_INCLUDED
#define SIMULATION_INCLUDED 1

#include "comms.h"

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

extern Serial_ Serial;
#endif // !SIMULATION_INCLUDED

