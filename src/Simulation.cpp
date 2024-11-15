#include "Simulation.h"
#include <Windows.h>

Serial_ Serial;

Serial_::Serial_()
{
	this->size = 0;
}

char Serial_::read()
{
	if (this->size > 0) {
		char first = this->buffer[0];
		memcpy((void*)&(this->buffer[0]), (void*)&(this->buffer[1]), MAX_MESSAGE_LENGTH - 1);
		this->buffer[MAX_MESSAGE_LENGTH - 1] = 0;
		this->size--;
		return first;
	}
	return -1;
}

bool Serial_::available() const
{
	return size > 0;
}

void Serial_::insertBuffer(char* buffer, int size)
{
	memcpy((void*)&(this->buffer[0]), (void*)&(buffer[0]), size);
	this->size = size;
	if (size < MAX_MESSAGE_LENGTH) {
		memset((void*)&(this->buffer[size]), 0, MAX_MESSAGE_LENGTH - size);
	}
}
