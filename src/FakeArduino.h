#pragma once
#ifndef ARDUINO_FAKE
#define ARDUINO_FAKE 1

/**
* Simula un ritardo in modo da non dover importare la libreria di arduino
* se il progetto viene compilato per Windows
* 
* @param Tempo in millisecondi da aspettare (teorico, non viene messo in pausa il PC)
*/
void delay(double);

#endif