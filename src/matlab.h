#pragma once
#include "comms.h"
#ifndef MATLAB_INCLUDED 
#define MATLAB_INCLUDED 1

#if MATLAB_COMPILE

#include <string>
#include <fstream>

//Definisce il massimo numero di punti che si possono simulare
#define LENGTH_VALS 10000
//Definisce il massimo numero di tratti di cui una legge di moto può essere composta
#define LENGTH_STEPS 7

/*
* Classe che, se il progetto viene compilato per Windows, permette di
* generare un file script di MATLAB che una volta eseguito mostra il risultato
* della simulazione in grafici.
*/
class MatLAB {

private:
	int m_current_step = 0;
	int m_step[LENGTH_STEPS]{ 0 };

	bool m_has_acceleration_data = false;

	int m_current_data = 0;
	double m_data[2][LENGTH_VALS]{ {0},{0} };
	double m_acc[2][LENGTH_VALS]{ {0},{0} };
	std::ofstream* m_outFile;

public:
	/**
	* Il distruttore della classe si occupa di chiudere lo stream se risultasse
	* ancora aperto.
	*/
	~MatLAB();

	/**
	* Crea un nuovo oggetto MATLAB.
	* 
	* @param file_name il nome del file da creare/in cui scrivere
	*/
	MatLAB(const std::string& file_name);

	/**
	* Aggiunge un nuovo punto tempo-posizione al file script.
	* 
	* @param time il tempo (coordinata x)
	* @param position la posizione (coordinata y)
	*/
	void appendData(double time, double position);
	/**
	* Aggiunge un nuovo punto tempo-accelerazione al file script.
	* 
	* @param time il tempo (coordinata x)
	* @param acceleration l'accelerazione (coordinata y)
	*/
	void appendAccData(double time, double acceleration);
	/**
	* Aggiunge un nuovo commento nel file script al punto 
	* corrente in cui la scrittura è arrivata.
	* 
	* @param comment la stringa da inserire come commento
	*/
	void addComment(const std::string comment);
	/**
	* Aggiunge un nuovo commento nel file script al punto
	* corrente in cui la scrittura è arrivata.
	* 
	* @param comment la stringa da inserire come commento
	*/
	void addComment(const char* comment);
	/**
	* Conclude il tratto della legge di moto corrente: in questo
	* modo nei grafici generati viene segnato il punto con una
	* X rossa. La conclusione di un tratto implica l'inizio del
	* successivo.
	*/
	void addEndDataStep();
	/**
	* Restituisce l'Output File Stream in cui l'istanza sta scrivendo
	* (ovvero il file script)
	* 
	* @return Restituisce una reference allo stream corrente.
	*/
	std::ofstream& getStream();
	/**
	* Scrive nel file tutti gli array di dati raccolti: tempo-posizione
	* ed eventualmente anche tempo-accelerazione.
	*/
	void writeData();
	/**
	* Salva i tutti i dati nel file eseguendo un flush del buffer e inoltre
	* Scrive anche l'array degli step (tratti) della legge di moto.
	*/
	void flushData();
	/**
	* Aggiunge al file script il codice MATLAB per generare i vari grafici.
	* 
	* @param title Il titolo da inserire nel grafico generato.
	*/
	void addPlotting(const std::string title);
};

/**
* Crea una nuova istanza globale della classe MatLAB.
* 
* @param file_name Il nome del file in cui lo stream andrà a scrivere i comandi MATLAB
* 
* @return Il puntatore alla variabile globale appena creata.
*/
MatLAB* createInstance(const std::string& file_name);

/**
* Restituisce il puntatore alla variabile globale che punta all'istanza di MatLAB 
* precedentemente associata con la function "createInstance(const std::string&)".
* 
* @return Il puntatore alla variabile globale.
*/
MatLAB* getMatLAB();
#endif
#endif