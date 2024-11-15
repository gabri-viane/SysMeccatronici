#include "matlab.h"
#include "comms.h"

#if MATLAB_COMPILE
#include <string>
#include <fstream>

//Inizializzo l'istanza di matlab a puntatore nullo
MatLAB* matlab = nullptr;

MatLAB* createInstance(const std::string& file_name) {
	//Creo una nuova istanza e l'assegno alla variabile globale
	matlab = new MatLAB(file_name);
	//Restituisco la variabile globale dell'istanza creata
	return matlab;
}

MatLAB* getMatLAB() {
	//Restituisce il puntatore alla variabile globale
	return matlab;
}

MatLAB::MatLAB(const std::string& file_name) {
	//Creo un nuovo Output File Stream per scrivere nel file
	this->m_outFile = new std::ofstream();
	//Apro lo stream in modalità scrittura
	this->m_outFile->open(file_name, std::ios::out);
}

MatLAB::~MatLAB() {
	//Alla distruzione dell'istanza controllo se ho lo stream esiste ed è aperto
	if (this->m_outFile && this->m_outFile->is_open()) {
		//Se lo stream del file è ancora aperto lo chiudo
		this->m_outFile->close();
	}
	//Elimino il puntatore dello stream
	delete this->m_outFile;
}

void MatLAB::addComment(const char* comment) {
	//Aggiungo una nuova riga commentata nel file matlab
	*this->m_outFile << "%" << comment << "\n";
}

void MatLAB::addEndDataStep()
{
	//Segno che ho finito uno step (ovvero un tratto della legge di moto)
	m_step[m_current_step] = m_current_data - 1;
	//Vado al prossimo step
	m_current_step++;
}

void MatLAB::addComment(const std::string comment) {
	//Aggiungo una nuova riga commentata nel file matlab
	*this->m_outFile << "%" << comment << "\n";
}

void MatLAB::appendData(double time, double position) {
	//Controllo di non aver superato i limiti massimi per l'array di dati salvabili
	if (m_current_data < LENGTH_VALS) {
		//Salvo il tempo
		this->m_data[0][m_current_data] = time;
		//Salvo la posizione
		this->m_data[1][m_current_data] = position;
		//Incremento la posizione per il prossimo salvataggio
		m_current_data++;
	}
}

//Questa funzione va chiamata prima di chiamare MatLAB::appendData(*)
void MatLAB::appendAccData(double time, double acceleration)
{
	//Controllo di non aver superato i limiti massimi per l'array di dati salvabili
	if (m_current_data < LENGTH_VALS) {
		//Segno che ho dei dati per l'accelerazione: in questo modo genererò lo script per il grafico dell'accelerazione
		this->m_has_acceleration_data = true;
		//Salvo il tempo
		this->m_acc[0][m_current_data] = time;
		//Salvo l'accelerazione
		this->m_acc[1][m_current_data] = acceleration;
	}
}

std::ofstream& MatLAB::getStream() {
	//Restituisce il puntatore all'istanza dello stream
	return *this->m_outFile;
}

void MatLAB::writeData()
{
	//Imposto quando ho raggiunto l'ultimo valore utile alla massima dimensione possibile dell'array
	int max_reached_at = LENGTH_VALS;
	//Scrivo un commento nel file matlab che genero
	//e creo l'array del tempo
	*this->m_outFile << "%Data from arduino\nt=[";
	//Eseguo per tutti i possibili dati salvati (possibilmente anche più di quelli che ho generato)
	for (int i = 0; i < LENGTH_VALS; i++) {
		//Controllo che il tempo salvato non sia 0 (ad esclusione della prima iterazione (i==0))
		if (i == 0 || (m_data[0][i] != 0)) {
			//Scrivo il tempo nel file
			*this->m_outFile << m_data[0][i] << " ";
		}
		// Se vedo che il tempo è 0 vuol dire che sono arrivato alla fine dei dati salvati
		else {
			//Segno quanti valori ho salvato in totale
			max_reached_at = i;
			break;
		}
	}
	//Scrivo nel file la nuova variabile "spazio"
	*this->m_outFile << "];\nspazio=[";
	for (int i = 0; i < max_reached_at; i++) {
		//Inserisco tutti i valori nell'array "spazio"
		*this->m_outFile << m_data[1][i] << " ";
	}
	//Se ho salvato anche i dati dell'accelerazione dalla simulazione scrivo anche quelli nel file matlab
	if (this->m_has_acceleration_data) {
		*this->m_outFile << "];\naccel=[";
		for (int i = 0; i < max_reached_at; i++) {
			//Salvo i dati dell'accelerazione
			*this->m_outFile << m_acc[1][i] << " ";
		}
	}
	//Chiudo nel file l'ultimo array generato
	*this->m_outFile << "];\n";
}

void MatLAB::flushData()
{
	//Scrivo tutti i dati nel file e inserisco gli step
	*this->m_outFile << "%Steps:\nsteps=[";
	for (int i = 0; i < LENGTH_STEPS; i++) {
		*this->m_outFile << m_data[0][m_step[i]] << " " << m_data[1][m_step[i]];
		if (i < (LENGTH_STEPS - 1)) {
			*this->m_outFile << ";";
		}
	}
	*this->m_outFile << "];\n";
	//Viene eseguito il flush dei dati nel file
	this->m_outFile->flush();
	//Sono reimpostati i dati
	this->m_current_data = 0;
	this->m_current_step = 0;
	//Viene calcolata la memoria da resettare
	size_t t = LENGTH_VALS * sizeof(double) * 2;
	//Vengono reimpostati gli array
	memset(this->m_acc, 0, t);
	memset(this->m_data, 0, t);
	memset(this->m_step, 0, LENGTH_STEPS * sizeof(int));
}

void MatLAB::addPlotting(const std::string title)
{
	//Crea una nuova figura
	*this->m_outFile << "%plotting\nfigure();\n";
	//Se sono stati aggiunti i dati dell'accelerazione viene gestita
	//la figura con i subplot.
	if (this->m_has_acceleration_data) {
		*this->m_outFile << "subplot(2,1,1);\n";
	}
	//Vengono scritte le informazioni e caricati i dati nel plot (o subplot) delle posizioni
	*this->m_outFile << "plot(t, spazio); \nhold on; \nplot(steps(:, 1), steps(:, 2), \"X\");\nlegend('Spazio[°]','Fine Tratto');\ntitle('Spazio "
		<< title << "');" << "\nxlabel('tempo[s]'); \nylabel('angolo[°]'); \nhold off; \n";
	//Se sono presenti i dati dell'accelerazione viene eseguito il subplot dei relativi dati.
	if (this->m_has_acceleration_data) {
		*this->m_outFile << "subplot(2, 1, 2); \nplot(t, accel); \nlegend('Accelerazione'); \ntitle('Accelerazione "
			<< title << "')\nxlabel('tempo[s]'); \nylabel('accel.[°/s^2]'); ";
	}
	//Viene eseguito il flush dei dati
	this->m_outFile->flush();
}
#endif