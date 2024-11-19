#include "seven.h"

#include "comms.h"
#if USE_ARDUINO_H
#include <Arduino.h>
#include <Servo.h>
#else
#include "FakeArduino.h"
#include "FakeServo.h"
#endif
#if MATLAB_COMPILE
#include "matlab.h"
#endif

void setteTratti(Instructions inst, Servo s) {
	double tempo_s = inst.tempo_tot_ms / 1000.0;// Converto da ms a s il tempo totale
	double tempi[] = {
		tempo_s * inst.lambdas.increm_lin / 10.0,	// tempo tratti accelerazione lineare
		tempo_s * inst.lambdas.cost_1 / 10.0,		// tempo tratto m_acc. costante positiva
		tempo_s * inst.lambdas.cost_2 / 10.0,		// tempo tratto m_acc. costante nulla
		tempo_s * inst.lambdas.cost_3 / 10.0 };	// tempo tratto m_acc. costante negativa

	// Tempo totale dell'insieme dei tratti 1-2-3
	double Ta = 2 * tempi[0] + tempi[1];
	// Calcolo il jerk massimo che ho per arrivare all'angolo richiesto
	double jerk_max = 1.0 * inst.segno * inst.delta_angolo / ((tempo_s - Ta) * (Ta - tempi[0]) * tempi[0]);

	//Condizioni iniziali del primo tratto
	InfoTratto inext = {
		inst.ci.angolo_inizio * 1.0,
		inst.ci.velocita_inizio * 1.0,
		inst.ci.accelerazione_inizio * 1.0,
		0, tempi[0] };

	//Ad ogni tratto aggiorno le condizioni iniziali per il prossimo tratto
	inext = tratto1(inext, jerk_max, s);
	// Aggiorno il tempo per il prossimo trato
	inext.delta_t = tempi[1];
	inext = tratto2(inext, jerk_max, s);
	inext.delta_t = tempi[0];
	inext = tratto3(inext, jerk_max, s);
	inext.delta_t = tempi[2];
	inext = tratto4(inext, jerk_max, s);
	inext.delta_t = tempi[0];
	inext = tratto5(inext, jerk_max, s);
	inext.delta_t = tempi[3];
	inext = tratto6(inext, jerk_max, s);
	inext.delta_t = tempi[0];
	inext = tratto7(inext, jerk_max, s);
}

InfoTratto tratto1(InfoTratto tratto0, double jerk, Servo s) {
#if MATLAB_COMPILE
	auto* ML = getMatLAB();
#endif
	double s1{ 0 }, v1{ 0 }, a1{ 0 }, t{ 0 };
	// variabili di supporto
	double jt{ 0 }, jt2{ 0 }, at{ 0 };
	// current time: a differenza della variabile "t" misura il tempo dall'inizio del tratto 1
	double ct{ tratto0.tempo0 };
	for (; t < tratto0.delta_t; t += TIME_CONST_S, ct += TIME_CONST_S) {
		jt = jerk * t;                    // jerk*t
		jt2 = 0.5 * jt * t;               // 1/2 * jerk * t^2
		at = tratto0.accelerazione0 * t;  // a0 * t
		a1 = jt + tratto0.accelerazione0;
		v1 = jt2 + at + tratto0.veloctia0;
		s1 = jt2 * t / 3 + at * t * 0.5 + tratto0.veloctia0 * t + tratto0.spazio0;
#if ARDUINO_COMPILE
		s.write(s1);
		delay(TIME_CONST_MS);
#endif
#if MATLAB_COMPILE
		ML->appendAccData(t, a1);
		ML->appendData(t, s1);
#endif
	}
#if MATLAB_COMPILE
	ML->addEndDataStep();
#endif
	return { s1, v1, a1, ct, 0 };
}

InfoTratto tratto2(InfoTratto tratto1, double jerk, Servo s) {
#if MATLAB_COMPILE
	auto* ML = getMatLAB();
#endif
	double s2{ 0 }, v2{ 0 }, t{ 0 };
	// l'accelerazione in questo tratto � costante ed � uguale a quella raggiunta a fine tratto 1
	const double a2{ tratto1.accelerazione0 };
	// variabili di supporto
	double at{ 0 };
	// current time: a differenza della variabile "t" misura il tempo dall'inizio del tratto 1
	double ct{ tratto1.tempo0 };
	for (; t < tratto1.delta_t; t += TIME_CONST_S, ct += TIME_CONST_S) {
		at = a2 * t;
		v2 = at + tratto1.veloctia0;
		s2 = at * t * 0.5 + tratto1.veloctia0 * t + tratto1.spazio0;
#if ARDUINO_COMPILE
		s.write(s2);
		delay(TIME_CONST_MS);
#endif
#if MATLAB_COMPILE
		ML->appendAccData(ct, a2);
		ML->appendData(ct, s2);
#endif
	}
#if MATLAB_COMPILE
	ML->addEndDataStep();
#endif
	return { s2, v2, a2, ct, 0 };
}


InfoTratto tratto3(InfoTratto tratto2, double jerk, Servo s) {
#if MATLAB_COMPILE
	auto* ML = getMatLAB();
#endif
	double s3{ 0 }, v3{ 0 }, a3{ 0 }, t{ 0 };
	// variabili di supporto
	double jt{ 0 }, jt2{ 0 }, at{ 0 };
	// current time: a differenza della variabile "t" misura il tempo dall'inizio del tratto 1
	double ct{ tratto2.tempo0 };
	for (; t < tratto2.delta_t; t += TIME_CONST_S, ct += TIME_CONST_S) {
		jt = -jerk * t;                   // -jerk*t
		jt2 = 0.5 * jt * t;               // 1/2 * jerk * t^2
		at = tratto2.accelerazione0 * t;  // a0 * t
		a3 = jt + tratto2.accelerazione0;
		v3 = jt2 + at + tratto2.veloctia0;
		s3 = jt2 * t / 3 + at * t * 0.5 + tratto2.veloctia0 * t + tratto2.spazio0;
#if ARDUINO_COMPILE
		s.write(s3);
		delay(TIME_CONST_MS);
#endif
#if MATLAB_COMPILE
		ML->appendAccData(ct, a3);
		ML->appendData(ct, s3);
#endif
	}
#if MATLAB_COMPILE
	ML->addEndDataStep();
#endif
	return { s3, v3, a3, ct, 0 };
}


InfoTratto tratto4(InfoTratto tratto3, double jerk, Servo s) {
#if MATLAB_COMPILE
	auto* ML = getMatLAB();
#endif
	double s4{ 0 }, t{ 0 };
	// velocit� e accelerazione in questo tratto sono costanti
	const double v4{ tratto3.veloctia0 }, a4{ 0 };
	// current time: a differenza della variabile "t" misura il tempo dall'inizio del tratto 1
	double ct{ tratto3.tempo0 };
	for (; t < tratto3.delta_t; t += TIME_CONST_S, ct += TIME_CONST_S) {
		s4 = v4 * t + tratto3.spazio0;
#if ARDUINO_COMPILE
		s.write(s4);
		delay(TIME_CONST_MS);
#endif
#if MATLAB_COMPILE
		ML->appendAccData(ct, a4);
		ML->appendData(ct, s4);
#endif
	}
#if MATLAB_COMPILE
	ML->addEndDataStep();
#endif
	return { s4, v4, a4, ct, 0 };
}


InfoTratto tratto5(InfoTratto tratto4, double jerk, Servo s) {
#if MATLAB_COMPILE
	auto* ML = getMatLAB();
#endif
	double s5{ 0 }, v5{ 0 }, a5{ 0 }, t{ 0 };
	// variabili di supporto
	double jt{ 0 }, jt2{ 0 }, at{ 0 };
	// current time: a differenza della variabile "t" misura il tempo dall'inizio del tratto 1
	double ct{ tratto4.tempo0 };
	for (; t < tratto4.delta_t; t += TIME_CONST_S, ct += TIME_CONST_S) {
		jt = -jerk * t;                   // -jerk*t
		jt2 = 0.5 * jt * t;               // 1/2 * jerk * t^2
		at = tratto4.accelerazione0 * t;  // a0 * t
		a5 = jt + tratto4.accelerazione0;
		v5 = jt2 + at + tratto4.veloctia0;
		s5 = jt2 * t / 3 + at * t * 0.5 + tratto4.veloctia0 * t + tratto4.spazio0;
#if ARDUINO_COMPILE
		s.write(s5);
		delay(TIME_CONST_MS);
#endif
#if MATLAB_COMPILE
		ML->appendAccData(ct, a5);
		ML->appendData(ct, s5);
#endif
	}
#if MATLAB_COMPILE
	ML->addEndDataStep();
#endif
	return { s5, v5, a5, ct, 0 };
}

InfoTratto tratto6(InfoTratto tratto5, double jerk, Servo s) {
#if MATLAB_COMPILE
	auto* ML = getMatLAB();
#endif
	double s6{ 0 }, v6{ 0 }, t{ 0 };
	// l'accelerazione � costante ed � pari a quella di fine tratto 5
	const double a6{ tratto5.accelerazione0 };
	// variabili di supporto
	double at{ 0 };
	// current time: a differenza della variabile "t" misura il tempo dall'inizio del tratto 1
	double ct{ tratto5.tempo0 };
	for (; t < tratto5.delta_t; t += TIME_CONST_S, ct += TIME_CONST_S) {
		at = tratto5.accelerazione0 * t;
		v6 = at + tratto5.veloctia0;
		s6 = at * t * 0.5 + tratto5.veloctia0 * t + tratto5.spazio0;
#if ARDUINO_COMPILE
		s.write(s6);
		delay(TIME_CONST_MS);
#endif
#if MATLAB_COMPILE
		ML->appendAccData(ct, a6);
		ML->appendData(ct, s6);
#endif
	}
#if MATLAB_COMPILE
	ML->addEndDataStep();
#endif
	return { s6, v6, a6, ct, 0 };
}


InfoTratto tratto7(InfoTratto tratto6, double jerk, Servo s) {
#if MATLAB_COMPILE
	auto* ML = getMatLAB();
#endif
	double s7{ 0 }, v7{ 0 }, a7{ 0 }, t{ 0 };
	// variabili di supporto
	double jt{ 0 }, jt2{ 0 }, at{ 0 };
	// current time: a differenza della variabile "t" misura il tempo dall'inizio del tratto 1
	double ct{ tratto6.tempo0 };
	for (; t < tratto6.delta_t; t += TIME_CONST_S, ct += TIME_CONST_S) {
		jt = jerk * t;                    // jerk*t
		jt2 = 0.5 * jt * t;               // 1/2 * jerk * t^2
		at = tratto6.accelerazione0 * t;  // a0 * t
		a7 = jt + tratto6.accelerazione0;
		v7 = jt2 + at + tratto6.veloctia0;
		s7 = jt2 * t / 3 + at * t * 0.5 + tratto6.veloctia0 * t + tratto6.spazio0;
#if ARDUINO_COMPILE
		s.write(s7);
		delay(TIME_CONST_MS);
#endif
#if MATLAB_COMPILE
		ML->appendAccData(ct, a7);
		ML->appendData(ct, s7);
#endif
	}
#if MATLAB_COMPILE
	ML->addEndDataStep();
#endif
	return { s7, v7, a7, ct, 0 };
}