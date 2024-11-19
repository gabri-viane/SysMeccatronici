#include "three.h"

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

void treTratti(Instructions ci, Servo *s) {
#if MATLAB_COMPILE
    auto M = getMatLAB();
#endif
    unsigned int durata_secondi = ci.tempo_tot_ms;// / 1000;  // Converto da ms a s perchè le formule sono in secondi

    char segno{1};
    if (ci.segno < 0) {                    // Guardo se devo andare in avanti o indietro
        segno = -1;                          // devo andare indietro
        // inverto il segno del delta per avere le accelerazioni corrette
    }
	
    // calcolo i tre intervalli di tempo (per A+, A=0, A-)
    double durate[] = {durata_secondi * ci.lambdas.cost_1 / 10.0,
                       durata_secondi * ci.lambdas.cost_2 / 10.0,
                       durata_secondi * ci.lambdas.cost_3 / 10.0};
    // creo una variabile di supporto per non effettuare più volte lo stesso calcolo
    double tmp = segno * ci.delta_angolo * 2.0 / durata_secondi;
    // ricavo la velocità massima che raggiungo
    double v_max = tmp / (2 + (-ci.lambdas.cost_1 - ci.lambdas.cost_3) / 10.0);
    // Aggiorno la variabile di supporto "derivandola" rispetto al tempo
    tmp = tmp / durata_secondi;
    // Creo una seconda variabile di supporto
    double tmp2 = 2 + (-ci.lambdas.cost_1 - ci.lambdas.cost_3) / 10.0;

    // Calcolo le accelerazioni A+ e A-
    double acc1 = tmp / (ci.lambdas.cost_1 / 10.0 * tmp2);
    double acc3 = tmp / (ci.lambdas.cost_3 / 10.0 * tmp2);

    double t{0};  // tempo che si usa per simulare il ciclo
#if MATLAB_COMPILE
    double ct{0};
#endif
    // Calcolo dello spazio per A+
    double angolo_corrente1{0.0};
    for (t = 0; t < durate[0]; t += TIME_CONST_S) {
        angolo_corrente1 = ci.ci.angolo_inizio + segno * 0.5 * acc1 * t * t;
#if ARDUINO_COMPILE
        s->write(angolo_corrente1);
        delay(TIME_CONST_MS);
#endif
#if MATLAB_COMPILE
        ct += TIME_CONST_S;
        M->appendData(ct, angolo_corrente1);
        M->appendAccData(ct, acc1);
#endif
    }
#if MATLAB_COMPILE
    M->addEndDataStep();
#endif
    // Calcolo dello spazio per A=0
    double angolo_corrente2{0.0};
    for (t = 0; t < durate[1]; t += TIME_CONST_S) {
        angolo_corrente2 = angolo_corrente1 + segno * v_max * t;
#if ARDUINO_COMPILE
        s->write(angolo_corrente2);
        delay(TIME_CONST_MS);
#endif
#if MATLAB_COMPILE
        ct += TIME_CONST_S;
        M->appendData(ct, angolo_corrente2);
        M->appendAccData(ct, 0);
#endif
    }
#if MATLAB_COMPILE
    M->addEndDataStep();
#endif
    // Calcolo dello spazio per A-
    double angolo_corrente3{0.0};
    for (t = 0; t < durate[2]; t += TIME_CONST_S) {
        angolo_corrente3 = angolo_corrente2 + segno * (v_max - 0.5 * acc3 * t) * t;
#if ARDUINO_COMPILE
        s->write(angolo_corrente3);
        delay(TIME_CONST_MS);
#endif
#if MATLAB_COMPILE
        ct += TIME_CONST_S;
        M->appendData(ct, angolo_corrente3);
        M->appendAccData(ct, -acc3);
#endif
    }
#if MATLAB_COMPILE
    M->addEndDataStep();
#endif
}
