#include "spline.h"

#include "comms.h"
#if USE_ARDUINO_H
#include <Arduino.h>
#include <Servo.h>
#else
#include "FakeArduino.h"
#include "FakeServo.h"
#endif

#include <vector>
#include <cmath>

#if MATLAB_COMPILE
#include "matlab.h"
#endif

// Funzione che esegue l'eliminazione Gaussiana
static std::vector<double> gaussian_elimination(std::vector<std::vector<double>>& A, std::vector<double>& B) {
    int n = B.size();
    for (int i = 0; i < n; i++) {
        // Trovo l'elemento pivot (il massimo nella colonna corrente)
        int maxRow = i;
        for (int k = i + 1; k < n; k++) {
            if (abs(A[k][i]) > abs(A[maxRow][i])) {
                maxRow = k;
            }
        }
        // Inverto le righe se necessario
        if (maxRow != i) {
            std::swap(A[i], A[maxRow]);
            std::swap(B[i], B[maxRow]);
        }
        // Elimina i valori sotto la diagonale
        for (int k = i + 1; k < n; k++) {
            if (A[k][i] == 0) continue;  // Se � gi� zero vado vanti
            double factor = A[k][i] / A[i][i];
            for (int j = i; j < n; j++) {
                A[k][j] -= factor * A[i][j];  // Aggiorno la colonna solo se necessario
            }
            B[k] -= factor * B[i];  // Aggiorno anche il vettore B
        }
    }
    // Metodo di sostituzione all'indietro
    std::vector<double> X(n);
    for (int i = n - 1; i >= 0; i--) {
        X[i] = B[i];
        for (int j = i + 1; j < n; j++) {
            X[i] -= A[i][j] * X[j];
        }
        X[i] /= A[i][i];  // Divisione finale
    }
    return X;  // Restituisco il vettore soluzione
}

void spline(std::vector<double> &tempi, std::vector<double> &punti, Servo *s) {
#if MATLAB_COMPILE
    auto ML = getMatLAB();
#endif

    int numero_punti = punti.size();
    // Variabile che contiene i coefficienti delle funzioni ricostruite
    std::vector<std::vector<double>> c;
    for (int i = 0; i < numero_punti - 1; i++) {
        // Calcolo la matrice A
        std::vector<std::vector<double>> A = {
            {pow(tempi[i], 3), pow(tempi[i], 2), tempi[i], 1},
            {pow(tempi[i + 1], 3), pow(tempi[i + 1], 2), tempi[i + 1], 1},
            {3 * pow(tempi[i], 2), 2 * tempi[i], 1, 0},
            {3 * pow(tempi[i + 1], 2), 2 * tempi[i + 1], 1, 0}
        };
        // Imposto il vettore B
        std::vector<double> B = { punti[i], punti[i + 1], 0, 0 };
        // Risolve il sistema matriciale A * x = B
        std::vector<double> coeffs = gaussian_elimination(A, B);
        // Salva i coefficienti
        c.push_back(coeffs);
    }

    // Example of using the function f for interpolation
    for (int i = 0; i < numero_punti - 1; i++) {
        // Define f(x) for the interval tempi[i] to tempi[i+1]
        auto f = [&](double x) {
            return c[i][0] * pow(x, 3) + c[i][1] * pow(x, 2) + c[i][2] * x + c[i][3];
        };

        //TODO: calcolare tra due punti
        double pos = 0.0;
        for (double t = tempi[i]; t < tempi[i + 1]; t += TIME_CONST_S) {
            pos = f(t);
#if ARDUINO_COMPILE
            s->write(pos);
            delay(TIME_CONST_MS);
#endif
#if MATLAB_COMPILE
            ML->appendData(t, pos);
#endif
        }
#if MATLAB_COMPILE
        ML->addEndDataStep();
#endif
    }
}