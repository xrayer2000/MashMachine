#include "PI_Controller.h"

PI_Controller::PI_Controller(double* input, double* output, double* setpoint, double* airTemp)
{
    T = input;
    U = output;
    Tset = setpoint;
    Tair = airTemp;

    kp = 30.0; //50C: kp = 45,     //65C: kp = 30,     75c: kp = 21.0 (65C,kp=70 -> oscillations), (75C,kp=70 -> oscillations)
    ki = 0.00; //
    kHeat = 0.7 * 1.65; // 165W/C 70% conservative  //kHeat = 1.5

    I = 0.0;
    Imax = 0.0;

    band1 = 10.0; // °C 
    band2 = 5.0;  // °C 
    outMin = 0;
    outMax = 4095;

    Ts_ms = 500;
    lastTime = 0;
}

void PI_Controller::SetTunings(double Kp, double Ki, double kHeat_)
{
    if (Kp < 0 || Ki < 0 || kHeat_ < 0) return;

    kp = Kp;
    ki_disp = Ki;      // store physical Ki [1/s]
    kHeat = kHeat_;

    // scale Ki by sample time
    double Ts = Ts_ms / 1000.0;
    ki = ki_disp * Ts;
}
void PI_Controller::ResetIntegral()
{
    I = 0.0;
}

void PI_Controller::SetOutputLimits(double minOut, double maxOut)
{
    outMin = minOut;
    outMax = maxOut;
}

void PI_Controller::SetCaptureBands(double band1DegC, double band2DegC)
{
    band1 = band1DegC;
    band2 = band2DegC;
}

void PI_Controller::SetIntegralLimit(double maxI)
{
    Imax = maxI;
}

void PI_Controller::SetSampleTime(uint32_t ms)
{
    if (ms == 0) return;

    double ratio = (double)ms / (double)Ts_ms;

    // rescale internal integrator
    ki *= ratio;

    Ts_ms = ms;
}

// bool PI_Controller::Compute()
// {
//     uint32_t now = millis();
//     if (now - lastTime < Ts_ms) return false;
//     lastTime = now;

//     double temp  = *T;
//     double error = *Tset - temp;

//     double Q_lossFF_W = kHeat * (temp - *Tair);
//     Q_lossFF_W = max(0.0, Q_lossFF_W);

//     double Q_W;

//     // ===== Phase 1: BANG =====
//     if (error > dT_preempt) {
//         I = 0.0;
//         Q_W = heaterRatedPower_W;
//         *U = Q_W * W_to_PWM;
//         return true;
//     }

//     // ===== Phase 1.5: COAST =====
//     if (error > band) {
//         Q_W = Q_lossFF_W;
//         *U = Q_W * W_to_PWM;
//         return true;
//     }

//     // ===== Phase 2: REGULATE =====
//     double Q_rampFF_W = 0.0;   // valfri, oftast 0 här

//     double P = kp * error;
//     double Q_PI_W = P + I;

//     double Q_unsat = Q_lossFF_W + Q_rampFF_W + Q_PI_W;
//     Q_W = constrain(Q_unsat, 0.0, heaterRatedPower_W);

//     if (Q_W == Q_unsat) {
//         I += ki * error * Ts_ms * 0.001;
//         I = constrain(I, -Imax, Imax);
//     }

//     *U = Q_W * W_to_PWM;
//     return true;
// }

bool PI_Controller::Compute()
{
    uint32_t now = millis();
    if (now - lastTime < Ts_ms) return false;
    lastTime = now;

    // -------- Measurements --------
    double temp  = *T;
    double error = *Tset - temp;

    // -------- Heat loss FF --------
    double Q_lossFF_W = kHeat * (temp - *Tair);
    Q_lossFF_W = max(0.0, Q_lossFF_W);

    double Q_W = 0.0;
    double P = kp * error;
  
    // ---- FAS 1: BANG ----
    if (error > band1) {
        I = 0.0;
        Q_W = heaterRatedPower_W;
        Serial.print("Phase 1: ");
    }
    // ---- FAS 2: SLOPE ----
    else if (error > band2) {
        I = 0.0;
        double deltaY = heaterRatedPower_W - Q_lossFF_W - P;
        double deltaX = band1 - band2;
        double slope = - (deltaY / deltaX);
        Q_W = heaterRatedPower_W + slope * (band1 - error);
        Serial.print("Phase 2: ");
        // Serial.print(", deltaY: ");
        // Serial.print(deltaY, 3);
        // Serial.print(", deltaX: ");
        // Serial.print(deltaX, 3);
        // Serial.print(", slope: ");
        // Serial.print(slope, 3);
    }
    // ---- FAS 2: REGULATE (PI + FF) ----
    else {
        Serial.print("Phase 3: ");

        double Q_PI_W = P + I;
        double Q_unsat = Q_lossFF_W + Q_PI_W;
        Q_W = constrain(Q_unsat, 0.0, heaterRatedPower_W);

        // Integral only if not saturated
        if (Q_W == Q_unsat) {
            I += ki * error * Ts_ms * 0.001;
            I = constrain(I, -Imax, Imax);
        }
    }

    Serial.println();

    // -------- Output --------
    *U = Q_W * W_to_PWM;
    return true;
}
