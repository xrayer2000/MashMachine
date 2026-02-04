#pragma once
#include <Arduino.h>
#include "../../include/mash_config.h"

class PI_Controller {
public:
    PI_Controller(double* input,
                   double* output,
                   double* setpoint,
                   double* airTemp);

    void SetTunings(double Kp, double Ki, double kHeat);

    void SetOutputLimits(double minOut, double maxOut);
    void SetCaptureBands(double band1DegC, double band2DegC);
    void SetIntegralLimit(double maxI);
    void ResetIntegral();
    void SetSampleTime(uint32_t ms);
    
    bool Compute();

private:
    double* T;
    double* U;
    double* Tset;
    double* Tair;

    double kp;
    double ki;        // INTERNAL (scaled by Ts)
    double ki_disp;   // USER Ki [1/s]
    double kHeat;

    double I;
    double Imax;

    double band1;
    double band2;
    double outMin;
    double outMax;

    uint32_t Ts_ms;
    uint32_t lastTime;
};
