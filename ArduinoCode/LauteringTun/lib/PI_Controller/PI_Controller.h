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

    void SetHeaterPowerLimits(double min_W, double max_W);
    void SetCaptureBands(double band1DegC, double band2DegC, double gamma); 
    void SetAmbientTemp(double* ambient);
    void SetIntegralLimit(double maxI);
    void Reset(double Q0_W);
    void SetSampleTime(uint32_t ms);
    double GetCurrentPower_W() const;
    
    bool Compute();

private:
    // ===== IO =====
    double* T_;
    double* U_;
    double* Tset_;
    double* Tair_;

    double kp_;
    double ki_;        // INTERNAL (scaled by Ts)
    double ki_disp_;   // USER Ki [1/s]
    double kHeat_;

    double I_;
    double Imax_;

    double band1_;
    double band2_;
    double gamma_;
    double outMin_;
    double outMax_;

    // ===== Power =====
    double Q_prev_W_;
    double Qmin_W_;
    double Qmax_W_;

    uint32_t Ts_ms_;
    uint32_t lastTime_;

    static double Clamp(double x, double lo, double hi);
};
