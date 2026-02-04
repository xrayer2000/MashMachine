#pragma once
#include <Arduino.h>
#include <math.h>
#include "../../include/mash_config.h"
#include "LQGModel.h"

class LQGController
{
public:
    static constexpr int Ne = 3;
    // ===== Active model =====
    const LQGModel* model_;

    LQGController(double* input,
                  double* output,
                  double* setpoint,
                  double* airTemp);

    void loadModel(const LQGModel& model);
    void InitEstimator();

    bool Compute();
    void Reset(double Q0_W);
    void SetCaptureBands(double band1DegC, double band2DegC, double gamma);    
    void SetHeaterPowerLimits(double min_W, double max_W);
    double GetCurrentPower_W() const;

private:
  
    // ===== States =====
    double xhat_[Ne];              // estimated states (incl integrator)

    // ===== IO =====
    double* T_;
    double* U_;
    double* Tset_;
    double* Tair_;

    double band1_= 10.0; // °C 
    double band2_ = 1.0;  // °C 
    double gamma_ = 0.3;

    // ===== Timing =====
    uint32_t lastTime_ms_;
    const double Ts_s_ = 1.0;

    // ===== Power =====
    double Q_prev_W_;
    double Qmin_W_;
    double Qmax_W_;
    double dQmax_W_;
    double Q_transport_ = 30.0; // [W]
    bool lqgi_saturated_prev_ = false;

    // ===== Internal =====
    void UpdateSteadyState(double r);

    static double Clamp(double x, double lo, double hi);
    static double Dot(const double* a, const double* b, int n);
};
