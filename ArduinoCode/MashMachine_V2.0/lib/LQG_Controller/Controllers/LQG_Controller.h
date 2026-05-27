#pragma once
#include <Arduino.h>
#include <math.h>
#include "../../include/mash_config.h"
#include "../Linear_SSM_Models/LQGModel.h"
#include "IController.h"

//-----------------------------------------------------------------------
//Model och enums
enum ModelID { MODEL_50, MODEL_65, MODEL_75, MODEL_Boil };
//-----------------------------------------------------------------------

class LQGController : public IController
{
public:
    static constexpr int Ne = 3;
    char phase_str_[16];
    double dbg_T_;
    double dbg_Tair_;
    double dbg_r_;
    double dbg_error_;
    double dbg_innov_;
    double dbg_Qff_;
    double dbg_u_lqgi_;
    double dbg_Qsat_;
    double dbg_Ttilde_;

    // ===== Active model =====
    const LQGModel* model_;

    LQGController(double* input,
                  double* output,
                  double* setpoint,
                  double* airTemp);

    void loadModel(const LQGModel& model);
    void InitEstimator();
    void resetIntegrator();
    // ===== Interface =====
    String GetDebugString() const;
    bool Compute() override;
    void Init(double Q0_W) override;
    double GetCurrentPower_W() const override;
    void setControlOutput(double Q_W);

    void SetCaptureBands(double band1DegC,
                         double band2DegC,
                         double gamma);

    void SetAmbientTemp(double* ambient);

    void SetHeaterPowerLimits(double min_W,
                              double max_W);
                              public:
    void SetActiveModel(ModelID id) { activeModel_ = id; }

private:

    ModelID activeModel_ = MODEL_65;
    // =====================================================
    // States
    // xhat_[0] = temperature state
    // xhat_[1] = hidden thermal state
    // xhat_[2] = unused (kept for compatibility)
    // =====================================================
    double xhat_[Ne];

    // =====================================================
    // Manual integrator
    // =====================================================
    double xI_ = 0.0;
    const double XI_MAX = 8000.0;
    // =====================================================
    // IO
    // =====================================================
    double* T_;
    double* U_;
    double* Tset_;
    double* Tair_;

    // =====================================================
    // Phase logic
    // =====================================================
    double band1_ = 4.5;
    double band2_ = 1.0;
    double gamma_ = 0.8;
    bool in_phase3_ = false;
    // =====================================================
    // Timing
    // =====================================================
    uint32_t lastTime_ms_;
    const double Ts_s_ = 1.0;

    // =====================================================
    // Power
    // =====================================================
    double Q_prev_W_;
    double Qmin_W_;
    double Qmax_W_;
    double dQmax_W_;

    double Q_transport_ = 30.0;

    bool lqgi_saturated_prev_ = false;

    // =====================================================
    // Internal
    // =====================================================
    void UpdateSteadyState(double r);

    static double Clamp(double x,
                        double lo,
                        double hi);

    static double Dot(const double* a,
                      const double* b,
                      int n);

    static void MatVec(const double (*A)[LQGModel::Ne],
                    const double* x,
                    double* out,
                    int rows,
                    int cols);

    const char* modelIDToString(ModelID id) const;
};

// #pragma once
// #include <Arduino.h>
// #include <math.h>
// #include "../../include/mash_config.h"
// #include "../Linear_SSM_Models/LQGModel.h"
// #include "IController.h"

// class LQGController : public IController
// {
// public:
//     static constexpr int Ne = 3;
//     // ===== Active model =====
//     const LQGModel* model_;

//     LQGController(double* input,
//                   double* output,
//                   double* setpoint,
//                   double* airTemp);

//     void loadModel(const LQGModel& model);
//     void InitEstimator();

//     // ✅ interface compliance
//     bool Compute() override;
//     void Init(double Q0_W) override;
//     double GetCurrentPower_W() const override;
//     void setControlOutput(double Q_W);

//     void SetCaptureBands(double band1DegC, double band2DegC, double gamma);    
//     void SetAmbientTemp(double* ambient);
//     void SetHeaterPowerLimits(double min_W, double max_W);

// private:
  
//     // ===== States =====
//     double xhat_[Ne];              // estimated states (incl integrator)

//     // ===== IO =====
//     double* T_;
//     double* U_;
//     double* Tset_;
//     double* Tair_;

//     double band1_= 10.0; // °C 
//     double band2_ = 1.0;  // °C 
//     double gamma_ = 0.3;

//     // ===== Timing =====
//     uint32_t lastTime_ms_;
//     const double Ts_s_ = 1.0;

//     // ===== Power =====
//     double Q_prev_W_;
//     double Qmin_W_;
//     double Qmax_W_;
//     double dQmax_W_;
//     double Q_transport_ = 30.0; // [W]
//     bool lqgi_saturated_prev_ = false;

//     // ===== Internal =====
//     void UpdateSteadyState(double r);

//     static double Clamp(double x, double lo, double hi);
//     static double Dot(const double* a, const double* b, int n);
// };
