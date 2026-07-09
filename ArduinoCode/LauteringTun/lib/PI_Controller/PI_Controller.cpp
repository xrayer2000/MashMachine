#include "PI_Controller.h"

// ============================================================
// CONSTRUCTOR / RESET
// ============================================================

PI_Controller::PI_Controller(double* input, double* output, double* setpoint, double* airTemp)
: T_(input), U_(output), Tset_(setpoint), Tair_(airTemp)
{
    kp_ = 0.0; //50C: kp = 45,     //65C: kp = 30,     75c: kp = 21.0 (65C,kp=70 -> oscillations), (75C,kp=70 -> oscillations)
    ki_ = 0.00; //
    kHeat_ = 0; //0.7 * 1.65; // 165W/C 70% conservative  //kHeat = 1.5

    I_ = 0.0;
    Imax_ = 0.0;

    band1_ = 4.0; // °C 
    band2_ = 0.5;  // °C 
    gamma_ = 0.5;

    Qmin_W_ = 0.0;
    Qmax_W_ = heaterRatedPower_W;

    Ts_ms_ = 1000;
    lastTime_ = 0;

    // scale ki on construction
    ki_ = ki_disp_ * (Ts_ms_ / 1000.0);
}


// ============================================================
// PARAMETER SETTERS
// ============================================================

void PI_Controller::SetTunings(double Kp, double Ki, double kHeat)
{
    if (Kp < 0 || Ki < 0 || kHeat_ < 0) return;

    kp_      = Kp;
    ki_disp_ = Ki;
    kHeat_   = kHeat;
    ki_      = ki_disp_ * (Ts_ms_ / 1000.0);
}

void PI_Controller::SetAmbientTemp(double* ambient)
{
    this->Tair_ = ambient;
}

void PI_Controller::Reset(double Q0_W)
{
    Q_prev_W_ = Clamp(Q0_W, Qmin_W_, Qmax_W_);
    I_        = 0.0;
    lastTime_ = millis();
}

void PI_Controller::SetHeaterPowerLimits(double min_W, double max_W)
{
    Qmin_W_ = min_W;
    Qmax_W_ = max_W;
    Q_prev_W_ = Clamp(Q_prev_W_, Qmin_W_, Qmax_W_);
}

void PI_Controller::SetCaptureBands(double band1DegC, double band2DegC, double gamma)
{
    band1_ = band1DegC;
    band2_ = band2DegC;
    gamma_ = gamma;
}

void PI_Controller::SetIntegralLimit(double maxI)
{
    Imax_ = maxI;
}

void PI_Controller::SetSampleTime(uint32_t ms)
{
    if (ms == 0) return;
    ki_    *= (double)ms / (double)Ts_ms_;
    Ts_ms_  = ms;
}

double PI_Controller::GetCurrentPower_W() const {
    return Q_prev_W_;
}

// ============================================================
// MAIN CONTROLLER
// ============================================================

bool PI_Controller::Compute()
{
    uint32_t now = millis();
    if (now - lastTime_ < Ts_ms_) return false;
    lastTime_ = now;

    const double T    = *T_;
    const double r    = *Tset_;
    const double Tair = *Tair_;
    const double error = r - T;

    // =========================================================
    // Feedforward (heat loss)
    // =========================================================
    double Qff = kHeat_ * (r - Tair);
    Qff = max(0.0, Qff);

    double Qcmd = 0.0;

    // =========================================================
    // PHASE LOGIC 
    // =========================================================
  
    // ---------- PHASE 1: BANG ----------
    if (error > band1_) {
        // Serial.printf("PI: PHASE 1 BANG:\t");
        I_    = 0.0;
        Qcmd  = Qmax_W_;
    }
    // ---------- PHASE 2: SLOPE ----------
    else if (error > band2_) {
        // Serial.printf("PI: PHASE 2 SLOPE:\t");
        I_ = 0.0;

        double P      = kp_ * error;
        double deltaY = Qmax_W_ - Qff - P;
        double deltaX = band1_ - band2_;
        double t      = (band1_ - error) / deltaX;
        t             = Clamp(t, 0.0, 1.0);
        double s      = pow(t, gamma_);
        Qcmd          = Qmax_W_ - deltaY * s;
        Qcmd = Clamp(Qcmd, Qff + P, Qmax_W_);
    }
    // ---------- PHASE 3: REGULATE (PI + FF) ----------
    else {
        // Serial.printf("PI: PHASE 3 REGULATE:\t");

        double P       = kp_ * error;
        double Q_PI    = P + I_;
        double Q_unsat = Qff + Q_PI;
        Qcmd           = Clamp(Q_unsat, Qmin_W_, Qmax_W_);

        // Anti-windup: integrate only when unsaturated
        if (Qcmd == Q_unsat) {
            I_ += ki_ * error;
            I_  = Clamp(I_, -Imax_, Imax_);
        }
    }

    // =========================================================
    // Saturation & output
    // =========================================================
    Q_prev_W_ = Clamp(Qcmd, Qmin_W_, Qmax_W_);
    // *U_ = Clamp(Q_prev_W_ * W_to_PWM, 0.0, PWM_MAX);

    // =========================================================
    // Logging
    // =========================================================
    // Serial.printf(
    //     "T=%.2f Tset=%.2f Error=%.2f | "
    //     "Qff=%.1f I=%.3f Q=%.1f\n",
    //     T, r, error, Qff, I_, Q_prev_W_
    // );

    return true;
}

double PI_Controller::Clamp(double x, double lo, double hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}