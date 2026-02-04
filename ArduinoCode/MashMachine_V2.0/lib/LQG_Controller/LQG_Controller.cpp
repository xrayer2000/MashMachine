#include "LQG_Controller.h"
#include <Arduino.h>
#include <math.h>

// ============================================================
// CONSTRUCTOR / RESET
// ============================================================

LQGController::LQGController(double* input,
                             double* output,
                             double* setpoint,
                             double* airTemp)
: T_(input), U_(output), Tset_(setpoint), Tair_(airTemp)
{
    Qmin_W_ = 0.0;
    Qmax_W_ = heaterRatedPower_W;
    dQmax_W_ = 100.0;

    band1_ = 10.0; // °C 
    band2_ = 1.0;  // °C 
    gamma_ = 0.35;
    
    // Initialize estimator states to zero
    for (int i = 0; i < Ne; ++i)
        xhat_[i] = 0.0;
}

// ============================================================
// PARAMETER SETTERS
// ============================================================

void LQGController::SetHeaterPowerLimits(double min_W, double max_W)
{
    Qmin_W_ = min_W;
    Qmax_W_ = max_W;

    Q_prev_W_ = Clamp(Q_prev_W_, Qmin_W_, Qmax_W_);
}

void LQGController::SetCaptureBands(double band1DegC, double band2DegC, double gamma)
{
    band1_ = band1DegC;
    band2_ = band2DegC;
    gamma_ = gamma;
}

void LQGController::Reset(double Q0_W)
{
    // 1. Bumpless start från faktisk effekt
    Q_prev_W_ = Clamp(Q0_W, Qmin_W_, Qmax_W_);

    // 2. Sätt utgång direkt
    if (U_)
        *U_ = Clamp(Q_prev_W_ * W_to_PWM, 0.0, PWM_MAX);

    // 3. Tidsbas
    lastTime_ms_ = millis();
}

void LQGController::loadModel(const LQGModel& m)
{
    model_ = &m;
}

void LQGController::InitEstimator()
{
    xhat_[0] = 0.0;
    xhat_[1] = *T_;   // Initialize temperature state to current measured temperature
    xhat_[2] = 0.0;   // integrator starts at zero
}

double LQGController::GetCurrentPower_W() const {
    return Q_prev_W_;
}

// ============================================================
// MAIN CONTROLLER (LQGI)
// ============================================================

bool LQGController::Compute()
{
    const uint32_t now = millis();
    if (now - lastTime_ms_ < 1000) return false;
    lastTime_ms_ = now;

    const double T    = *T_;
    const double r    = *Tset_;
    const double Tair = *Tair_;
    const double error = r - T;
    double r_eff = r;
    double u_raw = 0.0;
    bool lqgi_saturated = false;

    // =========================================================
    // Feedforward (physics only)
    // =========================================================
    double Qff = model_->kLoss_W_per_degC * (T - Tair);

    // =========================================================
    // Kalman predict xhat_next = (A_aug x + B_aug u + E_aug r)
    // =========================================================
    double xpred[Ne] = {0.0};
    for (int i = 0; i < Ne; i++) {
        for (int j = 0; j < Ne; j++)
        {
            xpred[i] += model_->Ae[i][j] * xhat_[j];
        }
        xpred[i] += model_->Be[i] * Q_prev_W_;
        if (!lqgi_saturated_prev_) {
            xpred[i] += model_->Ee[i] * r_eff;  
        }
    }

    // =========================================================
    // Innovation
    // =========================================================
    const double yhat  = Dot(model_->Ce, xpred, Ne);
    const double innov = T - yhat;

    // =========================================================
    // Kalman update (K_aug) - ALWAYS applied to correct estimates
    // =========================================================
    for (int i = 0; i < Ne; i++)
        xhat_[i] = xpred[i] + model_->Ke[i] * innov;

    double u_lqgi = 0.0;
    double Qcmd   = Qff;
    double u_Kr   = 0.0;
    double Kr_  = 0.0;

    Kr_ = 5 * model_->kLoss_W_per_degC;
    u_Kr = Kr_ * error;

    // =========================================================
    // PHASE LOGIC 
    // =========================================================
    u_lqgi = 0;
    // ---------- PHASE 1: BANG ----------
    if (error > band1_) {
        // Serial.printf("LQG: PHASE 1 BANG:\t");
        Qcmd = Qmax_W_;
    }
    // ---------- PHASE 2: SLOPE ----------
    else if (error > band2_) {
        // Serial.printf("LQG: PHASE 2 SLOPE:\t");

        double deltaY = Qmax_W_ - Qff - u_lqgi - u_Kr;
        double deltaX = band1_ - band2_;
        double t = (band1_ - error) / deltaX;
        t = Clamp(t, 0.0, 1.0);
        double s = pow(t, gamma_);
        Qcmd = Qmax_W_ - deltaY * s;

        // Serial.printf("t = %.3f | ", t);
        // Serial.printf("deltaY * s = %.3f | ", deltaY * s);

        Qcmd = Clamp(Qcmd, Qff + u_lqgi + u_Kr, Qmax_W_);
    }
    // ---------- PHASE 3: REGULATE (LQGI) ----------
    else {
        // Serial.printf("LQG: PHASE 3 REGULATE:\t");

        // =========================================================
        // LQR - Control law
        // =========================================================
        u_raw = 0.0;
        for (int i = 1; i < Ne; i++) // endast LQR på temp och integrator
        {
            if (i == 1) {
                // reglera på temperaturavvikelse
                const double Ttilde = xhat_[1] - r;
                u_raw -= model_->Le[i] * Ttilde;
                // Serial.printf("Le[%d]=%.3f Ttilde=%.3f | ", i, model_->Le[i], Ttilde);
            } 
            else 
            {
                u_raw -= model_->Le[i] * xhat_[i];
                // Serial.printf("Le[%d]=%.3f xhat=%.3f | ", i, model_->Le[i], xhat_[i]);
            }
        }
        // Serial.printf("u_raw=%.3f | ", u_raw);

        // tona ner LQR nära börvärdet
        double gL = fabs(error) / 0.3;        // full LQR utanför ±0.3 °C
        if (gL > 1.0) gL = 1.0;
        u_raw *= gL;

        // heater-only constraint
        u_lqgi = max(u_raw, 0.0);
        lqgi_saturated = (u_raw < 0.0);
       

        Qcmd = Qff + u_lqgi + u_Kr;
    }

    // =========================================================
    // Saturation & output
    // =========================================================
    SATURATE_AND_OUTPUT:
    double Qraw = Qcmd;
    double Qsat = Clamp(Qraw, Qmin_W_, Qmax_W_);

    Q_prev_W_ = Qsat;
    *U_ = Clamp(Qsat * W_to_PWM, 0.0, PWM_MAX);

    lqgi_saturated_prev_ = lqgi_saturated;

    // =========================================================
    // Logging
    // =========================================================

    // Serial.printf(
    //     "T=%.2f Tset=%.2f Error=%.2f | "
    //     "yhat=%.2f innov=%.3f | "
    //     "xI=%.3f | "
    //     "Qff=%.1f u=%.1f u_Kr=%.1f Q=%.1f\n",
    //     T, r, error,
    //     yhat, innov,
    //     xhat_[Ne-1],
    //     Qff, u_lqgi, u_Kr, Qsat
    // );

    return true;
}


// ============================================================
// HELPERS
// ============================================================

double LQGController::Clamp(double x, double lo, double hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

double LQGController::Dot(const double* a, const double* b, int n)
{
    double s = 0.0;
    for (int i = 0; i < n; i++) s += a[i] * b[i];
    return s;
}
