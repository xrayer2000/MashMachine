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

    band1_ = 4.5; // °C 
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

void LQGController::Init(double Q0_W)
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

void LQGController::SetAmbientTemp(double* ambient)
{
    this->Tair_ = ambient;
}

void LQGController::InitEstimator()
{
    xhat_[0] = *T_;   // temperature state
    xhat_[1] = 0.0;   // hidden dynamic state
    xI_      = 0.0;   // integrator
}

void LQGController::resetIntegrator()
{
    // Current temperature deviation from setpoint (in estimator coordinates)
    const double Ttilde = xhat_[0] - *Tset_;

    // Solve for xI such that u_lqgi = 0 at the moment of reset.
    //
    // The control law is:
    //   u = -(Le[0]*Ttilde + Le[1]*x2 + Le[2]*xI)
    //
    // Setting u = 0 and solving for xI:
    //   Le[2]*xI = -(Le[0]*Ttilde + Le[1]*x2)
    //   xI = -(Le[0]*Ttilde + Le[1]*x2) / Le[2]
    //
    // This means the integrator starts neutral — it contributes 0 to u,
    // so Qcmd = Qff at the first timestep after reset.
    // The integrator then accumulates error organically under the new model.
    xI_ = (-model_->Le[0] * Ttilde - model_->Le[1] * xhat_[1]) / model_->Le[2];

    // Clamp to prevent extreme values if Le[2] is very small (division amplification)
    xI_ = Clamp(xI_, -XI_MAX, XI_MAX);
}

double LQGController::GetCurrentPower_W() const {
    return Q_prev_W_;
}

void LQGController::setControlOutput(double Q_W)
{
    // Clamp to valid limits
    Q_prev_W_ = Clamp(Q_W, Qmin_W_, Qmax_W_);

    // Optional: also update output immediately (keeps PWM consistent)
    if (U_)
        *U_ = Clamp(Q_prev_W_ * W_to_PWM, 0.0, PWM_MAX);
}

String LQGController::GetDebugString() const {
    const double Ttilde_Le1 = -model_->Le[0] * dbg_Ttilde_;
    const double x2_Le2 = -model_->Le[1] * xhat_[1];
    const double xI_Le3 = -model_->Le[2] * xI_;

    char buf[256];

    snprintf(buf, sizeof(buf), 
    "[%-5s][%-8s] T=%6.3f Tair=%4.1f Tset=%2.0f err=%5.2f | x1=%6.3f x2=%8.3f xI=%7.1f inn=%9.5f |"
    " Qff=%6.1f u=%6.1f Q=%6.1f | Ttilde_Le1=%6.2f x2_Le2=%6.1f xI_Le3=%6.1f",
        phase_str_, modelIDToString(activeModel_), dbg_T_, dbg_Tair_, dbg_r_, dbg_error_,
        xhat_[0], xhat_[1], xI_, dbg_innov_,
        dbg_Qff_, dbg_u_lqgi_, dbg_Qsat_,
        Ttilde_Le1, x2_Le2, xI_Le3
    );

    return String(buf);
}

bool LQGController::Compute()
{
    const uint32_t now = millis();
    if (now - lastTime_ms_ < 1000) return false;
    lastTime_ms_ = now;

    // =========================================================
    // Measurements
    // =========================================================
    const double T     = *T_;
    const double r     = *Tset_;
    const double Tair  = *Tair_;
    const double error = r - xhat_[0]; // Kalman smoothed error

    // =========================================================
    // Feedforward
    // =========================================================
    const double Qff = model_->kLoss_W_per_degC * (r - Tair);

    // =========================================================
    // Kalman predict
    // =========================================================
    double xpred[2] = {0.0};
    MatVec(model_->Ae, xhat_, xpred, 2, 2);
    for (int i = 0; i < 2; i++)
        xpred[i] += model_->Be[i] * Q_prev_W_ + model_->Ee[i] * Tair;

    // =========================================================
    // Kalman update
    // =========================================================
    const double innov = T - xpred[0];
    for (int i = 0; i < 2; i++)
        xhat_[i] = xpred[i] + model_->Ke[i] * innov;

    // =========================================================
    // Observer outputs — always computed, every phase
    // =========================================================
    const double Ttilde = xhat_[0] - r;
    const double u_Kr   = 0.0;  // reserved: Kr_ * error
    const double xe[3]  = { Ttilde, xhat_[1], xI_ };
    const double u_lqgi = -Dot(model_->Le, xe, 3);

    // =========================================================
    // Phase selection
    // =========================================================
    double Qcmd = Qff;

    // ---------------------------------------------------------
    // PHASE 1 : BANG
    // ---------------------------------------------------------
    if (error > band1_)
    {
        Qcmd       = Qmax_W_;
        in_phase3_ = false;
    }

    // ---------------------------------------------------------
    // PHASE 2 : SLOPE
    // ---------------------------------------------------------
    else if (error > band2_)
    {
        // Clamp u_lqgi influence in slope to prevent diverged x2 from warping the curve, causeing big quantization on the Qcmd output.
        const double u_slope = Clamp(u_lqgi, -Qff, Qff);

        double deltaY = Qmax_W_ - Qff - u_slope - u_Kr;
        double deltaX = band1_ - band2_;
        double t = (band1_ - error) / deltaX;
        t = Clamp(t, 0.0, 1.0);
        double s = pow(t, gamma_);
        Qcmd = Qmax_W_ - deltaY * s;
        Qcmd = Clamp(Qcmd, Qff + u_slope + u_Kr, Qmax_W_);
        in_phase3_ = false;
    }

    // ---------------------------------------------------------
    // PHASE 3 : LQGI
    // ---------------------------------------------------------
    else
    {
        // Bumpless transfer from Phase 1/2
        if (!in_phase3_)
        {
            const double u_target = Q_prev_W_ - Qff - u_Kr;
            xI_ = (-u_target - model_->Le[0] * Ttilde - model_->Le[1] * xhat_[1]) / model_->Le[2];
            in_phase3_ = true;
        }

        // Integrator update with anti-windup
        const bool saturated_low  = (Q_prev_W_ <= Qmin_W_);
        const bool saturated_high = (Q_prev_W_ >= Qmax_W_);
        if (!(saturated_low  && (r - T) < 0) &&
            !(saturated_high && (r - T) > 0))
        {
            xI_ = model_->alpha * xI_ + (r - T);
        }
        xI_ = Clamp(xI_, -XI_MAX, XI_MAX);

        Qcmd = Qff + u_lqgi + u_Kr;
    }

    // =========================================================
    // Actuator saturation
    // =========================================================
    const double Qsat = Clamp(Qcmd, Qmin_W_, Qmax_W_);
    Q_prev_W_ = Qsat;

    // =========================================================
    // Debug
    // =========================================================
    const double Tterm  = -model_->Le[0] * Ttilde;
    const double X2term = -model_->Le[1] * xhat_[1];
    const double Iterm  = -model_->Le[2] * xI_;

    strncpy(phase_str_,
            (error > band1_) ? "BANG" : (error > band2_) ? "SLOPE" : "LQGI",
            sizeof(phase_str_));

    dbg_T_      = T;
    dbg_Tair_   = Tair;
    dbg_r_      = r;
    dbg_error_  = error;
    dbg_innov_  = innov;
    dbg_Qff_    = Qff;
    dbg_u_lqgi_ = u_lqgi;
    dbg_Qsat_   = Qsat;
    dbg_Ttilde_ = Ttilde;

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

void LQGController::MatVec(const double (*A)[LQGModel::Ne], const double* x, double* out, int rows, int cols)
{
    for (int i = 0; i < rows; i++)
    {
        out[i] = 0.0;
        for (int j = 0; j < cols; j++)
        {
            out[i] += A[i][j] * x[j];
        }
    }
}

const char* LQGController::modelIDToString(ModelID id) const
{
    switch (id)
    {
        case MODEL_50:   return "MODEL_50";
        case MODEL_65:   return "MODEL_65";
        case MODEL_75:   return "MODEL_75";
        case MODEL_Boil: return "MODEL_Boil";
        default:         return "UNKNOWN";
    }
}
