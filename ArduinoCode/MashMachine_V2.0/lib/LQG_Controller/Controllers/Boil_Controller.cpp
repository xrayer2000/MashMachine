#include "Boil_Controller.h"
#include <algorithm>
using std::max;

void BoilController::setInputs(double T, double Tair, double Tset)
{
    T_ = T;
    Tair_ = Tair;
    Tset_ = Tset;
}

void BoilController::setInitialPower(double Q_init)
{
    Q_offset_ = Q_init;
}

String BoilController::GetDebugString() const {

    char buf[256];

    snprintf(buf, sizeof(buf), 
    "[%-5s][%-8s] T=%6.3f Tair=%4.1f Tset=%2.0f err=%5.2f |"
    " Qff=%6.1f Q=%6.1f",
        "BOIL", "MODEL_Boil", dbg_T_, dbg_Tair_, dbg_r_, dbg_error_,
        dbg_Qff_, dbg_Qsat_
    );

    return String(buf);
}


bool BoilController::Compute()  
{
    double Qff   = model_.kLoss_W_per_degC * (T_ - Tair_);
    double error = Tset_ - T_;
    double u_Kr  = model_.Kr * error;

    // target boil power
    double Q_target = Qff + model_.P_evap_W + u_Kr;

    // smooth transition
    Q_offset_ += (Q_target - Q_offset_) * 0.025;
    Q_offset_ = max(Q_offset_, Q_target);  // never undershoot

    Q_ = Q_offset_;

    
    // =========================================================
    // Debug
    // =========================================================

    dbg_T_      = T_;
    dbg_Tair_   = Tair_;
    dbg_r_      = Tset_;
    dbg_error_  = error;
    dbg_Qff_    = Qff;
    dbg_Qsat_   = Q_;

    return true;  
}