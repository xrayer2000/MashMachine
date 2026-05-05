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

    return true;  
}