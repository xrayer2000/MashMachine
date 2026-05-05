#include "Model_Boil.h"

// Tune this from experiments
static const double kLoss_W_per_degC = 5.50;

// Typical for ~9L wort
static const double P_evap_W = 1600.0;

// Small correction gain (optional)
static const double Kr = 5 * kLoss_W_per_degC;

const BoilModel modelBoil = {
    kLoss_W_per_degC,
    P_evap_W,
    Kr
};