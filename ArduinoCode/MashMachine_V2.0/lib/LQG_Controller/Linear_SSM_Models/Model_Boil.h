#pragma once

struct BoilModel {
    double kLoss_W_per_degC;   // same as before
    double P_evap_W;           // constant evaporation power
    double Kr;                 // optional feedback gain
};

extern const BoilModel modelBoil;