#include "Model_65C.h"

// =====================
// Dimensions
// Ne = 2 states + 1 integrator
// =====================

static const double kLoss_W_per_degC = 4.5;
const double Model_65C::alpha_ = 0.99;

const double Model_65C::Ae_[Ne][Ne] = {
    {  0.99999866,  0.00083445, 0.0 },
    { -0.00252717, 0.68826604, 0.0 },
    { -1.0,        0.0,        0.99 }
};

const double Model_65C::Be_[Ne] = {
    -0.00010715,
     0.04285759,
     0.0
};

const double Model_65C::Ce_[Ne] = {
    1.0,
    0.0,
    0.0
};

const double Model_65C::Ee_[Ne] = {
     0.00232888,
    -0.90061350,
     0.0
};

// =====================
// LQGI gain
// =====================

const double Model_65C::Le_[Ne] = {
    69.02799661,
     0.18471342,
    -0.00928744
};

// =====================
// Kalman gain
// =====================

const double Model_65C::Ke_[Ne] = {
    0.59551306,
    0.05277051,
    0.0
};

// =====================
// Model struct
// =====================

const LQGModel model65 = {
    kLoss_W_per_degC,
    Model_65C::Ae_,
    Model_65C::Be_,
    Model_65C::Ce_,
    Model_65C::Ee_,
    Model_65C::Le_,
    Model_65C::Ke_,
    Model_65C::alpha_
};