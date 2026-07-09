#include "Model_50C.h"

// =====================
// Dimensions
// Ne = 2 states + 1 integrator
// =====================

static const double kLoss_W_per_degC = 2.7; //2.5
const double Model_50C::alpha_ = 0.99;

const double Model_50C::Ae_[Ne][Ne] = {
    {  0.99999767,  0.00093799, 0.0 },
    { -0.00455499,  0.87859862, 0.0 },
    { -1.0,         0.0,        0.99 }
};

const double Model_50C::Be_[Ne] = {
    -0.00003788,
     0.00591111,
     0.0
};

const double Model_50C::Ce_[Ne] = {
    1.0,
    0.0,
    0.0
};

const double Model_50C::Ee_[Ne] = {
     0.00631332,
    -0.81075829,
     0.0
};

const double Model_50C::Le_[Ne] = {
    65.29429361,
     0.50396677,
    -0.01000788
};

const double Model_50C::Ke_[Ne] = {
    0.59561117,
    0.19675401,
    0.0
};

// =====================
// Model struct
// =====================

const LQGModel model50 = {
    kLoss_W_per_degC,
    Model_50C::Ae_,
    Model_50C::Be_,
    Model_50C::Ce_,
    Model_50C::Ee_,
    Model_50C::Le_,
    Model_50C::Ke_,
    Model_50C::alpha_   
};