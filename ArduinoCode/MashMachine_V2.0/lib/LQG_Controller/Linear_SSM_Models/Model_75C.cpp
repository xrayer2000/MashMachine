#include "Model_75C.h"

// =====================
// Dimensions
// Ne = 2 states + 1 integrator
// =====================

static const double kLoss_W_per_degC = 5; //75C
const double Model_75C::alpha_ = 0.99;

const double Model_75C::Ae_[Ne][Ne] = {
    {  0.99997718,  0.00054183,  0.0 },
    { -0.03730442,  0.25096530,  0.0 },
    { -1.0,         0.0,         0.99 }
};

const double Model_75C::Be_[Ne] = {
    -0.00048433,
     0.68629151,
     0.0
};

const double Model_75C::Ce_[Ne] = {
    1.0,
    0.0,
    0.0
};

const double Model_75C::Ee_[Ne] = {
     0.03502099,
    -48.36787164,
     0.0
};

// =====================
// Kalman gain (Le = [Lx ; Li])
// =====================
const double Model_75C::Le_[Ne] = {
     65.61592461,
      0.04745992,
     -0.01250462
};

// =====================
// LQI gain (Ke = [Kx Ki])
// =====================
const double Model_75C::Ke_[Ne] = {
     0.59547261,
    -0.00427387,
     0.0
};

// =====================
// Model struct
// =====================
const LQGModel model75 = {
    kLoss_W_per_degC,
    Model_75C::Ae_,
    Model_75C::Be_,
    Model_75C::Ce_,
    Model_75C::Ee_,
    Model_75C::Le_,
    Model_75C::Ke_,
    Model_75C::alpha_
};