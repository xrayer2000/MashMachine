#include "Model_75C.h"

// =====================
// Dimensions
// Ne = 2 states + 1 integrator
// =====================

static const double kLoss_W_per_degC = 4.09; //75C

const double Model_75C::Ae_[Ne][Ne] = {
    { 0.0,    -0.68362,  0.0  },
    { 1.0,     1.6836,   0.0  },
    { 0.0,    -1.0,      0.99 }
};

const double Model_75C::Be_[Ne] = {
    -0.016047,
     0.016047,
     0.0
};

const double Model_75C::Ce_[Ne] = {
    0.0,
    1.0,
    0.0
};

const double Model_75C::Ee_[Ne] = {
    0.0,
    0.0,
    1.0
};

// =====================
// Kalman gain (Le = [Lx ; Li])
// =====================
const double Model_75C::Le_[Ne] = {
     26.272,
     26.381,
    -0.033676
};

// =====================
// LQI gain (Ke = [Kx Ki])
// =====================
const double Model_75C::Ke_[Ne] = {
    -0.1527,
     0.27326,
    -0.71501
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
    Model_75C::Ke_
};
