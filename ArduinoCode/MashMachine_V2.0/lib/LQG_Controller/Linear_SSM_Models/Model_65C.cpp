#include "Model_65C.h"

// =====================
// Dimensions
// Ne = 2 states + 1 integrator
// =====================

static const double kLoss_W_per_degC = 3.6; //65C

const double Model_65C::Ae_[Ne][Ne] = {
    { 0.0,    -0.41717,  0.0  },
    { 0.5,     1.2086,   0.0  },
    { 0.0,    -1.0,      0.99 }
};

const double Model_65C::Be_[Ne] = {
    -0.010801,
     0.0054014,
     0.0
};

const double Model_65C::Ce_[Ne] = {
    0.0,
    1.0,
    0.0
};

const double Model_65C::Ee_[Ne] = {
    0.0,
    0.0,
    1.0
};

// =====================
// Kalman gain (Le = [Lx ; Li])
// =====================
const double Model_65C::Le_[Ne] = {
     6.7154,
    13.437,
    -0.0050932
};

// =====================
// LQI gain (Ke = [Kx Ki])
// =====================
const double Model_65C::Ke_[Ne] = {
    -0.047691,
     0.12823,
    -0.81792
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
    Model_65C::Ke_
};


// const double Model_65C::Ae_[Ne][Ne] = {
//  { 0.0,  0.0, -0.56076,  0.0  },
//  { 0.5,  0.0, -0.31693,  0.0  },
//  { 0.0,  1.0,  1.5973 ,  0.0  },
//  { 0.0,  0.0, -1.0    ,  0.99 }
// };
// const double Model_65C::Be_[Ne] = {
//   0.0058291,
//  -0.0061976,
//   0.0032852,
//   0.0
// };
// const double Model_65C::Ce_[Ne] = {
//   0.0,
//   0.0,
//   1.0,
//   0.0
// };
// const double Model_65C::Ee_[Ne] = {
//   0.0, 0.0, 0.0, 1.0
// };
// const double Model_65C::Le_[Ne] = {
// 45.761, 91.548, 91.574, -0.00011271
//    -0.0033224
// };
// const double Model_65C::Ke_[Ne] = {
//  -0.18379,
//  -0.14689,
//   0.57681,
//  -0.42579
// };
// const LQGModel model65 = {
//   Model_65C::Ae_,
//   Model_65C::Be_,
//   Model_65C::Ce_,
//   Model_65C::Ee_,
//   Model_65C::Le_,
//   Model_65C::Ke_
// };