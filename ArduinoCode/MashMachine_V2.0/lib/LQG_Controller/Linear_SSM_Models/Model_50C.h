#pragma once
#include "LQGModel.h"

struct Model_50C {
  static constexpr int Ne = 3;

  static const double kLoss_W_per_degC;
  
  static const double Ae_[Ne][Ne];
  static const double Be_[Ne];
  static const double Ce_[Ne];
  static const double Ee_[Ne];
  static const double Le_[Ne];
  static const double Ke_[Ne];
};

extern const LQGModel model50;