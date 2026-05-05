#pragma once

struct LQGModel {
  static constexpr int Ne = 3;

  const double kLoss_W_per_degC;
  
  const double (*Ae)[Ne];
  const double* Be;
  const double* Ce;
  const double* Ee;
  const double* Le;
  const double* Ke;
};