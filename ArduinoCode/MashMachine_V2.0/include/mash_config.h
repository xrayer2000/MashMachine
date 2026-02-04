#pragma once
//Heat element
constexpr double heaterRatedPower_W = 2200.0;
constexpr int PWM_MAX = 4095;
constexpr double W_to_PWM = (double)PWM_MAX / heaterRatedPower_W;
constexpr double PWM_to_W = heaterRatedPower_W / (double)PWM_MAX;