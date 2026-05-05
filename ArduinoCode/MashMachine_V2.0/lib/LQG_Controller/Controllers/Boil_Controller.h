#pragma once
#include "IController.h"
#include "../Linear_SSM_Models/Model_Boil.h"

class BoilController : public IController {
public:
    BoilController(const BoilModel& model) : model_(model) {}

    void Init(double currentPower) override {}
    void setInitialPower(double Q_init);
    bool Compute() override;

    double GetCurrentPower_W() const override { return Q_; }

    void setInputs(double T, double Tair, double Tset);

private:
    const BoilModel& model_;

    double T_ = 0.0;
    double Tair_ = 0.0;
    double Tset_ = 0.0;

    double Q_ = 0.0;

    double Q_offset_ = 0.0;  // For bumpless start
};