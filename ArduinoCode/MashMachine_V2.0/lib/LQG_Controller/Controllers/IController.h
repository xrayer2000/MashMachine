#pragma once

class IController {
public:
    virtual ~IController() {}

    virtual void Init(double Q0_W) = 0;
    virtual bool Compute() = 0;
    virtual double GetCurrentPower_W() const = 0;
};