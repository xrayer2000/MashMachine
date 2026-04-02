#include "Excitation.h"
#include <Arduino.h>
#include <math.h>

Excitation::Excitation()
{
    type     = SINE;
    Qbias_W  = 0.0;
    A_W      = 0.0;
    omega    = 0.0;
    Qstep_W  = 0.0;
    t0_ms    = 0;
    running  = false;
}

void Excitation::setType(Type t)
{
    type = t;
}

void Excitation::setSine(double bias_W,
                         double amplitude_W,
                         double period_s)
{
    Qbias_W = bias_W;
    A_W     = amplitude_W;

    if (period_s > 0.0)
        omega = 2.0 * M_PI / period_s;
    else
        omega = 0.0;
}

void Excitation::setStep(double step_W)
{
    Qstep_W = step_W;
}

void Excitation::start()
{
    t0_ms = millis();
    running = true;
}

void Excitation::stop()
{
    running = false;
}

double Excitation::compute()
{
    if (!running)
        return 0.0;

    double t = (millis() - t0_ms) * 0.001;

    switch(type)
    {
        case SINE:
            return Qbias_W + A_W * sin(omega * t);

        case STEP:
            return Qstep_W;

        default:
            return 0.0;
    }
}