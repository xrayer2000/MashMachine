#ifndef EXCITATION_H
#define EXCITATION_H

class Excitation
{
public:
    enum Type
    {
        SINE,
        STEP
    };

    Excitation();

    void setSine(double bias_W,
                 double amplitude_W,
                 double period_s);

    void setStep(double step_W);

    void setType(Type t);

    void start();
    void stop();

    double compute();   // returns power in watts

private:
    Type type;

    double Qbias_W;
    double A_W;
    double omega;       // rad/s
    double Qstep_W;

    unsigned long t0_ms;
    bool running;
};

#endif