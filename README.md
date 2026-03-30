
Where

x_k is the internal state describing the thermal dynamics  
u_k is the input, corresponding to heater power  
y_k is the measured temperature  
v_k represents measurement noise  

The model is obtained from experimental data rather than assumed.

---

## Control (LQG)

The controller is based on LQG.

An LQR formulation is used for state feedback, and a Kalman filter is used for state estimation.

This allows the controller to operate on an estimate of the internal state despite noisy measurements.

The focus has been on achieving stable behaviour on the real system rather than optimal performance in simulation.

---

## System Identification

The model is obtained through experiments on the physical system.

Step response and sinusoidal excitation are used to capture the dynamics.  
From this data, a state-space model is estimated and used directly for controller design.

A reasonable model is essential. Without it, the controller has no meaningful foundation.

---

## Measurement System

The system initially used an NTC thermistor. This approach was abandoned due to calibration difficulty and unreliable results.

### NTC

The NTC sensor is strongly nonlinear and requires at least three reliable calibration points consisting of temperature and resistance.

In practice, this introduced several problems:

The calibration process was sensitive to noise  
Small resistance errors resulted in large temperature errors  
It was difficult to obtain consistent and accurate reference points  

This led to significant deviations between measured and actual temperature.

---

### PT100

The sensor was replaced with a PT100.

The main reasons were:

Near linear behaviour  
Simpler calibration, essentially gain and offset  
More stable and predictable measurements  

This change significantly improved both identification and state estimation.

---

## Signal Conditioning

The PT100 measurement requires a proper analog front-end.

Noise reduction before the ADC is necessary to obtain a stable signal.  
The quality of this signal directly affects the performance of the state estimator.

---

## System Characteristics

The system exhibits slow dynamics and significant thermal inertia.

Measurements are affected by noise.

As a result, aggressive control is not appropriate. Stability and consistency are more important than fast response.

---

## Notes

The main difficulty in this project was not the controller design itself.

The critical parts were:

Obtaining reliable measurement data  
Constructing a model that reflects the real system  
Handling non-ideal behaviour in practice  

---

## Academic Context

This work is based on concepts from

System Identification  
Linear Control Design  

The objective has been to apply these methods to a real system rather than limiting the work to simulation.

---

## Result

A state-space model identified from experimental data  
An implemented LQG controller  
Stable temperature control on a physical system  

---

## Project Structure
