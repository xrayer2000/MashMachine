# MashMachine

Where:

- x_k : system states (thermal dynamics)  
- u_k : input (heater power)  
- y_k : measured temperature  
- v_k : measurement noise  

---

## 🎯 Control Design — LQG

The control strategy is based on **LQG (Linear Quadratic Gaussian)**:

- LQR → optimal state feedback  
- Kalman filter → optimal state estimation  

This enables:

- Control under measurement noise  
- Use of internal state estimates  
- Systematic tuning via weighting matrices  

---

## 🌡️ Measurement System

### Transition: NTC → PT100

The system originally used an NTC thermistor, but this was replaced due to fundamental limitations.

### Issues with NTC

- Strong nonlinearity  
- Calibration required **three accurate reference points** (temperature + resistance)  
- Sensitive to noise  
- Error amplification through nonlinear mapping  

Observed in practice:

- Large deviation between measured and actual temperature  
- Time-consuming and unreliable calibration process  

---

### PT100 Implementation

The system was upgraded to a **PT100 RTD**, providing:

- Near-linear response  
- Higher accuracy  
- Improved stability  

Result:

- Simplified calibration (gain + offset)  
- Reliable measurements suitable for model-based control  

---

## 🔧 Signal Conditioning

- Analog front-end for PT100 measurement  
- Noise reduction before ADC  
- Stable input for state estimation  

---

## ⚠️ Hardware Reliability

Key lessons implemented:

- Common ground across all subsystems is mandatory  
- Series resistors (~220Ω) on control signals  
- Protection against voltage transients (diodes, bleeder resistors)  

Previous hardware failures were traced to grounding issues.

---

## 🧪 System Characteristics

- Slow thermal dynamics  
- Significant inertia  
- Measurement noise  

Implication:

> Control performance is fundamentally limited by system physics, not controller complexity.

---

## 🧠 Engineering Insight

The main challenge was not controller design, but:

- Obtaining a reliable model  
- Ensuring measurement quality  
- Handling real-world non-idealities  

---

## 🎓 Academic Context

This project is a direct implementation of:

- System Identification  
- Linear Control Design  

on real hardware, bridging theory and practice:

- Identified model from experimental data  
- Designed optimal controller  
- Deployed on embedded system  

---

## 🚀 Outcome

- Validated state-space model of thermal system  
- Working LQG controller  
- Stable real-world temperature regulation  
- End-to-end model-based control pipeline  

---

## 📁 Project Structure


# License

This project is licensed under the  
**Creative Commons Attribution-NonCommercial 4.0 International (CC BY-NC 4.0)** license.

You are free to:
- **Share** — copy and redistribute the material in any medium or format
- **Adapt** — remix, transform, and build upon the material

Under the following terms:
- **Attribution** — You must give appropriate credit, provide a link to the license, and indicate if changes were made.
- **NonCommercial** — You may not use the material for commercial purposes.

## Full License Text
The full legal code is available at:  
[https://creativecommons.org/licenses/by-nc/4.0/legalcode](https://creativecommons.org/licenses/by-nc/4.0/legalcode)

## Human-Readable Summary
A simple summary of the license terms:  
[https://creativecommons.org/licenses/by-nc/4.0/](https://creativecommons.org/licenses/by-nc/4.0/)
