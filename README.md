## Introduction - MashMachine
This project of mine has been a great journey for me to learn even more about my courses in the MPSYS program at Chalmers, Automation and Mechatronics. 

I have specifically used two courses in practice: SSY230 (System Identification) and SSY285 (Design of Linear Control Systems).

In the System Identification course, I used my mash tun to collect both estimation and validation data.

I used the System Identification Toolbox in MATLAB to derive state-space models for three different operating points: 50°C, 65°C, and 70°C.

This means that all system matrices were computed in MATLAB, and then implemented directly on the ESP32. The computational work was therefore performed offline, rather than on the microcontroller.

Once the state-space model was obtained, I used my other course, Design of Linear Control Systems, to design a controller based on this model.

I first implemented a state-feedback LQR controller, and later extended it to an LQG controller by adding a Kalman filter for state estimation.

This requires a state-space model, since LQG relies on an explicit representation of the system dynamics. In contrast to PID control, which operates only on the control error, LQG uses the model to estimate internal states and compute the control input.

With that said, the LQG controller was somewhat excessive for this application, since the system exhibits slow and stable dynamics. A PID controller would likely have been sufficient.

## Home Assistant Integration

The system is integrated with Home Assistant for monitoring and interaction.

A local Linux machine is used as a small server, running Home Assistant in a Docker container. The same setup also runs supporting services such as InfluxDB and Grafana.

Communication between the ESP32 and Home Assistant is handled using MQTT.

This setup allows:

Real-time monitoring of temperature, control signals, and system states  
Logging of data to InfluxDB for further analysis  
Visualization of system behaviour in Grafana  
Remote interaction through the Home Assistant interface  

The purpose of this integration is not only convenience, but also to enable data collection for system identification and validation, as well as to observe the controller performance over time.

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
