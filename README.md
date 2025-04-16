# -Aircraft-Flight-Control-System-AFCS-
This MATLAB-based repository implements a 6-DOF nonlinear simulation and control system for a fixed-wing aircraft. The system includes  Equations of motion Model, Trimming, Linearization, and a Modular controller for pitch, roll, and airspeed employing state feedback.

File Descriptions

| File                   | Description |
|------------------------|-------------|
| `Initialisation.m`     | Main simulation script that runs a closed-loop controlled flight using nonlinear dynamics. |
| `flight_controller.m`  | Modular AFCS controller computing elevator, aileron, and throttle commands. |
| `SixDof_eqm.m`         | Defines the full 6-degree-of-freedom nonlinear aircraft model. |
| `Main_Trim.m`          | Function to evaluate steady-state trim residuals. |
| `Fsolve_Wrapper.m`     | Script that calls `fsolve` to find trim conditions using `Main_Trim.m`. |
| `linearization.m`      | Function that computes linear A, B, C, D matrices from the nonlinear model. |
| `Longitudinal_dynamics.m` | Script to analyze longitudinal dynamics with step response. |
| `Lateral_dynamics.m` | Script to analyze lateral dynamics with step response. |


##  Requirements
- MATLAB R2020+ (tested)
- Optimization Toolbox (for `fsolve`)
- Control System Toolbox (for `ss`, `lqr`, `lsim`)


##  How to Run

Follow these steps in MATLAB to simulate and analyze the aircraft system:

###  1. **Trim the Aircraft**

Compute the steady-level/ constant climb / co-ordinated turn flight condition using nonlinear equations:
```matlab
run('Fsolve_Wrapper.m')
```
> This will display the trimmed pitch, angle of attack, and control inputs required to maintain steady flight.

---

### 2. **Run Full 6DOF Nonlinear Simulation**

Simulate a controlled flight using the nonlinear equations of motion and an autopilot controller:
```matlab
run('Initialisation.m')
```
> Generates time histories of aircraft states and a 3D trajectory plot under closed-loop control.

---

###  3. **Linearize the Dynamics**

Linearize the nonlinear system about the trimmed condition to study its small signal behavior:
```matlab
run('Longitudinal_dynamics.m')
```
> Visualizes the step response of the longitudinal model (e.g., pitch angle, pitch rate).

---


##  Outputs

- Time histories of key states (pitch, roll, yaw, velocities)
- Control inputs (elevator, aileron, rudder, throttle)
- Trim conditions
- 3D trajectory visualization

---
