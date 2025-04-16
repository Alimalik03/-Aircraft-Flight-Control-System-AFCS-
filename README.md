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

Compute the steady-level/ constant climb / co-ordinated turn flight condition using **"Main_Trim.mat"** script.
Run the Wrapper script to use fsolve in computing trim states
```matlab
run('Fsolve_Wrapper.m')
```
> This will display the trimmed pitch,trimmed bank, trimmed angle of attack etc and control inputs required to various trim manoeuvers.

---

### 2. **Run Full 6DOF Nonlinear Simulation**

Simulate a controlled flight using the nonlinear equations of motion and an modular controller:
```matlab
run('Initialisation.m')

```
>Initialise states and control inputs as a result of the wrapper file, to visualise time response
>Use **flight_controller.m** function in **initialisation** script to implement pitch, roll, speed controller
> Generates time histories of aircraft states.

---

###  3. **Linearize the Dynamics**

Linearize the nonlinear system about the trimmed condition to study behavior:
```matlab
run('Longitudinal_dynamics.m')
run('Lateral_dynamics.m')
```
> Visualizes the step response of the longitudinal and lateral dynamics (e.g., pitch angle, pitch rate, roll angle, roll rate, yaw angle, yaw rate).

---


##  Outputs
- Plot 1
- Steady Climb
- Time histories of key states (pitch, roll, yaw, velocities)
- Control inputs (elevator, aileron, rudder, throttle)
![States Vs Time]('./Aircraft%States.png')

- Plot 2
- 3D trajectory visualization
![Aircraft Trajectory]('./3D%Aircraft%Trajectory.png')
---
