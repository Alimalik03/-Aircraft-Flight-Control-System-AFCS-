# -Aircraft-Flight-Control-System-AFCS-
This MATLAB-based repository implements a 6-DOF nonlinear simulation and control system for a fixed-wing aircraft. The system includes trim calculation, full dynamics simulation, linearization, and a state-space controller for pitch, roll, yaw, and airspeed.

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
