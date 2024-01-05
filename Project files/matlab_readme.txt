# Sliding Mode Control based on Backstepping Approach for an UAV Type-Quadrotor

## Overview

This MATLAB code simulates the dynamics and control of a quadrotor using a Sliding mode controller with backstepping approach. The quadrotor's behavior is modeled based on dynamic equations, and the controller is applied to synthesize control laws for trajectory tracking.

## Files

1. QRBS.m: MATLAB function containing the quadrotor dynamics and QRBS controller.

2. run_file.m: MATLAB script to run the simulation and plot the results.

3. R_P_Y_traj.m: MATLAB script that plots the trajectory in each individual rotational axis and their corresponding tracking error.

4. x_y_z_traj.m: MATLAB script that plots the trajectory in each individual translational axis and their corresponding tracking error.


## Usage

1. Open all the above mentioned files in MATLAB.

2. Run the run_file.m script to get 3D plots of the trajectory. Run the R_P_Y_traj.m and x_y_z_traj.m to get 2D plots of the trajectory in ech individual axis.

## Variables in the Code

1. QRBS.m:

    - `m`, `d`, `ob`, `beta0`, `beta1`, `beta2`, `b`, `g`: Parameters defining the quadrotor and environmental constants.

    - `Ix`, `Iy`, `Iz`, `Jr`: Moments of inertia and rotor inertia.

    - `Kfax`, `Kfay`, `Kfaz`, `Kftx`, `Kfty`, `Kftz`: Aerodynamic and thrust constants.

    - `a1` to `a11`, `b1` to `b3`: Coefficients used in the dynamics equations.

    - `xd`: Desired trajectory vector.

    - `z`: Tracking error vector.

    - `alpha`: Tuning parameters for the controller.

    - `xdd`: Derivative of desired trajectory.

    - Controller gains `q1` to `k6`.

    - Calculation of control inputs `U1` to `Uy`.

    - Dynamics equations for the quadrotor.

    - Output derivatives `dx1` to `dx16`.



## Results

The simulation generates plots illustrating the quadrotor's trajectory and tracking performance compared to the desired trajectory.

Feel free to modify the parameters, initial conditions, or desired output duration based on your specific needs.

---

