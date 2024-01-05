# Quadrotor Dynamics and Control Optimization

## Overview

This Python script simulates the dynamics and optimizes the control parameters of a quadrotor using a sliding mode controller. The optimization aims to minimize the tracking error between the obtained and desired trajectories.

## Dependencies

Ensure you have the following Python libraries installed:

- `numpy`
- `scipy`
- `matplotlib`

Install the dependencies using the following:

```bash
pip install numpy scipy matplotlib
```

## Files

1. Controls_Optimization.py: Python script containing the quadrotor dynamics, optimization, and simulation.

## Usage

1. Open Controls_Optimization.py in your preferred Python environment.

2. Run the script. This will optimize the control parameters and generate plots for the obtained trajectory.

## Functions

### QRBS Function

- **Inputs:**
    - `t`: Time
    - `x`: State vector
    - `alpha`: Tuning parameter
    - `q_values`: Control gains for q
    - `k_values`: Control gains for k

- **Outputs:**
    - Returns the derivatives of the state vector based on the quadrotor dynamics.

### Objective Function

- **Inputs:**
    - `params`: Optimization parameters (alpha, q, k)

- **Outputs:**
    - Returns the tracking error for the given parameters.

## Optimization

The script utilizes the `minimize` function from `scipy.optimize` to find the optimal values for `alpha`, `q`, and `k` that minimize the tracking error.

## Results

The script prints the best parameters and corresponding error after optimization. 

Feel free to modify the script, initial parameters, or simulation duration based on your specific needs.

---

