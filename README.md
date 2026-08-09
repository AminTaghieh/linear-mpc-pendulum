# Linear MPC for Pendulum Control

A compact Python implementation of Linear Model Predictive Control (MPC) for pendulum stabilization.

The nonlinear pendulum is simulated in closed loop, while the controller uses a linearized discrete-time model to predict the system evolution and solve a constrained finite-horizon optimization problem.

## Features

* Nonlinear pendulum simulation
* Automatic linearization and discretization
* Linear MPC with quadratic cost
* Input constraints
* Terminal cost
* Closed-loop state regulation
* Visualization of angle, angular velocity, and control torque

## Run

Install the dependencies:

```bash id="install-deps"
pip install -r requirements.txt
```

Run the simulation:

```bash id="run-sim"
python run.py
```

## Project Structure

```text
pendulum.py    # dynamics, linearization, discretization
mpc.py         # MPC controller
simulation.py  # closed-loop simulation
plotting.py    # plotting utilities
run.py         # main example
```
