import numpy as np

from pendulum import get_linear_model
from mpc import LinearMPC
from simulation import simulate
from plotting import plot_results



parameters = {
    "m": 1.0,
    "l": 1.0,
    "b": 0.15,
    "g": 9.81,
}

dt = 0.01
simulation_time = 10.0

x0 = np.array([
    np.deg2rad(60),
    0.0
])



A, B, _, _ = get_linear_model(
    parameters,
    dt
)

print("Discrete A:")
print(A)

print("\nDiscrete B:")
print(B)

Q = np.diag([
    20,   # theta penalty
    1     # omega penalty
])

R = np.array([
    [0.1]
])

horizon = 100

u_max = 5.0
controller = LinearMPC(
    A=A,
    B=B,
    Q=Q,
    R=R,
    horizon=horizon,
    u_max=u_max
)
time, state, input_signal = simulate(
    controller=controller,
    parameters=parameters,
    x0=x0,
    dt=dt,
    simulation_time=simulation_time
)
plot_results(
    time,
    state,
    input_signal,
    u_max
)
