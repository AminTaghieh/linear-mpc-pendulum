import numpy as np

from mpc import LinearMPC
from pendulum import get_linear_model, C_meas
from estimator import KalmanFilter
from simulation import simulate
from plotting import plot_results, plot_comparison


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
sigma_v = 0.05      # measurement noise, rad
sigma_w = 0.05      # disturbance torque, Nm

R_kf = np.diag([sigma_v**2, sigma_v**2])

G = B.reshape(-1, 1)
Q_kf = (sigma_w**2) * (G @ G.T) + 1e-9*np.eye(2)

kf = KalmanFilter(
    A=A,
    B=B,
    C=C_meas,
    Q_kf=Q_kf,
    R_kf=R_kf,
    x0_hat=np.zeros(2),
    P0=np.diag([1.0, 1.0])
)

time, x, u, x_hat, y, y_noise = simulate(
    controller=controller,
    parameters=parameters,
    x0=x0,
    dt=dt,
    simulation_time=simulation_time,
    estimator=kf,
    sigma_v=sigma_v,
    sigma_w=sigma_w,
    seed=0
)

plot_results(
    time,
    x,
    u,
    u_max
)
err = x_hat[:-1] - x[:-1]
print("RMS theta error:", np.sqrt(np.mean(err[:,0]**2)))
print("RMS omega error:", np.sqrt(np.mean(err[:,1]**2)))
time2, x2, u2, x_hat2, y2, y_noise2 = simulate(
    controller=controller,
    parameters=parameters,
    x0=x0,
    dt=dt,
    simulation_time=simulation_time,
    estimator=None,
    sigma_v=sigma_v,
    sigma_w=sigma_w,
    seed=0
)

print("\n--- with Kalman filter ---")
print("RMS theta:", np.sqrt(np.mean(x[:,0]**2)))
print("RMS u    :", np.sqrt(np.mean(u**2)))
print("RMS du   :", np.sqrt(np.mean(np.diff(u)**2)))

print("\n--- no filter ---")
print("RMS theta:", np.sqrt(np.mean(x2[:,0]**2)))
print("RMS u    :", np.sqrt(np.mean(u2**2)))
print("RMS du   :", np.sqrt(np.mean(np.diff(u2)**2)))

idx = time > 3.0
print("\nsteady-state RMS theta (KF) :", np.sqrt(np.mean(x[idx,0]**2)))
print("steady-state RMS theta (raw):", np.sqrt(np.mean(x2[idx,0]**2)))

plot_comparison(time, x, u, x2, u2, u_max)