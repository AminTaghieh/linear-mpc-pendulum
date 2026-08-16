import numpy as np

from pendulum import rk4_step, measure

def simulate(
        controller,
        parameters,
        x0,
        dt,
        simulation_time,
        estimator=None,
        sigma_v=0,
        sigma_w=0,
        seed=0
):

    rng = np.random.default_rng(seed)

    time = np.arange(0, simulation_time+dt, dt)
    N = len(time)

    x = np.zeros((N, 2))
    x_hat = np.zeros((N, 2))
    y = np.zeros((N-1, 2))
    y_noise = np.zeros((N-1, 2))
    u = np.zeros(N-1)

    x[0] = x0

    for k in range(N-1):

        y[k] = x[k]                             # true state
        y_k = measure(x[k], sigma_v, rng)       # noisy measurement
        y_noise[k] = y_k

        if estimator is None:
            x_hat[k] = y_k                      # raw measurement, no filter
        else:
            x_hat[k] = estimator.update(y_k)

        u[k] = controller.control(x_hat[k])

        if estimator is not None and hasattr(estimator, "predict"):
            estimator.predict(u[k])

        w = rng.normal(0, sigma_w) if sigma_w > 0 else 0

        x[k+1] = rk4_step(time[k], x[k], u[k], dt, parameters, w)

    return time, x, u, x_hat, y, y_noise