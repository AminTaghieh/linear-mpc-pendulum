import numpy as np

from pendulum import rk4_step

def simulate(
        controller,
        parameters,
        x0,
        dt,
        simulation_time
):


    time=np.arange(
        0,
        simulation_time+dt,
        dt
    )


    number_of_steps = len(time)

    state = np.zeros(
        (number_of_steps, 2)
    )

    input_signal = np.zeros(
        number_of_steps-1
    )

    state[0] =x0



    for k in range(number_of_steps-1):
        x = state[k]

        u = controller.control(x)

        input_signal[k] =u

        state[k+1] = rk4_step(
            time[k],
            x,
            u,
            dt,
            parameters
        )
    return time, state, input_signal