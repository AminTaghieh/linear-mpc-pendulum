import numpy as np
import sympy as sp
from scipy.signal import cont2discrete

def dynamics(t,x,u, parameters,w=0):
    m = parameters["m"]
    l = parameters["l"]
    b = parameters["b"]
    g = parameters["g"]

    theta ,omega = x
    theta_dot = omega

    omega_dot = (
        -(g/l) * np.sin(theta)
        -(b/ (m*l**2)) * omega
        + (u+w)/ (m*l**2)
    )
    return np.array([
        theta_dot,
        omega_dot
    ])

def rk4_step(t,x,u,dt,parameters,w=0):

    k1=dynamics(t,x,u,parameters,w)

    k2=dynamics(
        t + dt/2,
        x+dt*k1/2,
        u,
        parameters,
        w,
    )
    k3=dynamics(
        t+dt/2,
        x+dt*k2/2,
        u,
        parameters,
        w,
    )
    k4=dynamics(
        t+dt,
        x+dt*k3,
        u,
        parameters,
        w,
    )
    return x+dt*(
        k1 + 2*k2 + 2*k3 + k4  
    ) /6

def get_linear_model(parameters,dt):

    theta, omega, u = sp.symbols(
        "theta omega u"
    )

    m,l,b,g = sp.symbols(
        "m l b g"
    ) 


    f = sp.Matrix([
        omega,

        -(g/l)*sp.sin(theta)
        -(b/ (m*l**2))*omega
        + u / (m*l**2)
    ]) 

    x=sp.Matrix([
        theta,
        omega,
    ])

    A_sym = f.jacobian(x)

    B_sym = f.jacobian(sp.Matrix([u]))

    equilibrium = {
        theta: 0,
        omega: 0,
        u: 0,

        m: parameters["m"],
        l: parameters["l"],
        b: parameters["b"],
        g: parameters["g"],
    }

    A_c = np.array(
        A_sym.subs(equilibrium),
        dtype=float
    )
    B_c = np.array(
        B_sym.subs(equilibrium),
        dtype=float
    )

    C_c = np.eye(2)
    D_c =np.zeros((2,1))

    A,B,_,_,_ = cont2discrete(
        (A_c, B_c, C_c, D_c),
        dt,
        method = "zoh"
    )

    return A,B, A_c, B_c


C_meas = np.eye(2)

def measure(x, sigma_v=0, rng=None):
    """Noisy full-state measurement. Returns shape (2,)."""
    y = C_meas @ np.asarray(x)
    if sigma_v > 0:
        if rng is None:
            raise ValueError("rng required when sigma_v > 0")
        y = y + rng.normal(0, sigma_v, size=2)
    return y

