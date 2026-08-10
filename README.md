# Linear MPC for Pendulum Regulation

Linear MPC for a damped pendulum. The plant is integrated as the full nonlinear system with RK4. The controller predicts with a linearized, ZOH-discretized model and solves a constrained QP at every sampling instant, applying only the first input.

## Model

$$
\dot{\theta} = \omega,
\qquad
\dot{\omega} = -\frac{g}{l}\sin\theta - \frac{b}{ml^{2}}\omega + \frac{u}{ml^{2}}
$$

State $x = [\theta, \omega]^{\top}$, input $u$ the pivot torque, $b$ viscous friction.

The angle is measured from the downward equilibrium, so $\theta = 0$ is open-loop stable. This is a regulation problem, not inverted-pendulum stabilization. With the default parameters the continuous-time linearized open-loop poles are $-0.075 \pm 3.131j$: roughly 0.5 Hz with a 13 s envelope time constant.

## Linearization

Jacobians are taken symbolically with SymPy at $(\theta, \omega, u) = (0,0,0)$:

$$
A_c =
\begin{bmatrix}
0 & 1 \\
-\dfrac{g}{l} & -\dfrac{b}{ml^{2}}
\end{bmatrix},
\qquad
B_c =
\begin{bmatrix}
0 \\
\dfrac{1}{ml^{2}}
\end{bmatrix}
$$

then discretized with a zero-order hold via `scipy.signal.cont2discrete`. Only $(A_d, B_d)$ reach the controller.

## Controller

$$
\min_{x,u} \quad \sum_{i=0}^{N-1} \left( x_i^{\top} Q x_i + u_i^{\top} R u_i \right) + x_N^{\top} P x_N
$$

subject to $x_0 = x(k)$, $x_{i+1} = A_d x_i + B_d u_i$, and $|u_i| \le u_{\max}$.

$P$ solves the discrete algebraic Riccati equation, so the terminal cost is the infinite-horizon unconstrained LQR cost-to-go. There is no terminal set, so the formulation carries no formal stability certificate.

The QP is built once in CVXPY with $x_0$ as a `Parameter` and re-solved with OSQP under warm start.

## Defaults

$m = 1$ kg, $l = 1$ m, $b = 0.15$, $g = 9.81$, $\Delta t = 0.01$ s, $N = 100$, $Q = \mathrm{diag}(20, 1)$, $R = 0.1$, $u_{\max} = 5$ N·m, $x_0 = [60^{\circ}, 0]$, 10 s simulation.

## Results

From $60^{\circ}$ the closed loop reaches $|\theta| < 1^{\circ}$ in 1.5 s with one small undershoot. The uncontrolled pendulum is still visibly oscillating at 10 s. The torque limit is active for the first 0.5 s.

## Limits of validity

The prediction model is linearized about $\theta = 0$, but the closed loop tolerates much more than the small-angle range and still converges from $170^{\circ}$. Only the first input is applied and it is recomputed from the measured state every 10 ms, so model error is corrected by feedback rather than accumulating. The predicted trajectories themselves are inaccurate far from the origin and are never relied on.

Full state feedback, no observer, no measurement noise, and controller parameters matched exactly to the plant.

## Run

```bash
pip install -r requirements.txt
python run.py
```

## Files

```
pendulum.py    # nonlinear dynamics, RK4, symbolic linearization, ZOH discretization
mpc.py         # QP construction, DARE terminal cost, receding-horizon solve
simulation.py  # closed-loop simulation against the nonlinear plant
plotting.py    # plots
run.py         # entry point
```
