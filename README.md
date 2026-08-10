# Linear MPC for Pendulum Regulation

Linear MPC for a damped pendulum. The plant is integrated as the full nonlinear system with RK4. The controller predicts with a linearized, ZOH-discretized model and solves a constrained QP at every sampling instant, applying only the first input.

## Model

The nonlinear pendulum dynamics are

$$
\dot{\theta} = \omega, \qquad \dot{\omega} = -\frac{g}{l}\sin\theta - \frac{b}{ml^{2}}\omega + \frac{u}{ml^{2}}
$$

The state is

$$
x =
\begin{bmatrix}
\theta \\
\omega
\end{bmatrix}
$$

where $u$ is the pivot torque and $b$ is the viscous friction coefficient.

The angle is measured from the downward equilibrium, so $\theta = 0$ is open-loop stable. This is a regulation problem, not inverted-pendulum stabilization.

With the default parameters, the continuous-time linearized open-loop poles are approximately $-0.075 \pm 3.131j$, corresponding to an oscillation frequency of roughly $0.5$ Hz and a damping envelope time constant of about $13$ s.

## Linearization

Jacobians are taken symbolically with SymPy at $(\theta, \omega, u) = (0, 0, 0)$. The continuous-time linear model is

$$
\dot{x} = A_c x + B_c u
$$

with

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

The model is then discretized with a zero-order hold using `scipy.signal.cont2discrete`, giving

$$
x_{k+1} = A_d x_k + B_d u_k
$$

Only $(A_d, B_d)$ are used by the MPC controller.

## Controller

At each sampling instant $k$, the MPC solves

$$
\min_{X, U} \quad \sum_{i=0}^{N-1} \left( x_{i|k}^{\top} Q x_{i|k} + u_{i|k}^{\top} R u_{i|k} \right) + x_{N|k}^{\top} P x_{N|k}
$$

subject to

$$
x_{0|k} = x(k), \qquad x_{i+1|k} = A_d x_{i|k} + B_d u_{i|k}, \qquad |u_{i|k}| \le u_{\max}
$$

Here, $x_{i|k}$ denotes the state predicted $i$ steps ahead using information available at time $k$.

The matrix $P$ is obtained from the discrete algebraic Riccati equation, so the terminal cost $x_{N|k}^{\top} P x_{N|k}$ is the value function of the corresponding unconstrained infinite-horizon LQR problem. There is no terminal constraint or terminal invariant set, so this implementation does not provide a separate formal closed-loop stability certificate. After solving the optimization problem, only the first optimal input $u_k = u_{0|k}^{\star}$ is applied to the nonlinear plant. At the next sampling instant, the plant state is obtained again and the optimization is repeated.

The QP is constructed once in CVXPY. The current state $x(k)$ enters the optimization through a `cp.Parameter` representing the initial predicted state $x_{0|k}$. At each sampling instant, the parameter value is updated with the current state and the same optimization problem is re-solved with OSQP using warm start.

## Defaults

| Parameter | Value |
| --- | --- |
| Mass $m$ | 1 kg |
| Length $l$ | 1 m |
| Friction $b$ | 0.15 |
| Gravity $g$ | 9.81 m/s² |
| Sampling time $\Delta t$ | 0.01 s |
| Horizon $N$ | 100 steps (1 s) |
| Torque limit $u_{\max}$ | 5 N·m |
| Simulation duration | 10 s |

The MPC weights are

$$
Q =
\begin{bmatrix}
20 & 0 \\
0 & 1
\end{bmatrix},
\qquad
R = 0.1
$$

The default initial condition is

$$
x_0 =
\begin{bmatrix}
60^{\circ} \\
0
\end{bmatrix}
$$

## Results

Starting from $\theta(0) = 60^{\circ}$, the closed loop reaches approximately $|\theta| < 1^{\circ}$ within about $1.5$ s, with one small undershoot. The uncontrolled pendulum remains visibly oscillatory at $10$ s. The torque limit is active during the initial part of the transient.
The main role of MPC in this example is therefore not to stabilize an unstable equilibrium, but to shape the transient while explicitly respecting the actuator constraint.

## Limits of Validity

The prediction model is linearized about $\theta = 0$, so its accuracy degrades as the pendulum moves farther from the equilibrium.
In simulation, the closed loop may still converge from large initial angles such as $170^{\circ}$, but this is far outside the validity region of the linearization and should be interpreted as an empirical closed-loop result rather than a theoretical guarantee. Far from the origin, the predicted trajectories can be inaccurate. However, MPC applies only the first optimized input and reinitializes the optimization from the current nonlinear plant state at every sampling instant. Therefore, model mismatch does not simply accumulate over the entire maneuver as it would under open-loop execution.
The implementation assumes full-state feedback: both $\theta$ and $\omega$ are known exactly from the simulator. There is currently no observer, measurement noise, external disturbance, or parameter mismatch.

## Run

```bash
pip install -r requirements.txt
python run.py
```
