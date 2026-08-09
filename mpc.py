import cvxpy as cp
from scipy.linalg import solve_discrete_are

class LinearMPC:

    def __init__(
        self,
        A,
        B,
        Q,
        R,
        horizon,
        u_max
    ):
        self.A =A
        self.B=B
        self.Q=Q
        self.R=R

        self.N = horizon
        self.u_max = u_max

        self.N = horizon
        self.u_max =u_max

        self.nx = A.shape[0]
        self.nu = B.shape[1]


        self.P= solve_discrete_are(
            A,
            B,
            Q,
            R
        )

        self._build_problem()


    def _build_problem(self):
        self.X = cp.Variable((self.nx, self.N + 1))
        self.U = cp.Variable((self.nu, self.N))
        self.x0 = cp.Parameter(self.nx)

        cost = 0
        constraints = []

        constraints.append(self.X[:, 0] == self.x0)
        for i in range(self.N):
            cost += cp.quad_form(self.X[:, i], self.Q)
            cost += cp.quad_form(self.U[:, i], self.R)
            constraints.append(
                self.X[:, i + 1]
                == self.A @ self.X[:, i] + self.B @ self.U[:, i]
            )
            constraints.append(self.U[:, i] <= self.u_max)
            constraints.append(self.U[:, i] >= -self.u_max)

        cost += cp.quad_form(self.X[:, self.N], self.P)

        self.problem = cp.Problem(cp.Minimize(cost), constraints)

    def control(self, x_current):
        self.x0.value = x_current
        self.problem.solve(solver=cp.OSQP, warm_start=True, verbose=False)

        if self.problem.status not in ["optimal", "optimal_inaccurate"]:
            raise RuntimeError(f"MPC failed: {self.problem.status}")

        return self.U.value[0, 0]
                
                    









