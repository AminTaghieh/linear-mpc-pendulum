import numpy as np


class KalmanFilter:

    def __init__(self, A, B, C, Q_kf, R_kf, x0_hat, P0):
        self.A = np.asarray(A, dtype=float)
        self.B = np.asarray(B, dtype=float)
        self.C = np.asarray(C, dtype=float)
        self.Q_kf = np.asarray(Q_kf, dtype=float)
        self.R_kf = np.asarray(R_kf, dtype=float)

        self.x_pred = np.asarray(x0_hat, dtype=float)
        self.P_pred = np.asarray(P0, dtype=float)

    def update(self, y):
        # Measurement update
        S = self.C @ self.P_pred @ self.C.T + self.R_kf
        K = self.P_pred @ self.C.T @ np.linalg.inv(S)

        innovation = np.atleast_1d(y) - self.C @ self.x_pred

        self.x_hat = self.x_pred + K @ innovation

        I = np.eye(self.A.shape[0])
        self.P_est = (I - K @ self.C) @ self.P_pred

        return self.x_hat

    def predict(self, u):
        # Time update
        self.x_pred = self.A @ self.x_hat + self.B @ np.atleast_1d(u)
        self.P_pred = self.A @ self.P_est @ self.A.T + self.Q_kf