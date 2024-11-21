from typing import Callable, Self
import numpy as np


class HelixonKalmanFilter:

    def __init__(self: Self, getA: Callable[[float], np.ndarray], getB: Callable[[float], np.ndarray], P: np.ndarray, Q: np.ndarray, R: np.ndarray, H: np.ndarray):
        self.getA = getA
        self.getB = getB
        self.P = P
        self.Q = Q
        self.R = R
        self.H = H
        self.xhat = np.zeros((P.shape[0], 1))

    def predict(self: Self, input: np.ndarray, dt: float):
        self.A = self.getA(dt)
        self.B = self.getB(dt)
        
        self.xhat = self.A @ self.xhat + self.B @ input
        self.P = self.A @ self.P @ self.A.T + self.Q

    def update(self: Self, y: np.ndarray):
        self.K = self.P @ self.H.T @ np.linalg.inv((self.H @ self.P @ self.H.T) + self.R)
        
        self.xhat += self.K @ (y - (self.H @ self.xhat))
        self.P = (np.identity(len(self.xhat)) - (self.K @ self.H)) @ self.P

    def run_offlne(self: Self, us: np.ndarray, ys: np.ndarray, ts: np.ndarray) -> np.ndarray:
        N = len(us)
        states = np.zeros((N, len(self.xhat)))
        for i in range(1, N):
            dt = (ts[i]-ts[i-1])
            self.predict(us[i], dt)
            self.update(ys[i])
            states[i] = self.xhat.flatten()
            print(states[i])
        return states
    
    def run_step(self: Self, u: np.ndarray, y: np.ndarray, dt: float) -> np.ndarray:
        self.predict(u, dt)
        self.update(y)
        state = self.xhat.flatten()
        return state
