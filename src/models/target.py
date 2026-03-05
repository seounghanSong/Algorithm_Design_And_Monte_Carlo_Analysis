import numpy as np
from .base import DynamicModel

# 표적 동역학 전파 역할
class Target3D(DynamicModel):
    # 표적의 초기 위치/속도 초기화
    def __init__(self, position, velocity):
        # 상태: [x, y, z, vx, vy, vz]
        self.state = np.array([
            position[0], position[1], position[2],
            velocity[0], velocity[1], velocity[2]
        ], dtype=float)

    @property
    def pos(self):
        return self.state[0:3]

    @property
    def vel(self):
        return self.state[3:6]

    # 등속 운동: 가속도 = 0
    def derivative(self, state):
        x, y, z, vx, vy, vz = state
        return np.array([
            vx, vy, vz,
            0.0, 0.0, 0.0
        ])

    # RK4 적분
    def step(self, dt):
        s = self.state

        k1 = self.derivative(s)
        k2 = self.derivative(s + 0.5 * dt * k1)
        k3 = self.derivative(s + 0.5 * dt * k2)
        k4 = self.derivative(s + dt * k3)

        self.state = s + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)