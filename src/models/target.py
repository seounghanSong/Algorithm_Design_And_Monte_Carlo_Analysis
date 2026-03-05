import numpy as np
from .base import DynamicModel

# 표적 동역학 전파 역할
class Target2D(DynamicModel):
    # 표적의 초기 위치/속도 초기화
    def __init__(self, position, velocity):
        self.state = np.array([
            position[0], position[1],
            velocity[0], velocity[1]
        ], dtype=float)

    @property
    def pos(self):
        return self.state[0:2]

    @property
    def vel(self):
        return self.state[2:4]

    # 등속 운동 모델을 가정한 위치 업데이트
    def derivative(self, state):
        x, y, vx, vy = state
        return np.array([
            vx,
            vy,
            0.0,
            0.0
        ])

    def step(self, dt):
        s = self.state

        k1 = self.derivative(s)
        k2 = self.derivative(s + 0.5 * dt * k1)
        k3 = self.derivative(s + 0.5 * dt * k2)
        k4 = self.derivative(s + dt * k3)

        self.state = s + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)