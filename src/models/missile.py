import numpy as np
from .base import DynamicModel

# 미사일 동역학 시간 전파 역할
class Missile2D(DynamicModel):
    # 미사일 동역학 시간 전파 역할
    def __init__(self, position, velocity):
        self.state = np.array([
            position[0], position[1],
            velocity[0], velocity[1]
        ], dtype=float)

        self.acc_cmd = np.zeros(2)

    @property
    def pos(self):
        return self.state[0:2]

    @property
    def vel(self):
        return self.state[2:4]

    # Guidance/Controller에서 계산된 가속도 명령 입력/저장
    def set_acceleration(self, acc):
        self.acc_cmd = np.array(acc)

    def derivative(self, state):
        x, y, vx, vy = state
        ax, ay = self.acc_cmd

        return np.array([
            vx,
            vy,
            ax,
            ay
        ])


    # RK4 적분을 통한 위치/속도 업데이트
    def step(self, dt):
        s = self.state

        k1 = self.derivative(s)
        k2 = self.derivative(s + 0.5 * dt * k1)
        k3 = self.derivative(s + 0.5 * dt * k2)
        k4 = self.derivative(s + dt * k3)

        self.state = s + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)