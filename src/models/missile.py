import numpy as np
from .base import DynamicModel

# 미사일 동역학 시간 전파 역할
class Missile3D(DynamicModel):
    # 미사일의 초기 위치/속도 초기화
    def __init__(self, position, velocity):
        self.state = np.array([
            position[0], position[1], position[2],
            velocity[0], velocity[1], velocity[2]
        ], dtype=float)

        self.acc_cmd = np.zeros(3)

    @property
    def pos(self):
        return self.state[0:3]

    @property
    def vel(self):
        return self.state[3:6]

    # Guidance/Controller에서 계산된 가속도 명령 입력/저장
    def set_acceleration(self, acc):
        self.acc_cmd = np.array(acc)

    def derivative(self, state):
        x, y, z, vx, vy, vz = state
        ax, ay, az = self.acc_cmd

        return np.array([
            vx, vy, vz,
            ax, ay, az
        ])

    # RK4 적분
    def step(self, dt):
        s = self.state

        k1 = self.derivative(s)
        k2 = self.derivative(s + 0.5 * dt * k1)
        k3 = self.derivative(s + 0.5 * dt * k2)
        k4 = self.derivative(s + dt * k3)

        self.state = s + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)