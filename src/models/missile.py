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

        # 중력/공력 추가를 위한 물리 파라미터
        self.mass = 100.0
        self.Cd = 0.5
        self.A = 0.03
        self.rho = 1.225
        self.g = np.array([0, 0, -9.81])

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

        # -----------------
        # PN acceleration
        # -----------------
        a_pn = self.acc_cmd

        # -----------------
        # Gravity(중력)
        # -----------------
        a_gravity = self.g

        # -----------------
        # Drag(공)
        # -----------------
        v = np.array([vx, vy, vz])

        v_norm = np.linalg.norm(v)

        if v_norm > 1e-6:
            drag = -0.5 * self.rho * self.Cd * self.A * v_norm * v / self.mass
        else:
            drag = np.zeros(3)

        # -----------------
        # Total acceleration
        # -----------------
        a_total = a_pn + a_gravity + drag

        ax, ay, az = a_total

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