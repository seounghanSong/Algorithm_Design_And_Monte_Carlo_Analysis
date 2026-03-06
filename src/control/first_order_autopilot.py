import numpy as np
from .base import Controller


class FirstOrderAutopilot(Controller):
    def __init__(self, tau=0.1):
        # 시간 상수
        self.tau = tau

        # 실제 기체의 가속도
        self.acc = None

    def update(self, missile, acc_cmd, dt):
        if self.acc is None:
            self.acc = np.zeros_like(acc_cmd)

        acc_dot = (acc_cmd - self.acc) / self.tau

        self.acc = self.acc + acc_dot * dt

        missile.set_acceleration(self.acc)