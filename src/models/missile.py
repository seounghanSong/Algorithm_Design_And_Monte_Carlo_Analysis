import numpy as np
from .base import DynamicModel

# 미사일 동역학 시간 전파 역할
class Missile2D(DynamicModel):
    # 미사일의 초기 위치/속도 설정 및 가속도 명령 초기화
    def __init__(self, position, velocity):
        self.pos = np.array(position, dtype=float)
        self.vel = np.array(velocity, dtype=float)
        self.acc_cmd = np.zeros(2)

    # Guidance/Controller에서 계산된 가속도 명령 입력/저장
    def set_acceleration(self, acc):
        self.acc_cmd = np.array(acc)

    # 오일러 적분을 통한 위치/속도 업데이트
    def step(self, dt):
        self.vel += self.acc_cmd * dt
        self.pos += self.vel * dt