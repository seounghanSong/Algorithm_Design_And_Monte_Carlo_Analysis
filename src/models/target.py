import numpy as np
from .base import DynamicModel

# 표적 동역학 전파 역할
class Target2D(DynamicModel):
    # 표적의 초기 위치/속도 초기화
    def __init__(self, position, velocity):
        self.pos = np.array(position, dtype=float)
        self.vel = np.array(velocity, dtype=float)

    # 등속 운동 모델을 가정한 위치 업데이트
    def step(self, dt):
        self.pos += self.vel * dt