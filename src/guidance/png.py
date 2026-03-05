import numpy as np
from .base import GuidanceLaw

# 유도법칙 계산
class ProportionalNavigation(GuidanceLaw):
    # navigation_gain 초기화
    def __init__(self, navigation_gain):
        self.N = navigation_gain

    # 타겟의 위치/속도에 기반한 가속도 벡터 생성
    def compute_command(self, missile, target):
        rel_pos = target.pos - missile.pos
        rel_vel = target.vel - missile.vel

        # 거리(r) 계산
        r = np.linalg.norm(rel_pos)
        if r < 1e-6:
            return np.zeros(2)

        # LOS(Line-Of-Sight) 각속도(λ) 계산
        lambda_dot = (rel_pos[0]*rel_vel[1] - rel_pos[1]*rel_vel[0]) / r**2

        # 상대 속도 계산
        closing_vel = -np.dot(rel_pos, rel_vel) / r

        # ProportionalNavigation 공식 적용
        a_mag = self.N * closing_vel * lambda_dot

        # 수직 방향의 단위 벡터 계산
        normal = np.array([-rel_pos[1], rel_pos[0]]) / r

        return a_mag * normal