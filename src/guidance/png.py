import numpy as np
from .base import GuidanceLaw

# 유도법칙 계산
class ProportionalNavigation3D(GuidanceLaw):
    # navigation_gain 초기화
    def __init__(self, navigation_gain, max_acc=None):
        self.N = navigation_gain
        self.max_acc = max_acc

    # 타겟의 위치/속도에 기반한 가속도 벡터 생성
    def compute_command(self, missile, target):
        rel_pos = target.pos - missile.pos
        rel_vel = target.vel - missile.vel

        # 거리(r_norm) 계산
        r_norm = np.linalg.norm(rel_pos)

        if r_norm < 1e-6:
            return np.zeros(3)

        # 상대 속도 계산
        closing_vel = -np.dot(rel_pos, rel_vel) / r_norm

        # 멀어지는 경우 차단
        if closing_vel <= 0:
            return np.zeros(3)

        # ProportionalNavigation 공식 적용
        omega = np.cross(rel_pos, rel_vel) / (r_norm**2)

        # PN acceleration
        a_cmd = self.N * closing_vel * np.cross(omega, rel_pos / r_norm)

        # 횡가속도만 적용(종가속도 제거)
        v_norm = np.linalg.norm(missile.vel)
        
        if v_norm > 1e-6:
            v_hat = missile.vel / v_norm
            a_cmd = a_cmd - np.dot(a_cmd, v_hat) * v_hat

        # 가속도 포화
        if self.max_acc is not None:
            a_norm = np.linalg.norm(a_cmd)
            if a_norm > self.max_acc:
                a_cmd = a_cmd * (self.max_acc / a_norm)

        return a_cmd