import numpy as np
from models.measured_target import MeasuredTarget3D

class IdealSeekerWithNoise:
    def __init__(self, pos_noise_std=5.0, vel_noise_std=2.0):
        self.pos_noise_std = pos_noise_std
        self.vel_noise_std = vel_noise_std

    def measure_target(self, target):
        measured_pos = target.pos + np.random.normal(0, self.pos_noise_std, 3)
        measured_vel = target.vel + np.random.normal(0, self.vel_noise_std, 3)

        return MeasuredTarget3D(measured_pos, measured_vel)

class PositionSeekerWithNoise:
    def __init__(self, pos_noise_std=5.0):
        self.pos_noise_std = pos_noise_std

    def measure_target(self, target):
        measured_pos = target.pos + np.random.normal(0, self.pos_noise_std, 3)
        
        # 직접 속도 측정X(속도는 EKF가 추정): measured_target 인터페이스는 유지
        measured_vel = np.zeros(3)

        return MeasuredTarget3D(measured_pos, measured_vel)