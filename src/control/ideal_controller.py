from .base import Controller

class IdealAccelerationController(Controller):
    # 전달된 가속도 명령을 그대로 미사일에 적용(지연X)
    def update(self, missile, acc_cmd, dt):
        missile.set_acceleration(acc_cmd)