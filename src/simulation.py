import numpy as np

# 전체 시스템 통합을 위한 엔진
class Simulation:
    # 미사일, 표적, 유도기, 제어기, 설정 등의 객체 저장 및 궤적 기록용 history 딕셔너리 생성
    def __init__(self, missile, target, guidance, controller, config):
        self.missile = missile
        self.target = target
        self.guidance = guidance
        self.controller = controller
        self.config = config

        self.history = {
            "missile": [],
            "target": []
        }

    # 시뮬레이션 루프 실행
    def run(self):
        t = 0.0
        while t < self.config.total_time:
            # 유도 명령 계산
            acc_cmd = self.guidance.compute_command(self.missile, self.target)

            # Controller 업데이트
            self.controller.update(self.missile, acc_cmd, self.config.dt)

            # 미사일 동역학 전파
            self.missile.step(self.config.dt)

            # 표적 동역학 전파
            self.target.step(self.config.dt)

            # 요격 판정시 조기 종료
            if np.linalg.norm(self.missile.pos - self.target.pos) < 1.0:
                break

            # 미사일/타겟 상태 기록
            self.history["missile"].append(self.missile.pos.copy())
            self.history["target"].append(self.target.pos.copy())

            t += self.config.dt

        return self.history