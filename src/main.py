import numpy as np
import matplotlib.pyplot as plt

from config import Config
from simulation import Simulation

from models.missile import Missile2D
from models.target import Target2D

from guidance.png import ProportionalNavigation
from control.ideal_controller import IdealAccelerationController
from analysis.metrics import compute_miss_distance


def main():
    # 설정 객체 생성
    config = Config()

    # models(missile/target) 객체 생성
    missile = Missile2D([0, 0], [300, 0])
    target = Target2D([5000, 1000], [250, 0])

    # guidance/control 객체 생성
    guidance = ProportionalNavigation(config.navigation_gain)
    controller = IdealAccelerationController()

    # 시뮬레이션 실행
    sim = Simulation(missile, target, guidance, controller, config)
    history = sim.run()

    # 오차 계산
    missile_traj = np.array(history["missile"])
    target_traj = np.array(history["target"])

    miss_distance = compute_miss_distance(missile_traj, target_traj)
    print(f"Miss Distance: {miss_distance:.2f} m")

    # 결과 출력 및 그래프 시각화
    plt.plot(missile_traj[:, 0], missile_traj[:, 1])
    plt.plot(target_traj[:, 0], target_traj[:, 1])
    plt.legend(["Missile", "Target"])
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title("2D Proportional Navigation")
    plt.axis("equal")
    plt.show()


if __name__ == "__main__":
    main()