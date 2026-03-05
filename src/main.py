import numpy as np
import matplotlib.pyplot as plt

from config import Config
from simulation import Simulation

from models.missile import Missile2D
from models.target import Target2D

from guidance.png import ProportionalNavigation
from control.ideal_controller import IdealAccelerationController
from analysis.metrics import compute_miss_distance


def run_simulation(N):
    # 설정 객체 생성
    config = Config()
    config.navigation_gain = N

    # # models(missile/target) 객체 생성
    # missile = Missile2D([0, 0], [300, 0])
    # target = Target2D([5000, 1000], [250, 0])

    # models(missile/target) 객체 생성
    missile = Missile2D([0, 0], [300, 0])
    target = Target2D([5000, 2000], [0, -200])

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

    return missile_traj, target_traj, miss_distance

def main():
    gains = [1, 2, 3, 4, 5, 6]
    results = []

    plt.figure(figsize=(8,6))

    for N in gains:
        missile_traj, target_traj, miss = run_simulation(N)
        results.append((N, miss))

        plt.plot(missile_traj[:,0], missile_traj[:,1], label=f"N={N}")

    plt.plot(target_traj[:,0], target_traj[:,1], 'k--', label="Target")

    plt.legend()
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title("Trajectory vs Navigation Gain")
    plt.axis("equal")
    plt.show()

    print("=== Miss Distance Results ===")
    for N, miss in results:
        print(f"N={N} → Miss Distance: {miss:.4f} m")


if __name__ == "__main__":
    main()