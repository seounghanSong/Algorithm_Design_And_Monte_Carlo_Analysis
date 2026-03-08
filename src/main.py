import numpy as np

from config import Config
from simulation import Simulation

from models.missile import Missile3D
from models.target import Target3D

from guidance.png import ProportionalNavigation3D
from control.first_order_autopilot import FirstOrderAutopilot
from sensor.seeker import PositionSeekerWithNoise
from estimation.ekf import TargetStateEKF
from analysis.metrics import compute_miss_distance


def run_simulation(N):
    config = Config()
    config.navigation_gain = N

    missile = Missile3D([0, 0, 0], [300, 0, 0])
    target = Target3D([5000, 2000, 1000], [0, -200, 0])

    guidance = ProportionalNavigation3D(
        config.navigation_gain,
        config.max_acceleration,
    )
    controller = FirstOrderAutopilot(tau=0.15)

    seeker = PositionSeekerWithNoise(pos_noise_std=5.0)

    # 초기 EKF 상태: 대략적인 표적 초기 추정
    x0 = [5000, 2000, 1000, 0, -200, 0]
    estimator = TargetStateEKF(x0=x0, q=1.0, r=25.0)

    sim = Simulation(
        missile,
        target,
        guidance,
        controller,
        config,
        seeker=seeker,
        estimator=estimator,
    )
    history = sim.run()

    missile_traj = np.array(history["missile"])
    target_traj = np.array(history["target"])

    miss_distance = compute_miss_distance(
        missile_traj,
        target_traj,
        config.dt,
    )

    return missile_traj, target_traj, miss_distance