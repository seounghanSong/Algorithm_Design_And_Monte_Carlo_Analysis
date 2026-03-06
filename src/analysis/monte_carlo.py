import numpy as np
import matplotlib.pyplot as plt

from config import Config
from simulation import Simulation

from models.missile import Missile3D
from models.target import Target3D

from guidance.png import ProportionalNavigation3D
from control.first_order_autopilot import FirstOrderAutopilot

from analysis.metrics import compute_miss_distance


def run_single_simulation(config):
    missile = Missile3D([0, 0, 0], [300, 0, 0])

    target = Target3D(
        [
            5000 + np.random.normal(0, 200),
            2000 + np.random.normal(0, 200),
            1000 + np.random.normal(0, 100),
        ],
        [
            0,
            -200 + np.random.normal(0, 20),
            0,
        ],
    )

    guidance = ProportionalNavigation3D(
        config.navigation_gain,
        config.max_acceleration,
    )

    controller = FirstOrderAutopilot(tau=0.15)

    sim = Simulation(missile, target, guidance, controller, config)

    history = sim.run()

    missile_traj = np.array(history["missile"])
    target_traj = np.array(history["target"])

    miss = compute_miss_distance(
        missile_traj,
        target_traj,
        config.dt,
    )

    return miss


def monte_carlo(num_runs=500):
    config = Config()

    miss_distances = []

    for _ in range(num_runs):

        # navigation gain randomization
        config.navigation_gain = np.random.uniform(2.5, 4.5)

        miss = run_single_simulation(config)

        miss_distances.append(miss)

    miss_distances = np.array(miss_distances)

    print("===== Monte Carlo Result =====")
    print(f"Runs : {num_runs}")
    print(f"Mean miss distance : {miss_distances.mean():.2f} m")
    print(f"Std  miss distance : {miss_distances.std():.2f} m")
    print(f"Min  miss distance : {miss_distances.min():.2f} m")
    print(f"Max  miss distance : {miss_distances.max():.2f} m")

    # histogram
    plt.figure()
    plt.hist(miss_distances, bins=30)
    plt.xlabel("Miss Distance (m)")
    plt.ylabel("Count")
    plt.title("Monte Carlo Miss Distance Distribution")
    plt.show()


if __name__ == "__main__":
    monte_carlo(500)