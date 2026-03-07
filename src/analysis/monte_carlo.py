import numpy as np
import matplotlib.pyplot as plt

from config import Config
from simulation import Simulation

from models.missile import Missile3D
from models.target import Target3D

from guidance.png import ProportionalNavigation3D
from control.first_order_autopilot import FirstOrderAutopilot

from analysis.metrics import compute_miss_distance


def sample_case():
    return {
        "missile_pos": [
            np.random.normal(0, 10),
            np.random.normal(0, 10),
            np.random.normal(0, 5),
        ],
        "missile_vel": [
            300 + np.random.normal(0, 5),
            np.random.normal(0, 2),
            np.random.normal(0, 2),
        ],
        "target_pos": [
            5000 + np.random.normal(0, 200),
            2000 + np.random.normal(0, 200),
            1000 + np.random.normal(0, 100),
        ],
        "target_vel": [
            np.random.normal(0, 10),
            -200 + np.random.normal(0, 20),
            np.random.normal(0, 10),
        ],
        "navigation_gain": np.random.uniform(2.5, 4.5),
        "max_acceleration": np.random.uniform(250.0, 450.0),
        "tau": np.random.uniform(0.03, 0.07),
        "success_threshold": 5.0,
    }

def run_single_simulation(config, case):
    missile = Missile3D(case["missile_pos"], case["missile_vel"])
    target = Target3D(case["target_pos"], case["target_vel"])

    guidance = ProportionalNavigation3D(
        case["navigation_gain"],
        case["max_acceleration"],
    )

    controller = FirstOrderAutopilot(tau=case["tau"])

    sim = Simulation(missile, target, guidance, controller, config)
    history = sim.run()

    missile_traj = np.array(history["missile"])
    target_traj = np.array(history["target"])

    miss = compute_miss_distance(
        missile_traj,
        target_traj,
        config.dt,
    )

    intercept_time = len(missile_traj) * config.dt
    success = miss < case["success_threshold"]

    return {
        "miss_distance": miss,
        "success": success,
        "intercept_time": intercept_time,
        "navigation_gain": case["navigation_gain"],
        "max_acceleration": case["max_acceleration"],
        "tau": case["tau"],
    }

def summarize_results(results):
    miss = np.array([r["miss_distance"] for r in results])
    success = np.array([r["success"] for r in results])
    intercept_time = np.array([r["intercept_time"] for r in results])

    print("===== Monte Carlo Result =====")
    print(f"Runs               : {len(results)}")
    print(f"Mean miss distance : {np.mean(miss):.2f} m")
    print(f"Std  miss distance : {np.std(miss):.2f} m")
    print(f"Median miss        : {np.median(miss):.2f} m")
    print(f"P95 miss distance  : {np.percentile(miss, 95):.2f} m")
    print(f"Min  miss distance : {np.min(miss):.2f} m")
    print(f"Max  miss distance : {np.max(miss):.2f} m")
    print(f"Pk (<5m)           : {np.mean(miss < 5.0):.3f}")
    print(f"Pk (<10m)          : {np.mean(miss < 10.0):.3f}")
    print(f"Success rate       : {np.mean(success):.3f}")
    print(f"Mean intercept time: {np.mean(intercept_time):.2f} s")

def plot_results(results):
    miss = np.array([r["miss_distance"] for r in results])
    n_values = np.array([r["navigation_gain"] for r in results])
    max_acc_values = np.array([r["max_acceleration"] for r in results])

    plt.figure()
    plt.hist(miss, bins=30)
    plt.xlabel("Miss Distance (m)")
    plt.ylabel("Count")
    plt.title("Monte Carlo Miss Distance Distribution")
    plt.show()

    plt.figure()
    plt.scatter(n_values, miss, s=10)
    plt.xlabel("Navigation Gain")
    plt.ylabel("Miss Distance (m)")
    plt.title("Miss Distance vs Navigation Gain")
    plt.show()

    plt.figure()
    plt.scatter(max_acc_values, miss, s=10)
    plt.xlabel("Max Acceleration (m/s^2)")
    plt.ylabel("Miss Distance (m)")
    plt.title("Miss Distance vs Max Acceleration")
    plt.show()

def plot_grouped_results(grouped_results, variable_name):
    x = []
    mean_miss = []
    p95_miss = []
    pk_5m = []

    for value, results in grouped_results.items():
        miss = np.array([r["miss_distance"] for r in results])

        x.append(value)
        mean_miss.append(np.mean(miss))
        p95_miss.append(np.percentile(miss, 95))
        pk_5m.append(np.mean(miss < 5.0))

    plt.figure()
    plt.plot(x, mean_miss, marker="o")
    plt.xlabel(variable_name)
    plt.ylabel("Mean Miss Distance (m)")
    plt.title(f"Mean Miss Distance vs {variable_name}")
    plt.show()

    plt.figure()
    plt.plot(x, p95_miss, marker="o")
    plt.xlabel(variable_name)
    plt.ylabel("P95 Miss Distance (m)")
    plt.title(f"P95 Miss Distance vs {variable_name}")
    plt.show()

    plt.figure()
    plt.plot(x, pk_5m, marker="o")
    plt.xlabel(variable_name)
    plt.ylabel("Pk (<5m)")
    plt.title(f"Intercept Probability vs {variable_name}")
    plt.show()

def summarize_grouped_results(grouped_results, variable_name):
    print(f"===== Sensitivity Analysis: {variable_name} =====")

    for value, results in grouped_results.items():
        miss = np.array([r["miss_distance"] for r in results])
        success = np.array([r["success"] for r in results])
        intercept_time = np.array([r["intercept_time"] for r in results])

        print(f"Runs               : {len(grouped_results)}")
        print(f"Mean miss distance : {np.mean(miss):.2f} m")
        print(f"Std  miss distance : {np.std(miss):.2f} m")
        print(f"Median miss        : {np.median(miss):.2f} m")
        print(f"P95 miss distance  : {np.percentile(miss, 95):.2f} m")
        print(f"Min  miss distance : {np.min(miss):.2f} m")
        print(f"Max  miss distance : {np.max(miss):.2f} m")
        print(f"Pk (<5m)           : {np.mean(miss < 5.0):.3f}")
        print(f"Pk (<10m)          : {np.mean(miss < 10.0):.3f}")
        print(f"Success rate       : {np.mean(success):.3f}")
        print(f"Mean intercept time: {np.mean(intercept_time):.2f} s")

# 시스템의 Robustness 확인可
def monte_carlo(num_runs=500, seed=42):
    np.random.seed(seed)

    config = Config()
    results = []

    for _ in range(num_runs):
        case = sample_case()
        result = run_single_simulation(config, case)
        results.append(result)

    summarize_results(results)
    plot_results(results)

    return results

# N의 순수 영향 비교可
def monte_carlo_by_navigation_gain(n_values, num_runs=200, seed=42):
    np.random.seed(seed)
    config = Config()
    grouped_results = {}

    for n in n_values:
        group = []

        for _ in range(num_runs):
            case = sample_case()
            case["navigation_gain"] = n
            result = run_single_simulation(config, case)
            group.append(result)

        grouped_results[n] = group

    summarize_grouped_results(grouped_results, "N")
    plot_grouped_results(grouped_results, "N")

    return grouped_results

# tau의 순수 영향 비교可
def monte_carlo_by_tau(tau_values, num_runs=200, seed=42):
    np.random.seed(seed)
    config = Config()
    grouped_results = {}

    for tau in tau_values:
        group = []

        for _ in range(num_runs):
            case = sample_case()
            case["tau"] = tau
            result = run_single_simulation(config, case)
            group.append(result)

        grouped_results[tau] = group

    summarize_grouped_results(grouped_results, "tau")
    plot_grouped_results(grouped_results, "tau")

    return grouped_results


# max_acc의 순수 영향 비교可
def monte_carlo_by_max_acc(acc_values, num_runs=200, seed=42):
    np.random.seed(seed)
    config = Config()
    grouped_results = {}

    for acc in acc_values:
        group = []

        for _ in range(num_runs):
            case = sample_case()
            case["max_acceleration"] = acc
            result = run_single_simulation(config, case)
            group.append(result)

        grouped_results[acc] = group

    summarize_grouped_results(grouped_results, "max_acc")
    plot_grouped_results(grouped_results, "max_acc")

    return grouped_results

if __name__ == "__main__":
    # # 전체 시스템 Rubustness 시뮬레이션
    # monte_carlo(500, 42)

    # # navigation gain 독립 영향 시뮬레이션
    # monte_carlo_by_navigation_gain(n_values=[1, 2, 3, 4, 5, 6], num_runs=200, seed=42)

    # # tau 독립 영향 시뮬레이션
    # monte_carlo_by_tau(tau_values=[0.02, 0.04, 0.06, 0.08, 0.10], num_runs=200, seed=42)

    # max_acc 독립 영향 시뮬레이션
    monte_carlo_by_max_acc(acc_values=[100, 200, 300, 400, 500], num_runs=200, seed=42)