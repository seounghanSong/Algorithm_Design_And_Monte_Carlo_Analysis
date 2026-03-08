import numpy as np
import matplotlib.pyplot as plt

from config import Config
from simulation import Simulation

from models.missile import Missile3D
from models.target import Target3D

from guidance.png import ProportionalNavigation3D
from control.first_order_autopilot import FirstOrderAutopilot
from sensor.seeker import PositionSeekerWithNoise
from estimation.ekf import TargetStateEKF

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

        # sensor noise
        "pos_noise_std": np.random.uniform(1.0, 10.0),
        "vel_noise_std": np.random.uniform(0.5, 5.0),

        # EKF param
        "ekf_q": np.random.uniform(0.1, 5.0),
        "ekf_r": np.random.uniform(4.0, 100.0),
        "ekf_init_pos_error_std": np.random.uniform(0.0, 100.0),
        "ekf_init_vel_error_std": np.random.uniform(0.0, 20.0),

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

    # sensor 생성
    seeker = PositionSeekerWithNoise(
        pos_noise_std=case["pos_noise_std"]
    )

    # EKF 초기 상태 설정
    ekf_init_pos = np.array(case["target_pos"]) + np.random.normal(0, case["ekf_init_pos_error_std"], 3)
    ekf_init_vel = np.array(case["target_vel"]) + np.random.normal(0, case["ekf_init_vel_error_std"], 3)
    x0 = np.concatenate([ekf_init_pos, ekf_init_vel])

    estimator = TargetStateEKF(
        x0=x0,
        q=case["ekf_q"],
        r=case["ekf_r"],
    )

    # simulation 실행
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
        "pos_noise_std": case["pos_noise_std"],
        "ekf_q": case["ekf_q"],
        "ekf_r": case["ekf_r"],
        "ekf_init_pos_error_std": case["ekf_init_pos_error_std"],
        "ekf_init_vel_error_std": case["ekf_init_vel_error_std"],
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
    tau_values = np.array([r["tau"] for r in results])
    pos_noise_values = np.array([r["pos_noise_std"] for r in results])
    ekf_q_values = np.array([r["ekf_q"] for r in results])
    ekf_r_values = np.array([r["ekf_r"] for r in results])

    plt.figure()
    plt.hist(miss, bins=30)
    plt.xlabel("Miss Distance(m)")
    plt.ylabel("Count")
    plt.title("Monte Carlo Miss Distance Distribution")
    plt.show()

    plt.figure()
    plt.scatter(n_values, miss, s=10)
    plt.xlabel("Navigation Gain")
    plt.ylabel("Miss Distance(m)")
    plt.title("Miss Distance vs Navigation Gain")
    plt.show()

    plt.figure()
    plt.scatter(max_acc_values, miss, s=10)
    plt.xlabel("Max Acceleration(m/s^2)")
    plt.ylabel("Miss Distance(m)")
    plt.title("Miss Distance vs Max Acceleration")
    plt.show()

    plt.figure()
    plt.scatter(tau_values, miss, s=10)
    plt.xlabel("Autopilot Time Constant(tau)")
    plt.ylabel("Miss Distance(m)")
    plt.title("Miss Distance vs Tau")
    plt.show()

    plt.figure()
    plt.scatter(pos_noise_values, miss, s=10)
    plt.xlabel("Position Noise STD")
    plt.ylabel("Miss Distance(m)")
    plt.title("Miss Distance vs Position Noise STD")
    plt.show()

    plt.figure()
    plt.scatter(ekf_q_values, miss, s=10)
    plt.xscale("log")
    plt.xlabel("EKF q")
    plt.ylabel("Miss Distance(m)")
    plt.title("Miss Distance vs EKF q")
    plt.show()

    plt.figure()
    plt.scatter(ekf_r_values, miss, s=10)
    plt.xscale("log")
    plt.xlabel("EKF r")
    plt.ylabel("Miss Distance(m)")
    plt.title("Miss Distance vs EKF r")
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

# position_noise의 순수 영향 비교可
def monte_carlo_by_position_noise(noise_values, num_runs=200, seed=42):
    np.random.seed(seed)
    config = Config()
    grouped_results = {}

    for noise in noise_values:
        group = []

        for _ in range(num_runs):
            case = sample_case()
            case["pos_noise_std"] = noise
            result = run_single_simulation(config, case)
            group.append(result)

        grouped_results[noise] = group

    summarize_grouped_results(grouped_results, "pos_noise_std")
    plot_grouped_results(grouped_results, "pos_noise_std")

    return grouped_results

# velocity_noise의 순수 영향 비교可
def monte_carlo_by_velocity_noise(noise_values, num_runs=200, seed=42):
    np.random.seed(seed)
    config = Config()
    grouped_results = {}

    for noise in noise_values:
        group = []

        for _ in range(num_runs):
            case = sample_case()
            case["vel_noise_std"] = noise
            result = run_single_simulation(config, case)
            group.append(result)

        grouped_results[noise] = group

    summarize_grouped_results(grouped_results, "vel_noise_std")
    plot_grouped_results(grouped_results, "vel_noise_std")

    return grouped_results

# ekf_q의 순수 영향 비교可
def monte_carlo_by_ekf_q(q_values, num_runs=200, seed=42):
    np.random.seed(seed)
    config = Config()
    grouped_results = {}

    for q in q_values:
        group = []

        for _ in range(num_runs):
            case = sample_case()
            case["ekf_q"] = q
            result = run_single_simulation(config, case)
            group.append(result)

        grouped_results[q] = group

    summarize_grouped_results(grouped_results, "ekf_q")
    plot_grouped_results(grouped_results, "ekf_q")

    return grouped_results

# ekf_r의 순수 영향 비교可
def monte_carlo_by_ekf_r(r_values, num_runs=200, seed=42):
    np.random.seed(seed)
    config = Config()
    grouped_results = {}

    for r in r_values:
        group = []

        for _ in range(num_runs):
            case = sample_case()
            case["ekf_r"] = r
            result = run_single_simulation(config, case)
            group.append(result)

        grouped_results[r] = group

    summarize_grouped_results(grouped_results, "ekf_r")
    plot_grouped_results(grouped_results, "ekf_r")

    return grouped_results


if __name__ == "__main__":
    # 전체 시스템 Rubustness 시뮬레이션
    monte_carlo(500, 42)

    # # navigation gain 독립 영향 시뮬레이션
    # monte_carlo_by_navigation_gain(n_values=[1, 2, 3, 4, 5, 6], num_runs=200, seed=42)

    # # tau 독립 영향 시뮬레이션
    # monte_carlo_by_tau(tau_values=[0.02, 0.04, 0.06, 0.08, 0.10], num_runs=200, seed=42)

    # # max_acc 독립 영향 시뮬레이션
    # monte_carlo_by_max_acc(acc_values=[100, 200, 300, 400, 500], num_runs=200, seed=42)

    # # position_noise 독립 영향 시뮬레이션
    # monte_carlo_by_position_noise(noise_values=[0, 0.5, 1, 2, 5, 10], num_runs=200, seed=42)

    # # velocity_noise 독립 영향 시뮬레이션
    # monte_carlo_by_velocity_noise(noise_values=[0, 0.2, 0.5, 1], num_runs=200, seed=42)

    # # ekf_q 독립 영향 시뮬레이션(baseline 주변)
    # monte_carlo_by_ekf_q(q_values=[0.1, 0.5, 1.0, 2.0, 5.0], num_runs=200, seed=42)

    # # ekf_r 독립 영향 시뮬레이션(2m~10m 위치 오차 범위)
    # monte_carlo_by_ekf_r(r_values=[4, 9, 25, 49, 100], num_runs=200, seed=42)
