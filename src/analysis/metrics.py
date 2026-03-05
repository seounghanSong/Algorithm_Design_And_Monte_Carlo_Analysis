import numpy as np

# 요격 성능 평가 지표
def compute_miss_distance(missile_traj, target_traj, dt):
    min_dist = np.inf

    for k in range(len(missile_traj) - 1):

        r0 = missile_traj[k] - target_traj[k]
        r1 = missile_traj[k+1] - target_traj[k+1]

        # 상대 속도 (선형 가정)
        v_rel = (r1 - r0) / dt

        v_norm_sq = np.dot(v_rel, v_rel)

        if v_norm_sq < 1e-12:
            dist = np.linalg.norm(r0)
            min_dist = min(min_dist, dist)
            continue

        # 구간 내 최소 거리 시점
        tau = -np.dot(r0, v_rel) / v_norm_sq
        tau = np.clip(tau, 0.0, dt)

        r_tau = r0 + v_rel * tau
        dist = np.linalg.norm(r_tau)

        min_dist = min(min_dist, dist)

    return min_dist