import numpy as np

# 요격 성능 평가 지표
def compute_miss_distance(missile_traj, target_traj):
    # 시간별 상대적 거리 계산
    distances = np.linalg.norm(missile_traj - target_traj, axis=1)

    # 최솟값 반환
    return np.min(distances)