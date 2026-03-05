class Config:
    # 시뮬레이션 간격(s): 계산 정확도 확보를 위한 변수
    dt = 0.005

    # 총 시뮬레이션 시간(s): 성능 판단의 타당성 확보를 위한 변수
    total_time = 20.0

    # 비례항법계수(N): Miss Distance 최소화를 위한 변수
    navigation_gain = 3.0

    # 최대 가속도 제한
    max_acceleration = 40.0   # m/s^2 (예: 약 4g)