# 📡 Algorithm_Design_And_Monte_CarloAnalysis

### 0. 개요

- 목표: **6-DOF(Degree of Freedom)** 기반의 **유도조종 알고리즘 설계 및 몬테카를로 성능 분석**

- 기대효과: 샘플

### 1. 배경 및 기대효과

### 2. 설계구조 및 실험환경

- 설계구조(베이스라인)
```
src/
│
├── main.py
│   └─ 시뮬레이션 실행 진입점(객체 생성, 시뮬레이션 실행, miss distance 계산, 결과 출력 및 시각화)
│
├── config.py
│   └─ 시뮬레이션 전역 설정값(dt, 총 시간, 항법 이득 등) 관리
│
├── simulation.py
│   └─ 시뮬레이션 엔진(시간 루프, 유도→제어→동역학 업데이트 흐름 제어, 히스토리 저장)
│
├── models/
│   ├── __init__.py
│   │   └─ 패키지 초기화 파일(모듈 import 경로 구성)
│   │
│   ├── base.py
│   │   └─ 동역학 모델 공통 인터페이스 정의(step 메서드 강제)
│   │
│   ├── missile.py
│   │   └─ 미사일 2D 동역학 모델(위치·속도·가속도 명령 상태 및 시간 적분)
│   │
│   └── target.py
│       └─ 표적 2D 동역학 모델(위치·속도 상태 및 시간 적분)
│
├── guidance/
│   ├── __init__.py
│   │   └─ 패키지 초기화 파일
│   │
│   ├── base.py
│   │   └─ 유도법칙 공통 인터페이스 정의(compute_command 강제)
│   │
│   └── png.py
│       └─ Proportional Navigation 구현(LOS rate 기반 가속도 명령 계산)
│
├── control/
│   ├── __init__.py
│   │   └─ 패키지 초기화 파일
│   │
│   ├── base.py
│   │   └─ 제어기 공통 인터페이스 정의(update 강제)
│   │
│   └── ideal_controller.py
│       └─ 이상적 가속도 응답 모델(명령 가속도를 즉시 적용)
│
└── analysis/
    ├── __init__.py
    │   └─ 패키지 초기화 파일
    │
    └── metrics.py
        └─ 성능 분석 함수(miss distance 계산 등 평가 지표 정의)

```

| Param | Unit |
| --- | --- |
| **위치(pos)** | m |
| **거리(vel)** | m/s |
| **가속도(acc)** | m/s<sup>2</sup> |
| **시간(t/dt)** | s |
| **LOS Rate** | rad/s |

- 실험환경: MacBook Pro
 
 | About | Specification |
 | --- | --- |
 | **OS** | 13.7.8 |
 | **Processor** | 2.3GHz Dual-Core Intel Core i5 |
 | **Graphic** | Intel Iris+ Graphics 640 1536MB |
 | **Memory** | 8GB 2133MHz LPDDR3 |

### 3. 실험 내용

#### [Exp01]: Baseline

- 목표: Baseline의 성능 확인
- 이유: 추후 연구 필요성 확인
- 일시: 2026.03.05

![exp01](../assets/result/exp01/Figure_1.png)

| Param | PNG(Base) |
| --- | --- |
| **Dt** | 0.01 |
| **TotalTime** | 20.0 |
| **NavigationGain** | 3.0 |
| **MissDistance** | 4121.55 |

