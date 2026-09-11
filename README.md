# Inertial State Estimation Benchmark

이 저장소는 IMU를 사용한 상태추정 실험을 한곳에서 실행하기 위해 정리한 코드입니다. EKF, UKF, PF, ESKF, Invariant EKF(InEKF)를 같은 입력과 평가 방식으로 비교하고, IMU-only dead reckoning과 위치 측정을 추가한 경우를 나눠서 확인합니다. CF231 비행 데이터에서는 IMU 구간으로부터 속도를 예측하는 Small TCN과 InEKF를 결합한 실험도 포함합니다.

우리가 확인하고자 한 것은 다음과 같습니다.

1. bias를 한 번 정해 고정했을 때 IMU-only 위치와 자세 오차가 어떻게 증가하는지 확인한다.
2. 같은 IMU에 위치 측정을 넣었을 때 필터별 위치, roll, pitch, heading 오차를 비교한다.
3. 회전 운동이 커질 때 InEKF의 특성이 실제 오차에 어떻게 나타나는지 확인한다.
4. 테스트 시점에는 IMU만 사용하면서, 이전 데이터로 학습한 속도 정보가 dead reckoning을 얼마나 개선하는지 확인한다.


## 1. 상태와 입출력

모든 필터의 IMU 입력 순서는 같습니다.

```text
[ax, ay, az, gx, gy, gz]
```

- 가속도: `m/s^2`
- 각속도: `rad/s`
- 시간 간격 `dt`: `s`

공통 nominal state는 위치, 속도, 자세, 자이로 bias, 가속도계 bias로 구성합니다.

```text
X = {R, v, p, bg, ba}
dx = [dtheta, dv, dp, dbg, dba] in R^15
```

외부에서 읽는 pose는 다음 6개 값으로 통일했습니다.

```text
[px, py, pz, roll, pitch, yaw]
```

자세 출력은 radian입니다. 중력 방향과 센서 축은 데이터셋마다 다르므로 `config/`의 `gravity`와 각 loader의 축 변환을 먼저 확인해야 합니다.

## 2. 구현된 필터

| 필터 | 자세 표현 | 용도 | 현재 상태 |
|---|---|---|---|
| EKF | quaternion nominal state | 기본 비교군 | EuRoC 실험에서 정상 동작 |
| UKF | quaternion + 15D local error | sigma point 비교 | EuRoC 실험에서 정상 동작 |
| PF | quaternion particle | 비가우시안 비교 | particle 수에 따라 실행시간과 결과 차이가 큼 |
| ESKF | rotation matrix + 15D error state | error-state 비교 | 현재 EuRoC에서 발산하여 검증 중 |
| InEKF | `SE_2(3)` + invariant error | 주요 비교 필터 | analytic Jacobian 사용 |

ESKF는 API에는 남겨 두었지만, 현재 결과를 필터 성능으로 해석하지 않습니다. error injection 부호와 frame, measurement Jacobian을 다시 확인한 다음 비교표에 넣어야 합니다.

## 3. 폴더 구조

```text
config/             데이터셋, 필터 noise, update 주기 설정
datasets/           EuRoC, i2Nav, Pohang, CF231 loader
evaluation/         위치·자세 오차 계산과 그래프
filters/            EKF, UKF, PF, ESKF, InEKF
learned/            Small TCN과 InEKF velocity update
models/             상태 순서와 SO(3)/SE_2(3) 계산
runners/            명령행 실험 실행기
state_estimation/   설치 후 사용하는 Python API
tests/              필터 API와 Small TCN 회귀 테스트
reports/            결과 CSV, 그래프, 보고서
docs/               상태정의와 InEKF 구현 설명
```

## 4. 설치

Python 3.10 이상을 사용합니다.

```bash
git clone https://github.com/INHA-Artemis/inertial-state-estimation-benchmark.git
cd inertial-state-estimation-benchmark

python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
pip install -e .
```

Small TCN까지 실행할 때는 PyTorch를 추가로 설치합니다.

```bash
pip install -e ".[learned]"
```

Python 환경을 따로 만들기 어려운 경우에는 데이터가 필요 없는 기본 예제를 Docker로 확인할 수 있습니다.

```bash
docker build -t inertial-state-estimation .
docker run --rm inertial-state-estimation
```

## 5. 설치 확인

데이터셋 없이도 공통 API를 확인할 수 있습니다.

```bash
python examples/basic_filter_loop.py
```

정상적으로 실행되면 정지 IMU와 주기적인 원점 위치 측정을 사용한 최종 pose가 출력됩니다.

테스트는 다음과 같이 실행합니다.

```bash
pip install -e ".[test,learned]"
python -m pytest -q
```

## 6. Python에서 필터 사용하기

```python
from state_estimation import create_filter

filter_ = create_filter(
    "inekf",
    mode="fused",
    motion_config={
        "gravity": [0.0, 0.0, -9.81],
        "gyro_bias": [0.0, 0.0, 0.0],
        "accel_bias": [0.0, 0.0, 0.0],
    },
    measurement_config={
        "measurement_noise_diag": [0.01, 0.01, 0.01],
    },
    initialization_config={
        "mean": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        "velocity_mean": [0.0, 0.0, 0.0],
        "cov_diag": [1.0e-3] * 15,
    },
)

filter_.predict([ax, ay, az, gx, gy, gz], dt)
filter_.measurement_update([gps_x, gps_y, gps_z])
pose = filter_.estimate_pose()
```

`mode="imu_only"`로 두면 `predict()`만 사용하고, `mode="fused"`로 두면 위치 측정이 들어오는 시점에 `measurement_update()`를 호출합니다.

## 7. 데이터 배치

원시 데이터는 GitHub에 넣지 않고, 저장소 루트의 `data/`에 배치합니다.

```text
data/
├── euroc/V1_01_easy/mav0/
├── i2nav_robot/street00/
├── pohang_canal/pohang05/
└── cf231_leave_one_out/csv/
```

필요한 파일명과 다운로드 링크는 [DATASETS.md](DATASETS.md)에 정리했습니다.

## 8. 실험 실행

### 8.1 EuRoC IMU-only

```bash
state-estimation-run-all \
  --config config/euroc_imu_only.yaml \
  --filters ekf ukf pf inekf
```

초기 위치, 속도, 자세와 bias를 설정한 뒤 IMU만 적분합니다. 평가 구간 중 GT를 update에 사용하지 않습니다.

### 8.2 EuRoC IMU + 2 Hz 위치 측정

```bash
state-estimation-run-all \
  --config config/euroc.yaml \
  --filters ekf ukf pf inekf
```

이 설정은 실제 GPS가 아니라 EuRoC GT 위치에 0.05 m 표준편차의 noise를 넣은 값을 2 Hz로 사용합니다. 따라서 GPS 성능 검증이 아니라, 동일한 위치 update에서 필터 구조를 비교하는 실험입니다.

### 8.3 3D motion-regime stress test

```bash
state-estimation-motion-benchmark \
  --config config/motion_regimes.yaml
```

센서 noise, bias, 실험 길이와 위치 update 조건은 고정하고, mobile robot-like, surface vessel-like, drone-like 운동의 회전 및 운동 복잡도를 단계적으로 증가시킵니다.

### 8.4 i2Nav street00

```bash
state-estimation-run-all \
  --config config/i2nav_street00.yaml \
  --filters ekf ukf pf inekf
```

F9P GNSS를 위치 update에 사용하고 `groundtruth.nav`로 오차를 계산합니다. `DATASETS.md`의 파일명이 실제로 받은 sequence와 같은지 먼저 확인해야 합니다.

### 8.5 Pohang05

```bash
state-estimation-run-all \
  --config config/pohang05.yaml \
  --filters ekf ukf pf inekf
```

현재 설정은 `baseline.txt`를 위치 update와 평가에 같이 사용합니다. 코드를 실행할 수는 있지만, 이렇게 얻은 위치 RMSE를 독립적인 localization 성능으로 사용하면 안 됩니다.

### 8.6 필터 하나만 실행

```bash
state-estimation-run \
  --filter inekf \
  --config config/euroc.yaml
```

`--max-steps 1000`을 추가하면 앞부분만 빠르게 확인할 수 있습니다. PF particle 수는 `--particles 5000`, 그림 저장을 생략할 때는 `--no-plots`를 사용합니다.

### 8.7 CF231 Small TCN

```bash
state-estimation-cf231-tcn \
  --dataset data/cf231_leave_one_out/csv \
  --output outputs/cf231_small_tcn
```

- Runs 3, 4, 9, 10: Small TCN 학습
- Runs 3, 9, 10: IMU noise 분산과 gyro intrinsic 계산
- Run 5: 학습에서 제외한 평가 데이터
- Run 5 시작 2.63 s: IMU 평균으로 bias를 한 번 계산한 뒤 고정
- 추론 구간: Run 5 IMU만 사용

Run 5에 적용한 고정 bias는 다음과 같습니다.

```text
gyro bias  = [-3.308e-4, -9.214e-5,  3.601e-5] rad/s
accel bias = [ 0.41543,   0.14960,    0.00771  ] m/s^2
```

Small TCN의 학습 label로는 다른 run의 GT velocity를 사용합니다. 따라서 이 결과는 테스트 중 사용한 센서는 IMU만이지만, 학습 없는 순수 IMU 적분과는 구분해서 표기해야 합니다.

## 9. 결과 파일

`state-estimation-run` 또는 `state-estimation-run-all`을 실행하면 다음 형태로 저장됩니다.

```text
outputs/<dataset>/<sequence>/<filter>/
├── estimate.csv
├── metrics.json
├── runtime.json
├── trajectory.png
└── error.png
```

`metrics.json`에는 위치 RMSE, 최대/최종 위치 오차, roll/pitch/heading RMSE, 최종 heading 오차, 실행시간이 저장됩니다.

CF231 학습 실험은 `summary.json`, `trajectories.npz`, `small_tcn.pt`, `trajectory_comparison.png`를 저장합니다.

## 10. 현재까지의 주요 결과

| 실험 | 방법 | 위치 RMSE | 자세/헤딩 RMSE | 해석 |
|---|---|---:|---:|---|
| EuRoC IMU-only | EKF | 1,129.28 m | heading 7.72° | 위치는 시간이 지날수록 발산 |
| EuRoC IMU-only | InEKF | 1,130.12 m | heading 7.72° | update가 없으면 EKF와 거의 같은 결과 |
| EuRoC + 2 Hz 위치 | EKF | 0.110 m | heading 6.09° | 현재 설정에서 가장 낮은 위치 RMSE |
| EuRoC + 2 Hz 위치 | InEKF | 0.114 m | heading 6.43° | 정확도는 EKF와 비슷, 실행시간은 더 짧음 |
| CF231 | fixed-bias IMU DR | 522.20 m | SO(3) 3.55° | 위치 발산 |
| CF231 | Small TCN | **2.40 m** | SO(3) 3.55° | 속도를 직접 적분했을 때 위치 RMSE가 가장 낮음 |
| CF231 | Small TCN + InEKF | 2.61 m | **SO(3) 1.80°** | Small TCN보다 위치는 약간 나빠지지만 자세가 개선됨 |

전체 수치와 그래프는 [RESULTS.md](RESULTS.md), 각 실험의 설정은 [EXPERIMENTS.md](EXPERIMENTS.md), 연구 과정과 해석은 [보고서](reports/STATE_ESTIMATION_REPORT.md)에서 확인할 수 있습니다.
