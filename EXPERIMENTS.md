# 실험 방법

이 문서는 저장소에 포함된 결과를 어떤 조건에서 만들었는지 기록한 것입니다.

## EuRoC V1_01_easy

- IMU: 200 Hz 원본에서 5개마다 하나를 사용해 40 Hz로 입력
- 위치 측정: 20 IMU sample마다 한 번, 즉 2 Hz
- 위치 측정 noise 표준편차: 0.05 m
- 평가 sample 수: 5,824
- 비교 필터: EKF, UKF, PF(500 particles), InEKF

```bash
state-estimation-run-all --config config/euroc.yaml --filters ekf ukf pf inekf
state-estimation-run-all --config config/euroc_imu_only.yaml --filters ekf ukf pf inekf
```

`config/euroc.yaml`의 위치 측정은 실제 GPS가 아니라 GT 위치에 noise를 추가해 만든 값입니다. 이 실험은 모든 필터에 같은 위치 측정을 주었을 때의 차이를 보기 위한 것으로, GPS 성능 실험은 아닙니다.

ESKF는 같은 조건에서 발산했기 때문에 현재 성능표에서 제외했습니다. 구현을 없앤 것은 아니며, error injection 부호와 frame을 다시 확인하는 용도로 남겨 두었습니다.

## 회전 운동 비교

실험 시간, IMU 주기, sensor noise, bias와 1 Hz 위치 update는 같게 두고 각속도와 roll, pitch, z축 운동만 바꿨습니다.

```bash
state-estimation-motion-benchmark --config config/motion_regimes.yaml
```

결과는 `outputs/motion_regimes/metrics.csv`에 저장됩니다. 이 실험은 회전 운동 자체가 필터 차이에 미치는 영향을 분리해 보기 위한 합성 실험입니다. 실제 이동로봇, 선박, 드론의 센서 성능을 그대로 나타내지는 않습니다.

## CF231: 이전 비행으로 학습하고 Run 5에서 평가

- 학습: Runs 3, 4, 9, 10
- IMU noise 분산과 gyro intrinsic 계산: Runs 3, 9, 10
- 평가: 학습에서 제외한 Run 5
- IMU window: 200 samples, 약 2초
- 모델: parameter 36,003개의 Small TCN
- Run 5 입력: 초기 상태, 처음에 계산해 고정한 bias, Run 5 IMU
- Run 5에서 사용하지 않는 값: GT, 위치 측정, GNSS, camera, PWM, thrust, 운동 범위 제약

```bash
state-estimation-cf231-tcn \
  --dataset data/cf231_leave_one_out/csv \
  --output outputs/cf231_small_tcn
```

실행기는 다음 과정을 순서대로 수행합니다.

1. IMU와 pose CSV의 시간을 맞춥니다.
2. Run 5 시작 2.63초의 정지 구간에서 bias를 계산하고 이후에는 바꾸지 않습니다.
3. 학습 run을 번갈아 한 개씩 제외하며 속도 예측 오차 공분산을 구합니다.
4. Runs 3, 4, 9, 10 전체로 Small TCN을 학습합니다.
5. Run 5 IMU로 속도를 예측합니다.
6. 예측 속도를 직접 적분한 결과와 InEKF velocity update에 사용한 결과를 각각 평가합니다.

Run 5의 pose는 초기 위치·속도·자세를 정하고 마지막에 오차를 계산할 때만 사용합니다. 다만 Small TCN을 학습할 때는 다른 run의 GT velocity가 정답으로 필요합니다. 따라서 이 방법은 “평가 중 IMU만 사용하는 학습 기반 추정”이며, 학습 없는 순수 관성항법과는 다릅니다.

상세 수치는 `reports/data/cf231_run5_summary.json`에 있습니다.

## 실험 내용

`reports/data/historical_full_sequence.csv`에는 당시 i2Nav, Pohang, UrbanNav 결과를 옮겨 두었습니다. 이 표는 이전에 어떤 비교를 했는지 확인하기 위한 자료입니다. 아래 조건을 다시 맞추기 전에는 현재 코드의 성능 비교에 사용하지 않습니다.

1. Pohang에서 filter update와 평가 기준으로 서로 다른 위치 신호 사용
2. UrbanNav loader와 당시 설정 확인
3. 데이터별 평가 시간, 위치 update 주기, noise 기록
4. 이동로봇·선박·드론에 같은 평가 방법 적용
5. 회전이 큰 구간의 heading, SO(3), NIS, NEES 비교

## 보고서 그림 다시 만들기

```bash
python reports/build_report_figures.py
```
