# IMU 기반 상태추정 벤치마크 최종 기술 보고서

## Invariant EKF 검증, 고정-bias 관성항법, 위치 결합, 회전량 통제 실험 및 경량 학습 속도 보정

> **Repository:** `INHA-Artemis/inertial-state-estimation-benchmark`  
> **Document status:** Final repository report  
> **Primary evidence:** 현재 저장소의 실행기와 설정으로 다시 생성 가능한 EuRoC, 합성 회전량 실험, CF231 held-out Run 5 결과

---

## Executive summary

이 저장소의 목적은 서로 다른 상태추정 알고리즘을 **같은 IMU 입력, 같은 상태 정의, 같은 측정 조건, 같은 평가지표**로 비교할 수 있는 재현 가능한 벤치마크를 만드는 것이다. 비교 대상은 EKF, UKF, PF, ESKF, Invariant EKF(InEKF)이며, 실험은 크게 세 범주로 나뉜다. 첫째, 외부 측정 없이 IMU만 적분하는 dead reckoning. 둘째, 동일한 위치 측정을 추가한 filtering. 셋째, 과거 데이터로 학습한 IMU-to-velocity 모델을 평가 시 IMU만 사용하여 적용하고 그 속도를 직접 적분하거나 InEKF의 virtual measurement로 결합하는 learned inertial estimation이다.

현재 저장소에서 다시 실행 가능한 결과는 다음 결론을 지지한다.

| 연구 질문 | 현재 결론 | 근거 |
|---|---|---|
| 고정 bias만으로 장시간 IMU-only 위치를 유지할 수 있는가? | **아니오.** 위치 drift가 빠르게 누적된다. | EuRoC, CF231 |
| InEKF가 EKF보다 항상 더 정확한가? | **아니오.** 현재 EuRoC fused 설정에서는 EKF가 위치/heading RMSE가 약간 낮다. | EuRoC V1_01_easy |
| 회전이 커질수록 InEKF의 상대 이점이 단조롭게 커지는가? | **확인되지 않음.** | 회전량 통제 합성 실험 |
| 과거 run으로 학습한 IMU-to-velocity가 held-out 비행의 위치 drift를 줄이는가? | **예.** CF231 Run 5에서 큰 폭의 개선을 확인했다. | Small TCN held-out evaluation |
| learned velocity를 InEKF에 넣으면 무엇이 좋아지는가? | 위치 RMSE는 직접 적분보다 약간 증가하지만 **SO(3) 자세 안정성이 크게 개선**된다. | CF231 Run 5 |

가장 중요한 해석은 다음과 같다. **InEKF는 IMU의 비관측 drift를 없애는 알고리즘이 아니다.** 측정 update가 없는 prediction-only 조건에서는 EKF와 InEKF가 같은 strapdown nominal dynamics를 적분하므로 궤적이 거의 동일하게 발산한다. InEKF의 구조적 차이는 측정이 들어와 covariance와 cross-correlation을 통해 상태를 교정할 때 의미가 있다.

또한 CF231의 learned 결과는 “학습 없는 순수 IMU dead reckoning”과 구분해야 한다. Run 5 평가 중에는 GT, GNSS, camera, PWM, thrust를 사용하지 않지만, Small TCN은 다른 비행 run의 GT velocity를 supervision으로 사용한다. 따라서 본 보고서에서는 이를 **learned IMU-only inference**로 표기한다.

---

## 목차

1. [연구 배경과 목표](#1-연구-배경과-목표)  
2. [벤치마크 구조와 상태 정의](#2-벤치마크-구조와-상태-정의)  
3. [비교 필터와 InEKF 구현](#3-비교-필터와-inekf-구현)  
4. [실험 설계와 평가 원칙](#4-실험-설계와-평가-원칙)  
5. [EuRoC 실험](#5-euroc-실험)  
6. [회전량 증가 가설의 통제 실험](#6-회전량-증가-가설의-통제-실험)  
7. [과거 실제 데이터 결과와 재현성 상태](#7-과거-실제-데이터-결과와-재현성-상태)  
8. [CF231 learned velocity + InEKF](#8-cf231-learned-velocity--inekf)  
9. [종합 해석](#9-종합-해석)  
10. [한계와 후속 연구](#10-한계와-후속-연구)  
11. [재현 방법](#11-재현-방법)  
12. [결론](#12-결론)  
13. [참고문헌](#13-참고문헌)  

---

# 1. 연구 배경과 목표

## 1.1 문제 정의

관성항법은 가속도계와 자이로스코프의 측정을 이용해 위치, 속도, 자세를 추정한다. 이상적인 경우에는 자이로로 자세를 적분하고, 그 자세를 이용해 body-frame 가속도를 world frame으로 회전한 뒤 중력을 제거하고, 가속도를 두 번 적분하여 위치를 얻을 수 있다. 실제 IMU에는 bias와 noise가 존재하므로 오차가 계속 누적된다.

가속도계 bias는 직접 속도 오차를 만들고, 장시간에는 위치 오차를 크게 증가시킨다. 자세 오차가 생기면 중력 성분 일부가 수평 가속도로 잘못 투영되므로 작은 roll/pitch 오차도 위치 drift로 연결된다. yaw는 중력만으로 절대 관측되지 않으므로 외부 heading 정보가 없는 일반적인 6자유도 시스템에서는 장시간 안정화가 어렵다.

이 연구는 단순히 “어떤 필터의 RMSE가 더 작은가”를 보는 데서 끝나지 않고, 다음 질문을 분리해 검증하는 것을 목표로 한다.

1. **고정 bias 조건의 순수 IMU 적분은 어느 정도까지 유지되는가?**
2. **동일한 위치 측정을 넣었을 때 필터 구조에 따른 차이는 무엇인가?**
3. **회전 운동이 커질수록 InEKF가 EKF보다 유리해지는가?**
4. **평가 시 다른 센서를 사용하지 않고도 과거 데이터에서 학습한 IMU-to-velocity prior로 drift를 줄일 수 있는가?**
5. **학습 속도를 직접 적분하는 것과 InEKF에 measurement로 결합하는 것은 어떤 trade-off를 만드는가?**

## 1.2 연구 진행 흐름

| 시기 | 수행 내용 | 핵심 판단 |
|---|---|---|
| 6월 말 | 2D/3D 상태, quaternion, rotation matrix, Lie-group 상태 표현 검토 | 공통 15D error-state 규약 확정 |
| 7월 | 고정 bias, IMU-only DR, 위치/자세 시각화, EuRoC 및 비행 데이터 검증 | 평가 중 GT로 bias를 계속 보정하는 방식 배제 |
| 8월 초 | 팀원 filter library 통합, 외부 InEKF 구현 비교, i2Nav/Pohang/UrbanNav 결과 정리 | 일부 historical result의 재현성/독립 reference 문제 확인 |
| 8월 | shallow learned velocity, Small TCN, InEKF velocity update 비교 | 위치와 자세 사이 trade-off 확인 |
| 9월 | 설치형 API, CLI runner, test, 결과 파일, 보고서와 PDF 정리 | 재현 가능한 결과와 historical result를 분리 |

본 보고서는 **현재 저장소에서 재실행 가능한 결과를 1차 근거**로 사용한다. 이전 발표자료에만 남아 있거나 현재 loader/config가 없는 결과는 historical evidence로 따로 구분한다.

---

# 2. 벤치마크 구조와 상태 정의

## 2.1 전체 파이프라인

벤치마크는 dataset loader, 공통 filter API, measurement update, evaluator, result writer를 분리했다. 이 구조의 목적은 필터별로 서로 다른 데이터 전처리나 평가 코드를 사용해 생기는 비교 오류를 줄이는 것이다.

주요 디렉터리는 다음과 같다.

```text
config/             dataset, filter noise, update rate 설정
datasets/           EuRoC, i2Nav, Pohang, CF231 loader
evaluation/         위치/자세 오차 및 plotting
filters/            EKF, UKF, PF, ESKF, InEKF
learned/            Small TCN, CF231 protocol, InEKF velocity update
models/             상태 순서, SO(3)/SE_2(3) utility
runners/            command-line experiment runners
state_estimation/   설치 후 사용하는 public Python API
tests/              filter API / learned pipeline regression tests
reports/data/       보고서 수치 CSV/JSON
reports/figures/    보고서 그림
```

## 2.2 센서 모델

공통 IMU 입력 순서는 다음과 같다.

```text
u_k = [a_x, a_y, a_z, ω_x, ω_y, ω_z]^T
```

단위는 accelerometer가 `m/s^2`, gyroscope가 `rad/s`, 시간 간격이 `s`이다. 일반적인 연속시간 inertial model은 다음과 같이 정리할 수 있다.

\[
\omega_m = \omega + b_g + n_g
\]

\[
a_m = R^T(a_w-g) + b_a + n_a
\]

\[
\dot R = R(\omega_m-b_g)^\wedge, \qquad
\dot v = R(a_m-b_a)+g, \qquad
\dot p = v
\]

여기서 `R`은 body-to-world rotation, `v`는 world-frame velocity, `p`는 position, `b_g`, `b_a`는 gyro/accelerometer bias이다.

## 2.3 공통 상태와 오차 상태

공통 nominal state는 다음과 같다.

```text
X = {R, v, p, b_g, b_a}
```

공통 15차원 local error는 다음 순서를 사용한다.

```text
δx = [δθ, δv, δp, δb_g, δb_a] ∈ R^15
```

외부에서 읽는 pose 출력은 다음 6개 값으로 통일했다.

```text
[p_x, p_y, p_z, roll, pitch, yaw]
```

자세 출력은 radian이며, metric writer에서 degree로 변환해 RPY/heading RMSE를 기록한다.

---

# 3. 비교 필터와 InEKF 구현

## 3.1 비교 대상

| Filter | 자세/오차 표현 | 역할 | 현재 상태 |
|---|---|---|---|
| EKF | quaternion nominal + local linearization | 기본 nonlinear Kalman baseline | EuRoC에서 정상 동작 |
| UKF | quaternion nominal + 15D local error | sigma-point 기반 비교 | 정상 동작, 실행시간 큼 |
| PF | quaternion particle | 비가우시안 비교 | particle 수에 민감 |
| ESKF | rotation matrix + 15D error state | error-state baseline | 현재 EuRoC에서 비정상 발산, 성능표 해석 제외 |
| InEKF | `SE_2(3)` + invariant error | 주요 연구 대상 | analytic Jacobian 기반, 정상 수렴 |

현재 ESKF의 발산은 “ESKF라는 방법 자체의 성능”으로 해석하지 않는다. error injection sign, 좌표계 정의, measurement Jacobian을 다시 검증하기 전까지 diagnostic implementation으로만 남겨 둔다.

## 3.2 InEKF의 핵심 구조

InEKF는 회전, 속도, 위치를 다음 형태의 Lie-group state에 묶어 표현한다.

\[
\chi =
\begin{bmatrix}
R & v & p \\
0 & 1 & 0 \\
0 & 0 & 1
\end{bmatrix}
\in SE_2(3)
\]

본 구현에서는 bias를 포함한 전체 local error를 15차원으로 유지한다. InEKF의 핵심은 적절한 group-affine system에서 invariant error dynamics가 추정 trajectory에 덜 의존하도록 만들 수 있다는 점이다. 이는 linearization의 구조적 일관성을 높일 수 있지만, **모든 데이터셋과 모든 tuning에서 EKF보다 RMSE가 낮아진다는 보장은 아니다.**

특히 prediction-only에서는 EKF와 InEKF가 같은 nominal strapdown dynamics를 적분한다. covariance 정의가 달라도 measurement update가 없다면 nominal state를 교정할 기회가 없으므로 두 필터의 trajectory가 거의 같아지는 것이 자연스럽다.

---

# 4. 실험 설계와 평가 원칙

## 4.1 결과 등급

본 보고서는 결과를 세 등급으로 나눈다.

| 등급 | 정의 | 사용 방식 |
|---|---|---|
| **Primary / reproducible** | 현재 repository의 config와 runner로 다시 생성 가능 | 최종 결론의 핵심 근거 |
| **Historical / archival** | 과거 발표 수치는 남아 있으나 현재 코드/loader/config로 완전 재현되지 않음 | 연구 진행 기록과 가설 형성에만 사용 |
| **Diagnostic / unresolved** | 구현 또는 평가 기준에 문제가 확인됨 | 성능 주장에 사용하지 않음 |

이 구분을 통해 “한 번 잘 나온 수치”보다 **같은 조건으로 다시 만들 수 있는 수치**를 우선한다.

## 4.2 데이터셋과 조건

| 데이터/조건 | 플랫폼 | IMU-only | 외부 update | 역할 | 근거 등급 |
|---|---|---:|---|---|---|
| EuRoC V1_01_easy | MAV / 3D motion | O | GT position + noise, 2 Hz | classical filter 비교 | Primary |
| Synthetic motion regimes | mobile/vessel/drone-like | O | 1 Hz position | 회전량만 통제한 가설 검증 | Primary |
| CF231 Run 5 | 반복 8자 비행 | O | learned velocity | learned inertial estimation | Primary |
| i2Nav street01 historical | mobile robot | 기록 미완전 | F9P GNSS | 과거 장시간 비교 | Historical |
| Pohang05 historical | USV | 기록 미완전 | baseline position | 과거 해양 비교 | Diagnostic/Historical |
| UrbanNav historical | ground vehicle | 기록 미완전 | GNSS/position | 과거 도심 비교 | Historical |

> **주의:** 현재 repository의 i2Nav 기본 config/data layout은 `street00`이다. 8월 4일 historical table은 `street01` 결과이므로 둘을 같은 재현 실험으로 취급하지 않는다.

## 4.3 GT 사용 원칙과 leakage 방지

본 연구에서 GT는 목적에 따라 명확히 구분한다.

- **IMU-only classical evaluation:** 초기 상태 설정과 최종 오차 계산에만 GT를 사용하고, 평가 중 filter update에는 사용하지 않는다.
- **EuRoC fused control experiment:** GT position에 0.05 m 표준편차 noise를 넣어 만든 pseudo position measurement를 2 Hz로 사용한다. 이 실험은 실제 GPS 성능 검증이 아니라 filter update 구조 비교다.
- **CF231 learned method:** 다른 training runs의 GT velocity를 model label로 사용한다. held-out Run 5에서는 GT를 모델 입력이나 filter update에 사용하지 않고 최종 평가에만 사용한다.
- **Pohang historical result:** 현재 loader는 `baseline.txt`를 update와 reference에 모두 사용하므로 독립 localization 성능으로 주장하지 않는다.

## 4.4 공통 평가지표

주요 metric은 다음과 같다.

- Position RMSE / final position error
- Roll, pitch, heading RMSE
- SO(3) geodesic rotation error
- Final SO(3) error
- Runtime

SO(3) 자세 오차는 두 회전행렬 사이의 geodesic angle로 계산한다.

\[
e_R = \cos^{-1}\left(\frac{\operatorname{tr}(R_{gt}^T R_{est})-1}{2}\right)
\]

trajectory는 초기 위치와 초기 yaw를 한 번 맞춘 뒤 평가 중 반복 정렬하지 않는다. 장거리 항법 성능을 더 엄밀히 비교하려면 향후 ATE 외에도 고정 시간/거리 구간의 RPE와 NEES/NIS를 추가해야 한다.

---

# 5. EuRoC 실험

## 5.1 설정

EuRoC `V1_01_easy`를 사용했다.

- 원본 IMU: 200 Hz
- benchmark 입력: 5개마다 1개를 사용하여 40 Hz
- 평가 sample 수: 5,824
- fused position update: 20 IMU sample마다 1회 = 2 Hz
- pseudo position noise: 0.05 m standard deviation
- 비교 필터: EKF, UKF, PF(500 particles), InEKF

실행 명령은 다음과 같다.

```bash
state-estimation-run-all \
  --config config/euroc_imu_only.yaml \
  --filters ekf ukf pf inekf

state-estimation-run-all \
  --config config/euroc.yaml \
  --filters ekf ukf pf inekf
```

## 5.2 결과

| Mode | Filter | Position RMSE [m] | Final position [m] | Roll [deg] | Pitch [deg] | Heading [deg] | Runtime [s] |
|---|---|---:|---:|---:|---:|---:|---:|
| IMU-only | EKF | 1,129.277 | 2,880.71 | 8.405 | 2.657 | 7.721 | 13.20 |
| IMU-only | UKF | 1,225.22 | 3,202.69 | 8.53 | 2.71 | 7.89 | 31.38 |
| IMU-only | PF-500 | 1,263.08 | 3,253.15 | 8.62 | 2.71 | 8.31 | 8.95 |
| IMU-only | InEKF | 1,130.116 | 2,882.19 | 8.405 | 2.657 | 7.721 | **4.85** |
| IMU + 2 Hz position | EKF | **0.1098** | 0.047 | **3.044** | 0.790 | **6.094** | 15.90 |
| IMU + 2 Hz position | UKF | 0.114 | **0.042** | 4.31 | **0.78** | 6.37 | 30.77 |
| IMU + 2 Hz position | PF-500 | 7.186 | 4.088 | 8.46 | 2.53 | 24.25 | 8.60 |
| IMU + 2 Hz position | InEKF | 0.1140 | **0.042** | 4.367 | **0.783** | 6.426 | **8.51** |

![EuRoC comparison](figures/euroc_imu_only_vs_fused.png)

## 5.3 해석

### IMU-only

EKF와 InEKF의 position/attitude 결과가 거의 동일하다. 이는 두 필터가 같은 IMU propagation을 사용하고 measurement update가 없기 때문이다. 결과적으로 InEKF를 사용한다는 사실만으로 bias-driven drift가 제거되지는 않는다.

### Position-aided

2 Hz position update를 넣으면 Kalman 계열 필터의 위치 발산이 크게 제한된다. 현재 설정에서는 EKF가 InEKF보다 position RMSE가 약 **3.8% 낮고**, heading RMSE가 약 **5.4% 낮다**. 반대로 InEKF runtime은 EKF보다 약 **46.5% 짧다**.

따라서 이 실험의 결론은 “InEKF가 정확도에서 우월하다”가 아니다. 더 정확한 표현은 다음과 같다.

> **현재 EuRoC V1_01_easy 통제 조건에서 InEKF는 안정적으로 수렴하고 UKF와 비슷한 정확도를 더 짧은 실행시간에 제공했지만, EKF 대비 position/heading RMSE 우위는 확인되지 않았다.**

---

# 6. 회전량 증가 가설의 통제 실험

## 6.1 실험 목적

실제 mobile robot, USV, drone 데이터는 센서 품질, trajectory length, GNSS rate, 좌표계, 초기화가 모두 다르다. 데이터셋별 RMSE만 비교하면 “플랫폼 차이”와 “필터 구조 차이”를 분리할 수 없다.

따라서 다음 조건은 고정하고 회전량만 증가시키는 synthetic experiment를 만들었다.

- duration: 45 s
- IMU rate: 50 Hz
- 동일 sensor noise / bias
- 동일 1 Hz position update
- mobile-like -> vessel-like -> drone-like 순으로 angular rate와 roll/pitch/z motion만 증가

```bash
state-estimation-motion-benchmark \
  --config config/motion_regimes.yaml
```

## 6.2 결과

| Regime | Angular-rate RMS [rad/s] | Roll/Pitch amplitude [deg] | InEKF IMU-only heading [deg] | InEKF fused heading [deg] | InEKF fused position [m] |
|---|---:|---:|---:|---:|---:|
| mobile-like | 0.312 | 1 / 1 | 0.366 | 2.008 | 0.305 |
| vessel-like | 0.586 | 5 / 4 | 0.748 | 1.686 | 0.334 |
| drone-like | 1.019 | 15 / 10 | 1.162 | 1.781 | 0.316 |

![Motion regime comparison](figures/motion_regime_summary.png)

## 6.3 해석

회전량이 증가할수록 **IMU-only InEKF heading error 자체는 증가**했다. 그러나 fused 조건에서 InEKF의 EKF 대비 상대 우위가 mobile -> vessel -> drone 순서로 단조롭게 증가하지는 않았다. vessel-like heading에서는 InEKF가 유리했지만 mobile-like와 drone-like에서는 그렇지 않았다.

따라서 다음 주장은 현재 결과로 지지되지 않는다.

> “3차원 회전이 많을수록 InEKF가 자동으로 EKF보다 더 정확해진다.”

더 타당한 가설은 InEKF의 이점이 회전 크기 하나만이 아니라 **measurement model의 invariant structure, attitude-velocity-position cross-correlation, 초기 오차, update interval, noise tuning, consistency**와 함께 결정된다는 것이다.

후속 실험에서는 high-angular-rate 구간을 별도 mask로 나누어 heading/SO(3) RMSE와 NEES/NIS를 함께 확인해야 한다.

---

# 7. 과거 실제 데이터 결과와 재현성 상태

## 7.1 8월 4일 historical results

아래 표는 연구 과정에서 얻었던 장시간 데이터 결과를 보존한 것이다. 현재 repository의 최종 성능표와 동일한 의미로 사용하지 않는다.

| Dataset | Metric | EKF | UKF | PF | InEKF | Historical best |
|---|---|---:|---:|---:|---:|---|
| i2Nav street01 | Position RMSE [m] | 2.336 | 2.411 | **2.290** | 2.333 | PF |
|  | Heading RMSE [deg] | 1.388 | 2.698 | 17.552 | **1.058** | InEKF |
| Pohang05 | Position RMSE [m] | 0.0898 | 0.0902 | 0.1210 | **0.0821*** | InEKF* |
|  | Heading RMSE [deg] | 6.703 | 6.821 | 101.721 | **6.442** | InEKF |
| UrbanNav | Position RMSE [m] | 0.3725 | 0.3534 | 2.1210 | **0.2766** | InEKF |
|  | Heading RMSE [deg] | **88.212** | 94.419 | 117.165 | 96.975 | EKF |

`*` Pohang position result는 현재 loader에서 `baseline.txt`가 measurement와 reference 양쪽에 사용되기 때문에 독립적인 localization accuracy로 주장할 수 없다.

## 7.2 결과를 최종 결론에서 분리한 이유

- i2Nav historical table은 `street01`인데 현재 기본 config는 `street00`이다.
- UrbanNav는 과거 loader와 정확한 당시 config가 현재 repository에 남아 있지 않다.
- Pohang은 measurement-reference leakage 문제가 있다.
- 데이터셋마다 evaluation duration, update rate, noise, 초기화가 동일하지 않다.

이 결과는 “InEKF가 어떤 실제 데이터에서 좋은 경향을 보인 적이 있다”는 참고자료로는 의미가 있지만, **동일 조건의 cross-platform benchmark**로 사용하기에는 부족하다.

## 7.3 외부 InEKF 구현과 비교

자체 InEKF를 공개 reference implementation과 여러 데이터셋에서 수치 비교한 기록을 `reports/data/reference_comparison.csv`에 유지한다. EuRoC, KAIST, UZH, Crazy4에서는 position/heading 결과가 대체로 유사했으며, ADVIO는 두 구현 모두 heading error가 매우 커 좌표계 또는 initial yaw 문제를 확인해야 하는 diagnostic case로 분류했다.

이 비교의 목적은 “reference implementation보다 우수함”을 주장하는 것이 아니라, **자체 predict/update가 외부 구현과 유사한 방향으로 동작하는지 sanity check**하는 것이다.

---

# 8. CF231 learned velocity + InEKF

## 8.1 문제 설정

고정 bias를 적용한 pure IMU dead reckoning은 CF231 장시간 비행에서도 크게 발산했다. 이에 따라 과거 비행의 IMU pattern과 velocity 관계를 학습하고, held-out run에서는 IMU만으로 velocity를 예측하는 경량 모델을 추가했다.

중요한 조건은 **train/test split을 비행 run 단위로 분리**한 것이다.

- Training runs: 3, 4, 9, 10
- IMU noise / gyro intrinsic estimation: 3, 9, 10
- Held-out evaluation: Run 5
- Input window: 200 IMU samples, 약 2 s
- Input: 6-axis IMU
- Model: Small TCN, 36,003 parameters
- Output: heading-frame velocity
- Run 5 runtime sensors: IMU only
- Run 5에서 사용하지 않는 값: GT state update, GNSS, camera, PWM, thrust, motion-range constraint

Small TCN의 label은 training runs의 GT velocity이다. 따라서 이 방법은 learned prior를 사용하는 inference이며, 학습 없는 pure inertial navigation과 구분한다.

## 8.2 Run 5 fixed bias

Run 5 시작 후 약 2.63 s의 정지 구간에서 bias를 한 번 계산한 뒤 전체 평가 구간에 고정한다.

```text
gyro bias  = [-3.308e-4, -9.214e-5,  3.601e-5] rad/s
accel bias = [ 0.41543,   0.14960,    0.00771 ] m/s^2
```

평가 중 GT에 맞춰 bias를 시간에 따라 다시 추정하지 않는다.

## 8.3 Small TCN

Small TCN은 약 2초 길이의 IMU temporal pattern에서 속도를 예측한다. 핵심은 Run 5의 position을 미분해서 virtual velocity를 만드는 것이 아니라, **과거 runs에서 학습한 IMU-to-velocity mapping을 held-out Run 5 IMU에 적용**한다는 점이다.

```text
200 × 6 IMU window
        ↓
Small TCN (36,003 parameters)
        ↓
heading-frame velocity + prediction covariance
        ↓
world-frame velocity
        ↓
(1) direct integration
or
(2) InEKF velocity measurement update
```

## 8.4 두 가지 결합 방식

### A. Small TCN loose integration

예측 velocity를 직접 적분하여 position을 만든다. attitude는 fixed-bias IMU propagation을 사용한다. Kalman innovation을 통한 자세 교정은 수행하지 않는다.

### B. Small TCN + InEKF

동일한 predicted velocity를 다음 virtual measurement로 넣는다.

\[
z_v = v + n_v
\]

measurement covariance는 training runs의 cross-validation residual로 추정한다. InEKF는 velocity residual뿐 아니라 state covariance의 cross term을 통해 attitude와 position도 함께 교정한다.

## 8.5 결과

| Method | Position RMSE [m] | Final position [m] | SO(3) RMSE [deg] | Final SO(3) [deg] | Roll/Pitch/Yaw RMSE [deg] |
|---|---:|---:|---:|---:|---|
| Fixed-bias IMU DR | 522.2007 | 1,005.4296 | 3.5535 | 8.7520 | 1.41 / 3.15 / 0.83 |
| Shallow velocity + InEKF | 4.39 | 7.75 | 4.03 | 5.07 | 1.16 / 0.87 / 3.76 |
| Small TCN loose | **2.3955** | **2.8399** | 3.5535 | 8.7520 | 1.41 / 3.15 / 0.83 |
| Small TCN + InEKF | 2.6131 | 3.2410 | **1.7964** | **0.4605** | 1.10 / 0.84 / 1.14 |

![CF231 learned comparison](figures/cf231_learned_comparison.png)

## 8.6 정량적 해석

Small TCN loose integration은 fixed-bias IMU DR 대비 position RMSE를 약 **99.54% 감소**시켰다. 이는 held-out Run 5에서 learned velocity가 장기 위치 drift를 매우 크게 줄였음을 보여준다.

Small TCN + InEKF는 loose integration보다 position RMSE가 약 **9.1% 높다**. 따라서 현재 결과에서 “InEKF fusion이 position accuracy를 더 높였다”고 주장하면 안 된다.

반면 SO(3) RMSE는 3.5535 deg에서 1.7964 deg로 약 **49.5% 감소**했고, final SO(3) error는 8.7520 deg에서 0.4605 deg로 약 **94.7% 감소**했다. 즉 InEKF의 효과는 최고 position score보다 **attitude와 전체 state consistency의 장기 안정화**에서 뚜렷하게 나타났다.

또한 yaw만 보면 fixed-bias/loose TCN의 RMSE가 약 0.83 deg로 TCN+InEKF의 약 1.14 deg보다 낮다. 따라서 “모든 자세 축이 개선되었다”는 표현도 부정확하다. SO(3) 개선은 특히 pitch와 장기 final orientation stability의 개선이 크게 기여한다.

---

# 9. 종합 해석

## 9.1 확인된 사실

1. **고정 bias만 적용한 pure IMU position은 장시간 유지되지 않는다.** EuRoC와 CF231 모두에서 확인됐다.
2. **Position update는 position drift를 크게 제한한다.** 다만 heading을 직접 관측하지 않는 경우 heading accuracy는 filter structure, cross-correlation, motion, tuning에 영향을 받는다.
3. **InEKF는 현재 EuRoC에서 EKF보다 항상 정확하지 않다.** 오히려 현재 fused setting에서는 EKF가 position/heading RMSE가 조금 낮다.
4. **InEKF의 runtime은 현재 구현에서 비교적 짧다.** EuRoC fused에서 EKF 대비 약 46.5% 짧았다.
5. **회전량이 커진다는 사실만으로 InEKF의 상대 정확도 이점이 커지지는 않았다.**
6. **Held-out CF231에서 Small TCN velocity는 pure IMU drift를 큰 폭으로 줄였다.**
7. **Small TCN velocity를 InEKF에 결합하면 position RMSE는 소폭 악화되지만 SO(3) attitude가 크게 안정화됐다.**

## 9.2 현재 결과가 보여주는 InEKF의 역할

본 벤치마크에서 InEKF의 가치는 “언제나 가장 작은 RMSE를 내는 필터”로 요약하기 어렵다. 오히려 다음과 같이 정리하는 편이 정확하다.

- prediction-only: nominal trajectory는 EKF와 거의 동일 -> drift 해결 불가
- position-aided: 정상적으로 수렴하며 UKF와 유사한 accuracy, 짧은 runtime
- controlled rotation: 회전량만으로 우위 설명 불가
- learned velocity fusion: velocity residual을 covariance-aware state correction에 사용하여 SO(3) attitude 안정화에 기여

즉 InEKF의 구조적 장점은 **measurement와 error definition이 잘 맞을 때 state coupling을 일관되게 활용하는 것**에서 찾아야 한다.

## 9.3 연구적으로 가장 의미 있는 결과

단순한 “filter ranking”보다 다음 두 결과가 더 중요하다.

첫째, 회전이 많으면 InEKF가 무조건 좋아질 것이라는 직관적 가설을 통제 실험으로 시험했고, 현재 조건에서는 그 가설을 그대로 지지할 수 없다는 사실을 확인했다. 이는 negative result이지만 후속 실험 변수를 명확하게 만든다.

둘째, held-out flight에서 learned velocity와 InEKF를 결합해 **position drift 감소와 attitude consistency 사이의 trade-off**를 수치로 분리했다. 이 결과는 향후 maritime/multi-IMU 환경에서 learned measurement를 어떻게 활용할지 구체적인 방향을 제공한다.

---

# 10. 한계와 후속 연구

## 10.1 현재 주장할 수 없는 내용

다음 주장은 현재 evidence로는 충분히 지지되지 않는다.

1. mobile robot -> USV -> drone 순으로 motion complexity가 증가할수록 InEKF advantage가 증가한다.
2. Pohang historical position RMSE 약 8 cm가 독립 reference에 대한 실제 localization accuracy다.
3. CF231에서 학습한 TCN이 다른 플랫폼이나 다른 IMU로 calibration 없이 일반화된다.
4. 현재 발산하는 ESKF 결과가 ESKF theory의 성능을 대표한다.
5. learned IMU-only inference와 학습 없는 pure inertial dead reckoning이 같은 의미다.

## 10.2 우선순위가 높은 후속 실험

### Priority 1 - 동일 조건의 실제 cross-platform benchmark

mobile robot, USV, drone에 대해 다음 조건을 최대한 동일하게 맞춘다.

- 동일 GNSS/position update rate
- 동일하게 정의한 measurement noise
- 독립 reference trajectory
- 동일 evaluation duration 또는 distance
- 동일 initialization protocol
- 같은 metric set

이를 통해 sensor/dataset 차이와 filter 차이를 분리할 수 있다.

### Priority 2 - high-rotation segment evaluation

전체 trajectory RMSE만 보지 않고 angular-rate threshold를 사용해 고회전 구간을 별도 평가한다.

- Heading RMSE
- SO(3) RMSE
- Position RPE
- NIS
- NEES

이 실험이 InEKF의 consistency advantage를 직접 검증하는 데 더 적합하다.

### Priority 3 - adaptive covariance

고정된 `Q`와 `R` 대신 motion intensity와 innovation consistency에 따라 covariance를 조정한다.

- NIS chi-square gate 기반 adaptive `R`
- IMU vibration/angular-rate 기반 adaptive `Q`
- learned velocity residual covariance의 online calibration

### Priority 4 - learned model generalization

CF231 single held-out run을 넘어 다음을 확인해야 한다.

- leave-one-run-out 전체 반복
- 다른 trajectory pattern
- 다른 vehicle/platform
- IMU mounting/calibration 변화
- model size/latency와 accuracy trade-off

### Priority 5 - ESKF implementation verification

현재 ESKF는 다음 항목을 unit test로 분리해야 한다.

- error injection sign
- left/right perturbation convention
- rotation frame
- measurement Jacobian
- covariance reset Jacobian

검증이 끝난 뒤에만 최종 filter comparison에 다시 포함한다.

---

# 11. 재현 방법

## 11.1 설치

```bash
git clone https://github.com/INHA-Artemis/inertial-state-estimation-benchmark.git
cd inertial-state-estimation-benchmark

python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
pip install -e .
```

Small TCN까지 실행하려면 다음 extras를 설치한다.

```bash
pip install -e ".[learned]"
```

test까지 포함할 경우:

```bash
pip install -e ".[test,learned]"
python -m pytest -q
```

## 11.2 데이터 배치

원시 데이터는 repository에 포함하지 않는다.

```text
data/
├── euroc/V1_01_easy/mav0/
├── i2nav_robot/street00/
├── pohang_canal/pohang05/
└── cf231_leave_one_out/csv/
```

정확한 파일명과 download source는 [`DATASETS.md`](../DATASETS.md)를 따른다.

## 11.3 핵심 실험 명령

### EuRoC IMU-only

```bash
state-estimation-run-all \
  --config config/euroc_imu_only.yaml \
  --filters ekf ukf pf inekf
```

### EuRoC IMU + 2 Hz pseudo-position

```bash
state-estimation-run-all \
  --config config/euroc.yaml \
  --filters ekf ukf pf inekf
```

### Motion-regime controlled benchmark

```bash
state-estimation-motion-benchmark \
  --config config/motion_regimes.yaml
```

### CF231 Small TCN + InEKF

```bash
state-estimation-cf231-tcn \
  --dataset data/cf231_leave_one_out/csv \
  --output outputs/cf231_small_tcn
```

### Single filter

```bash
state-estimation-run \
  --filter inekf \
  --config config/euroc.yaml
```

## 11.4 결과 파일

일반 runner는 다음 구조로 결과를 저장한다.

```text
outputs/<dataset>/<sequence>/<filter>/
├── estimate.csv
├── metrics.json
├── runtime.json
├── trajectory.png
└── error.png
```

CF231 learned experiment는 다음 파일을 저장한다.

```text
summary.json
trajectories.npz
small_tcn.pt
trajectory_comparison.png
```

보고서에 사용한 정리 수치는 다음에 보존한다.

```text
reports/data/euroc_results.csv
reports/data/motion_regime_metrics.csv
reports/data/cf231_run5_summary.json
reports/data/cf231_learned.csv
reports/data/historical_full_sequence.csv
reports/data/reference_comparison.csv
```

---

# 12. 결론

이 연구에서는 여러 관성 상태추정 필터를 동일한 API와 평가 흐름으로 묶고, pure IMU dead reckoning, position-aided filtering, controlled rotation experiment, learned velocity-aided InEKF를 한 저장소에서 재현할 수 있도록 정리했다.

핵심 결과는 세 가지다. 첫째, 고정 bias를 사용한 일반적인 IMU-only position은 EuRoC와 CF231에서 장시간 크게 발산했다. 둘째, InEKF는 EuRoC position-aided 조건에서 안정적으로 동작하고 runtime 장점을 보였지만 EKF보다 accuracy가 항상 우수하지 않았으며, 회전량을 증가시킨 합성 실험에서도 상대 우위가 단조롭게 커지지 않았다. 셋째, CF231 held-out Run 5에서 Small TCN velocity는 position drift를 크게 줄였고, 이를 InEKF에 결합했을 때 position RMSE는 loose integration보다 약간 증가하는 대신 SO(3) attitude error가 크게 감소했다.

따라서 본 연구의 최종 결론은 “InEKF가 모든 조건에서 가장 정확하다”가 아니다. 더 중요한 성과는 **비교 조건과 GT 사용 방식을 명확히 분리한 재현 가능한 benchmark를 만들고, InEKF가 언제 의미 있는 state correction을 제공하는지 실험적으로 구분한 것**이다.

다음 단계는 실제 mobile robot, USV, drone에 대해 독립 reference와 동일 GNSS 조건을 사용한 benchmark를 구성하고, high-rotation segment의 NIS/NEES와 RPE까지 포함해 InEKF의 consistency를 검증하는 것이다. 동시에 CF231에서 확인한 learned velocity measurement를 multi-run 및 cross-platform 환경으로 확장해, runtime IMU-only 상태추정의 일반화 가능성을 평가해야 한다.

---

# 13. 참고문헌

1. A. Barrau and S. Bonnabel, **“The Invariant Extended Kalman Filter as a Stable Observer,”** *IEEE Transactions on Automatic Control*, 2017. https://arxiv.org/abs/1410.1465
2. M. Burri et al., **“The EuRoC Micro Aerial Vehicle Datasets,”** *International Journal of Robotics Research*, 2016. https://www.research-collection.ethz.ch/entities/researchdata/bcaf173e-5dac-484b-bc37-faf97a594f1f
3. D. Chung et al., **“Pohang Canal Dataset: A Multimodal Maritime Dataset for Autonomous Navigation in Restricted Waters,”** 2023. https://github.com/dhchung/pohang_canal_dataset
4. i2Nav-WHU, **i2Nav-Robot dataset.** https://github.com/i2Nav-WHU/i2Nav-Robot
5. W. Wen et al., **“UrbanNav: An Open-Sourced Multisensory Dataset for Benchmarking Positioning Algorithms Designed for Urban Areas,”** ION GNSS+, 2021. https://github.com/weisongwen/UrbanNavDataset
6. W. Liu et al., **“TLIO: Tight Learned Inertial Odometry,”** *IEEE Robotics and Automation Letters*, 2020. https://arxiv.org/abs/2007.01835
7. S. Bai, J. Z. Kolter, and V. Koltun, **“An Empirical Evaluation of Generic Convolutional and Recurrent Networks for Sequence Modeling,”** 2018. https://arxiv.org/abs/1803.01271
8. G. H. Haggerty, **Invariant EKF reference implementation for a high-speed racing UAV.** https://github.com/ghaggin/invariant-ekf
9. INHA-Artemis, **Inertial State Estimation Benchmark repository.** https://github.com/INHA-Artemis/inertial-state-estimation-benchmark

---

