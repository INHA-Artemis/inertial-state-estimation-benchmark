# 변경 내용

## 0.1.0 — 2026-09-05

- EKF, UKF, PF, ESKF, InEKF를 하나의 Python API로 실행하도록 정리
- 6축 IMU 입력과 6차원 pose 출력 형식 통일
- EuRoC IMU-only와 위치 update 실험 설정 추가
- 회전 운동을 세 단계로 바꾼 합성 실험 추가
- CF231 Small TCN 학습과 InEKF velocity update 추가
- 학습에서 제외한 Run 5의 평가 결과 저장
- 결과 CSV·JSON, 그래프 생성 코드, 사용 설명과 연구 보고서 추가
- EuRoC에서 발산한 ESKF와 Pohang의 update/reference 중복 사용 문제 기록
