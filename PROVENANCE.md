# 코드 출처와 추가한 내용

## 필터 코드의 기반

- 팀원 저장소: <https://github.com/andyjaehun/State_Estimation>
- 가져온 commit: `0967c4737f803aad897e1ba6e16211fb26288821`
- commit 날짜: 2026-08-13 14:34:16 +09:00
- 이 저장소에 옮긴 날짜: 2026-09-03

팀원 저장소에서 EKF, UKF, PF, ESKF, InEKF의 공통 인터페이스와 기본 상태 모델, 일부 데이터 loader와 평가 코드를 가져왔습니다.

## 이번에 추가하거나 고친 부분

- 설치 후 `create_filter()`로 모든 필터를 생성할 수 있는 Python API
- EuRoC IMU-only와 2 Hz 위치 update 설정
- 회전 운동만 단계적으로 바꾸는 합성 실험
- CF231 Small TCN 학습과 InEKF velocity update
- 명령 한 번으로 학습, 평가, 결과 저장까지 수행하는 실행기
- 결과 CSV·JSON과 그래프 생성 코드
- 실험 방법, 데이터 배치, 결과 해석을 설명한 문서와 보고서

`reports/data/historical_full_sequence.csv`와 `reference_comparison.csv`의 값은 8월 4일 발표자료에서 가져왔습니다. 당시 실험 설정과 출력 파일이 모두 남아 있지 않아 현재 코드의 최종 결과에는 포함하지 않고, 이전 진행 내용을 확인하는 용도로만 보관합니다.
