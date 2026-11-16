# 연결 계약

## 데이터와 실행

- 모든 비교는 **같은 JSBSim plant와 같은 센서 기록**을 쓴다. A/B/C별로 비행을 다시 생성하지 않는다.
- `scripts/generate_flight.py`: JSBSim → GT·IMU·카메라 포즈. 기체 XML은 `sim/aircraft/`에 추가한다.
  현재 controller/trim은 C172p 연동 시험용이다. 새 기체에는 전용 controller/trim을 연결한다.
- `scripts/render_airsim.py`: 포즈 → UE/Colosseum ComputerVision 영상. `config/airsim_render_only.json` 사용.
  `--origin-in-airsim N E D`는 level에서 측정한 **첫 CG의 실제 AirSim 위치**다. AGL과 NED z를 혼동하지 않는다.
- `pipeline run`은 센서·calibration만 `runs/<name>/sensor_input/`에 복사한다.
  GT/카메라 포즈/공력 디버그 로그는 estimator에 전달하지 않는다. 입력 hash를 각 실행 전후 검사한다.
- `config/methods.yaml`의 `command`는 shell 문자열이 아닌 argv다.
  `{input}`, `{output}`, `{config}`, `{root}`를 치환한다. timeout 1시간, 로그는 각 `runner.log`.
- 실제 MSCKF runner는 method YAML의 dynamics/cfd 설정을 적용하고,
  `{output}/trajectory.csv`에 아래 열을 기록한다. 초기화/실패 시각도 runner log에 남긴다.

```csv
t_ns,px,py,pz,qw,qx,qy,qz
```

세계는 첫 CG의 고정 geodetic tangent NED, IMU는 CG의 FRD, quaternion은 IMU→world의 wxyz다.
시각은 공통 시작부터 정수 ns, 위치는 m다. 카메라는 optical right/down/forward,
`T_IC`는 optical→IMU이고 `T_WC=T_WI*T_IC`다. 카메라 포즈 CSV에는 장착 변환이 이미 포함돼 있다.
센서는 200 Hz physics에서 100 Hz IMU/10 Hz 영상으로 뽑는다. IMU는 flat-local 이상 모델이며
동일 GT의 세계 속도·자세 미분으로 만든다. 실제 Earth-rate/진동/랜덤워크 모델은 포함하지 않는다.

## 추정기 세 개

- A: 기존 baseline 그대로. `dynamics.enabled=false`, `cfd.enabled=false`.
- B: visual update 뒤 `dynamics/constraint.py`의 beta residual을 별도 EKF update로 적용.
- C: 같은 update에 `cfd/model.py`의 beta 기준/분산을 사용. CFD는 별도의 두 번째 update가 아니다.

`linearize(v_W,R_WB,...)`의 H는 오차 `[dp_W,dv_W,dtheta_B]`,
`R_new=R*Exp(dtheta)` 기준이다. 실제 MSCKF의 회전 오차 정의에 맞게 변환하고 유한차분 검증한다.
함수는 잔차 `reference-prediction`, H는 prediction 미분을 반환한다. 저속은 업데이트를 생략한다.
LUT 조건은 업데이트 직전의 **추정 상태**에서 계산해 고정하는 근사다. 바람 처리는 초기 무풍부터 검증한다.

CFD 모델 폴더:

```text
metadata.yaml  # aircraft_id, provenance, calibration_split, demo_only:false, sigma_floor_rad
beta_lut.csv   # bank_rad,airspeed_mps,beta_ref_rad,sigma_beta_rad
```

bank×speed 완전 격자와 양의 sigma가 필요하다. LUT 범위 밖은 보수적인 B fallback을 쓴다.
CFD 원본 6축 계수·질량·관성·기준점/단위/축·감쇠/추진계 출처는 모델과 함께 보관한다.
plant 계수와 LUT 보정용 기동, 최종 평가 기동을 구분한다. 실제 C 실행은 모델이 없거나 demo LUT면 거부한다.

## 평가

`evaluation/metrics.py`: 첫 공통 포즈에서 translation+yaw만 한 번 정렬한다. scale fit/구간별 재정렬은 없다.
카메라 시각의 ATE RMSE, 1 s RPE(병진/회전 RMSE·p95), 유효 출력률을 비교한다.
기본 150 ms를 넘는 추정 간격은 보간하지 않고, tracking outage를 가로지르는 RPE도 제외한다.
누락·실패 방법은 summary에 0 출력률과 사유를 남긴다. 사용한 정렬 시각과 RPE pair 수도 기록한다.

`pipeline demo`는 synthetic 3D 점 영상 + **IMU-only demo**로 입출력 연결만 검사한다.
시각 frontend/MSCKF와 CFD 해석은 들어 있지 않다. 모든 demo report는 `scientific_result=false`다.
