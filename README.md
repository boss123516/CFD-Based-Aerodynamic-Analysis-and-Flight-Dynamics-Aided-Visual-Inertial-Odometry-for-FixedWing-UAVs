# CFD-Based Aerodynamic Analysis and Flight-Dynamics-Aided Visual-Inertial Odometry for Fixed-Wing UAVs

## Sim

[![Strasbourg 3D flight capture](assets/sim/strasbourg-flight.gif)](assets/sim/strasbourg-flight.mp4)

[MP4 demo](assets/sim/strasbourg-flight.mp4) · 실측 3D 도시 메시 → CesiumJS 영상 + JSBSim IMU/GT.
12초 비행에서 영상 121장·IMU 1,201개 취득. 평균풍·돌풍·Dryden 난류·IMU 잡음 설정 가능.
이 영상은 센서 취득 데모이며, VIO/CFD 성능 결과가 아니다.

[Map data](https://data.strasbourg.eu/explore/dataset/photomaillage_portail/information/): Ville et Eurométropole de Strasbourg — Données produites avec le soutien du FEDER et de DataGrandEst.

JSBSim → 센서/영상 → A/B/C → 공통 평가.

| 방법 | 구성 |
|---|---|
| A | 기본 VIO/MSCKF |
| B | A + 고정익 동역학 제약 |
| C | B + CFD 기반 모델 |

세 방법은 같은 비행·영상·IMU를 공유한다.

```bash
./setup.sh
./run.sh pipeline demo --output runs/demo01
```

데모는 JSBSim·합성 점 영상·IMU-only runner로 전체 연결을 점검한다.
**실제 VIO/CFD 성능 결과가 아니다.** 실제 코드·모델은 아래 위치에 연결한다.

```text
scripts/                  JSBSim 생성, UE/Colosseum 렌더링, 실행
sim/aircraft/             기체별 JSBSim 모델
estimators/baseline/      기존 MSCKF 실행 어댑터
config/methods.yaml       A/B/C 실행 명령·설정
dynamics/               동역학 residual/Jacobian
cfd/                     CFD 모델/LUT (demo_lut는 연결 시험 전용)
evaluation/              ATE·RPE·출력률
runs/                    실행 결과 (Git 제외)
```

실제 데이터 실행:

```bash
./run.sh generate --output datasets/flight01
./run.sh render datasets/flight01 --origin-in-airsim N E D
# config/methods.yaml에 estimator command와 C 모델 경로 지정
./run.sh pipeline run --dataset datasets/flight01 --output runs/exp01
```

`N E D`는 Unreal level에서 확인한 첫 CG의 AirSim 위치다.
기존 UE 프로젝트는 `config/airsim_render_only.json`으로 실행한다.
실제 렌더러 연결은 아직 검증 전이며, CFD 모델은 추후 추가한다.
C 모델이 없으면 실제 실행은 blocked로 기록한다.

실측 3D 도시 맵으로 영상·IMU 취득 (Strasbourg 항공사진 기반 메시, CesiumJS):

```bash
bash scripts/setup_map.sh
./run.sh map --record
```

화면에서 평균풍·난류·돌풍·IMU 잡음을 설정한다. PNG/IMU/GT는 `datasets/strasbourg_*`에 저장한다.
이 렌더러는 Unreal과 별개다. 설정·출처·검증: [docs/WIND_NOISE.md](docs/WIND_NOISE.md).

비교 출력: `A/`, `B/`, `C/`, `comparison.csv`, `comparison.json`, `trajectories.png`.
연결 규약은 [docs/CONTRACTS.md](docs/CONTRACTS.md). 테스트: `./run.sh test`.

## Research aircraft

SUPER STINGRAY를 연구 기체로 준비한다. 유료 CAD·제작 파일·치수·기체별 공력/동역학 데이터는 별도 비공개 작업 공간에 보관한다. 이 저장소에는 시뮬레이터 코드와 공개가 허용된 이미지·영상만 포함한다. 기체별 CFD/비행 모델 검증은 아직 진행 전이다.
