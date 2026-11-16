# KRoC Fixed-Wing VIO Sim

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

출력: `A/`, `B/`, `C/`, `comparison.csv`, `comparison.json`, `trajectories.png`.
연결 규약은 [docs/CONTRACTS.md](docs/CONTRACTS.md). 테스트: `./run.sh test`.
