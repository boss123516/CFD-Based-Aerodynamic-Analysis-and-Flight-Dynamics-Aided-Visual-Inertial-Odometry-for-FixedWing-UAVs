# 바람·노이즈와 실측 3D 맵

`config/strasbourg.yaml`에서 비행·센서 조건, `config/map.yaml`에서 지도 소스를 지정한다.
`./run.sh map` 화면에서도 평균풍·난류·돌풍·gyro/가속도 백색잡음 크기를 조절할 수 있다.

| 설정 | 의미 |
|---|---|
| `wind.mean_ned_mps` | 북/동/아래 축 공기 질량의 지상 상대 속도, m/s |
| `wind.turbulence.model` | `none`, `dryden_milspec`, `dryden_tustin` |
| `windspeed_20ft_mps`, `severity` | Dryden 파라미터, severity 0..7. sigma와 일대일 대응하지 않음 |
| `wind.gusts` | 시작·상승·유지·하강 시간과 NED 돌풍 벡터. 코사인 전이 |
| `wind.seed` | JSBSim 난류 난수 시드 |
| `noise.*_white_std_*` | **샘플별** 측정 표준편차. noise density가 아님 |
| `noise.*_bias_*` | 초기 상수 bias |
| `noise.*_bias_random_walk_*_sqrt_s` | bias 증가량 = 계수 × sqrt(dt) × N(0,1) |

바람은 JSBSim 공기 상대 속도·공력·비행 응답에 작용한다. 정지된 건물 메시가 바람에 흔들리는 모델은 아니다.
트림은 평균풍에서 구한 뒤 난류를 활성화한다. IMU는 같은 비행의 GT에서 생성하며 AirSim IMU를 섞지 않는다.
풍속·난류·돌풍·실제 bias는 평가/디버그 CSV에 보존하고 온라인 estimator에 전달하지 않는다.
같은 시드의 바람·IMU 로그 재현성을 테스트했다. 카메라 blur·픽셀 잡음은 현재 적용하지 않는다.

실측 맵은 Strasbourg 2022 항공사진으로 복원한 **텍스처 삼각형 메시**다.
출처: [Strasbourg open data](https://data.strasbourg.eu/explore/dataset/photomaillage_portail/information/),
공급: Ville et Eurométropole de Strasbourg, FEDER, DataGrandEst. Licence Ouverte v2.0.
화면과 데이터셋 metadata에 출처를 남긴다. CesiumJS 1.124.0 사용이며 Unreal 실행과 구별한다.

각 이미지 시각에 카메라 포즈를 고정하고 목표 LOD 타일 완료를 기다린다.
실제 렌더러 카메라의 ECEF 위치·방향과 JSBSim 카메라 포즈, pinhole 재투영을 검사한다.
PNG는 640×480, 10 Hz **시뮬레이션 시각**으로 저장한다. 취득 벽시계 속도는 타일 로딩에 따라 느려질 수 있다.
cache 파일 hash, 이미지 hash, 특징점 수, 포즈/재투영 오차는 각 데이터셋에 기록한다.
지형은 시각 렌더링용이다. JSBSim ground reaction/지형 충돌에는 아직 연결하지 않았다.

공식 구현: [JSBSim 1.3.3 FGWinds enum/설정](https://github.com/JSBSim-Team/jsbsim/blob/v1.3.3/src/models/atmosphere/FGWinds.h),
[난류·시드 구현](https://github.com/JSBSim-Team/jsbsim/blob/v1.3.3/src/models/atmosphere/FGWinds.cpp),
[Cesium 타일 완료 이벤트](https://cesium.com/learn/cesiumjs/ref-doc/Cesium3DTileset.html).
이 버전의 enum을 확인해 Milspec=3, Tustin=4를 사용한다.
