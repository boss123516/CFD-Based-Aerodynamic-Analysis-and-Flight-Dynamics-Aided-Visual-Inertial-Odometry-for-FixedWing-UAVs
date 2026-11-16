# Strasbourg VIO scenarios (CFD excluded)

All sequences use the same real Strasbourg 2022 photogrammetric city mesh. Difficulty increases aircraft motion, while camera calibration/rates, wind and IMU noise stay common. This does not guarantee that estimator error increases monotonically: stronger excitation can also improve initialization/observability.

| Scenario | Horizontal path | Reference airspeed | Reference altitude h-sl | Bank limit |
|---|---|---|---|---|
| easy | Circle, 220m radius | 22m/s | 322m | 20° |
| mid | Ellipse, 300m / 200m radii | 22 ± 1m/s | 292 ± 25m | 30° |
| hard | Figure-eight, 360m / 200m amplitudes | 22 ± 3m/s | 262 ± 35m | 40° |

Initial location: 48.578°N, 7.740°E. The local terrain reference is approximately 142m; altitude above this reference is not guaranteed AGL / terrain following. The bank limits are commands, with small controller overshoot recorded in actual statistics. Roll command slew limits increase 2 / 5 / 12°/s to avoid an abrupt easy-mode entry. Hard includes bank reversals and 60s speed/altitude modulation; mid uses slower 100s speed / 150s altitude modulation.

Each sequence lasts **600s**, with **12,001 mono 640×480 images at 20Hz** and **120,001 IMU samples at 200Hz**, including both endpoints. Raw image payload alone is 3,686,707,200 bytes per uncompressed bag. PNG/LZ4 size depends on texture. Actual speed ranges/mean, distance, altitude, angular rates, aircraft assumptions and byte counts are recorded locally. Dataset/config snapshots and aircraft profiles are Git-ignored.

## Generate / acquire

```bash
bash scripts/setup_map.sh
.venv/bin/python -m pip install -r requirements-bag.lock.txt
./run.sh scenarios --aircraft-profile /absolute/private/flight_profile.private.yaml --output datasets/city01
./run.sh capture-scenarios datasets/city01
./run.sh status datasets/city01
# Run capture-scenarios again to resume interrupted image acquisition.
# Interactive scenario/altitude/speed/wind controls:
./run.sh map --aircraft-profile /absolute/private/flight_profile.private.yaml
```

Edit `config/scenarios.yaml` for duration, location, path shape/radius, speed/altitude changes, sensor calibration/rates, wind and noise. The private aircraft profile supplies mass, inertia, geometry and provenance. `aircraft_model.py` generates an approximate JSBSim flying-wing model outside Git. Published dimensions plus assumed mass/inertia, analytic lift/drag/moment coefficients and ideal actuators/thrust are **not a validated flight model of the purchased aircraft**. No CFD is performed. Physical maximum speed stays unknown unless measured; a simulation speed limit is an assumption.

The renderer waits for the configured map LOD, validates camera pose/intrinsics and records mesh/sky coverage and image hashes. Large banks may legitimately put the sky in view; missing below-horizon map coverage is rejected. Source URLs, attribution, map cache hashes, resolved config and sensor counts are retained. Wall-clock rendering can be substantially slower than the 10-minute simulated flight.

## Bag / calibration / timing

Default output: ROS1 `sensors.bag`; `--ros-version 2`: Humble-compatible SQLite bag. Topics: `/imu0` (`sensor_msgs/Imu`), `/cam0/image_raw` (`sensor_msgs/Image`, mono8), `/cam0/camera_info`. ROS messages use **FLU IMU / optical camera**; ground truth uses **ENU / FLU**. Source CSVs remain NED / FRD. IMU orientation is unavailable (`orientation_covariance[0] = -1`); ground-truth pose is not supplied to the estimator.

```bash
./run.sh bag datasets/city01/easy --output datasets/city01/easy/offset_test.bag \
  --camera-clock-offset-ns 12000000 --imu-clock-offset-ns -3000000 \
  --camera-latency-ns 25000000 --stamp-mode corrected
```

`raw_header = epoch + capture + sensor_clock_offset`; `bag_record = epoch + capture + delivery_latency`. `corrected` removes known clock offsets from headers. `raw` preserves them for time-calibration tests. Kalibr/OpenVINS convention: `t_imu = t_cam + timeshift_cam_imu`; raw timeshift is `imu_offset - camera_offset`. Rendering wall time never sets sensor timestamps. Every message is independently deserialized after writing to check counts, order, exact header/record timestamps and corrected grids. Default epoch is 1s and capture times span 0..600s. Images use instantaneous global shutter; exposure integration / rolling shutter / blur are not simulated.

Each bag has a `<bag-name>.calibration/` bundle containing `camchain_ros.yaml`, `imu_ros.yaml`, `groundtruth_enu_flu.tum` and `initial_state_evaluation_only.yaml`. The initial-state file is an optional evaluation oracle: disclose its use and apply it equally across methods. Sequences start **in flight**: OpenVINS needs dynamic initialization; the referenced MSCEqF provides static and parametric initialization. Do not initialize an airborne flight as zero velocity. Increase feature depth limits for the actual flight height (the MSCEqF example defaults to 20m). No MSCKF/MSCEqF accuracy claim is made before replay succeeds.

`scene_quality.json` records fixed-threshold FAST counts, Shi–Tomasi coverage and adjacent-frame forward/backward KLT survival. Low-feature/sky frames are retained. This is a diagnostic, not a VIO accuracy result.

[Map source and Licence Ouverte v2](https://data.strasbourg.eu/explore/dataset/photomaillage_portail/information/): Ville et Eurométropole de Strasbourg — Données produites avec le soutien du FEDER et de DataGrandEst. Retain the attribution and provider/funder logos when publishing imagery. [ROS bag writer](https://ternaris.gitlab.io/rosbags/) · [OpenVINS initialization](https://docs.openvins.com/classov__init_1_1InertialInitializer.html) · [MSCEqF](https://github.com/aau-cns/MSCEqF).
