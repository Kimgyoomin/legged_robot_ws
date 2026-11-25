# 운용 튜닝 — 어느 파일의 무엇을 바꾸는가

아래 경로는 workspace 루트 기준입니다. 실행 명령의 `operation_file:="$OPS"`가 선택한 프로필을 먼저 확인하세요. RUBI 프로필에서 `common_params_file`, `params_file`, `config`, `wrapper_config`는 아래 소스 YAML을 직접 가리킵니다. 파일을 수정한 뒤 해당 노드를 다시 실행합니다. 모드별 덮어쓰기도 확인합니다. 이 프로필은 `common.localization`에서 실물 CustomMsg를 선택하고 `replay.localization`에서 PointCloud2를 선택합니다.

새 현장용 사본을 만들려면 README의 [터미널 환경](../README.md#터미널-환경)을 적용한 셸에서 다음을 실행합니다. 이후 새 터미널마다 `OPS`를 같은 사본 경로로 지정합니다.

```bash
cp -i "$OPS" "$LEGGED_WS/src/navigation_dds/config/operation_site_a.yaml"
export OPS="$LEGGED_WS/src/navigation_dds/config/operation_site_a.yaml"
nano "$OPS"
```

## 자주 바꾸는 운용값

[운용 YAML](../src/navigation_dds/config/operation_rubi.yaml)에서 각 패키지의 launch 입력을 수정한 뒤 해당 패키지를 다시 실행합니다. YAML의 앵커와 별칭은 같은 토픽·frame을 여러 노드에서 공유합니다.

| 변경할 값 | 운용 YAML 위치 |
|---|---|
| Livox JSON / 센서 frame / 발행 주기 | `common.livox.config_file / lidar_frame / publish_freq` |
| ROS 속도와 LCM 송신 | `live.navigation.cmd_vel_topic`과 `live.lcm.cmd_vel_topic`은 YAML 앵커로 연결. 무선 조이스틱 입력·LCM 채널·주소는 `live.lcm.joy_topic / lcm_channel / lcm_url` |
| 장소 폴더 / CPU 목록 | 최상단 `site` / 각 패키지의 `cpu_affinity` (빈 문자열이면 부모 CPU 상속) |
| 복도 rosbag / 정지 시각 / goal | `corridor_replay.bag / pause_at_s / goal_xy_m` |
| SLAM 저장 PCD / localization 기준 PCD | `common.slam.pcd_map` / `common.localization.pcd_map` |
| SLAM LiDAR·IMU 및 점군·odom 출력 | `common.slam.lidar_topic / imu_topic / registered_topic / registered_body_topic / odom_output_topic` |
| Localization 메시지 형식·LiDAR·IMU 입력 | `common.localization.input_format / lidar_topic / imu_topic` |
| 정합 점군·body 점군·odom·전역 위치 출력 | `common.localization.registered_topic / registered_body_topic / odom_output_topic / localization_topic` |
| Height mapper scan·자세·frame | `common.height_mapping.input_scan / odom_topic / map_frame / base_frame` |
| 로컬 grid와 전역 cloud | `common.height_mapping.local_grid_topic`은 발행·구독 양쪽에 적용. 최종 cloud는 `global_cloud_topic` |
| Height 출력 폴더 / Nav2에서 읽을 파일 | `height_mapping.save_dir` / `navigation.height_map`. replay 절의 재정의도 확인 |
| Navigation cloud와 장애물 입력 | `common.navigation.cloud_topic` → `filtered_cloud_topic` → `local_cloud_topic`, `common.hybrid.global_cloud_topic` |
| Nav2 frame | `common.navigation.map_frame / local_frame / base_frame / lidar_frame`. 저장 RGHM의 frame과 TF가 일치해야 함 |
| 2D 지도 / 속도 출력 / RViz | `common.hybrid.map_yaml`, `live` 또는 `replay`의 `navigation.cmd_vel_topic`, `localization.use_rviz / navigation.rviz` |

FAST-LIO 내부의 관성·odom TF는 `body`와 `camera_init`이다. 장착 TF의 `base_frame`·`lidar_frame`을 바꾸면 실제 센서 메시지의 frame과 RViz 표시도 맞춘다. 토픽 remap은 메시지의 frame ID를 바꾸지 않는다. 원시 PointCloud2 모드에서는 `input_format: pointcloud2`와 해당 메시지 형식·필드를 가진 `lidar_topic`을 함께 지정한다.

## 센서·SLAM·localization

| 목적 | 파일 / 파라미터 |
|---|---|
| 센서 IP·host IP·180° 장착 회전 | `src/livox_ros_driver2/config/MID360s_config.json` |
| msg driver 출력·주기 | 운용 YAML `common.livox.publish_freq`, driver는 `xfer_format=1`; PC2는 `src/lddc.cpp`에서 매 2번째 스캔 |
| SLAM 포인트 수·필터 | `src/FAST_LIO_ROS2/config/mid360.yaml`: `point_filter_num`, `filter_size_surf`, `filter_size_map`, `max_iteration` |
| SLAM 저장 경로 | `mapping.launch.py pcd_map:=...`, 기존 PCD 갱신 허용은 `pcd_overwrite:=true` |
| Localization CustomMsg | `src/FAST_LIO_LOCALIZATION2/config/rubi_custommsg.yaml`: `common.*`, `preprocess.*`, `mapping.*` |
| Localization 정합 주기·voxel·threshold | `src/FAST_LIO_LOCALIZATION2/launch/localization.launch.py`: `freq_localization`, `map_voxel_size`, `scan_voxel_size`, `localization_threshold` |
| 로봇 장착 TF | 운용 프로필의 `common.localization.imu_to_base_{x,y,z}`, `base_to_lidar_{x,y,z}`(m). [장착 TF launch](../src/FAST_LIO_LOCALIZATION2/launch/localization_nav_rubi.launch.py)가 이 값을 읽습니다. |

### 다른 LiDAR 또는 로봇에 연결할 때

먼저 새 센서의 드라이버를 실행하고 LiDAR·IMU 토픽 이름, 메시지 형식, `header.frame_id`를 확인합니다. 아래는 RUBI 토픽을 검사하는 예입니다. 다른 센서에서는 토픽 이름을 실제 출력으로 바꿉니다.

```bash
ros2 topic type /livox/lidar
ros2 topic echo /livox/lidar_pointcloud --once --field fields
ros2 topic echo /livox/imu --once --field header
```

| 입력 | FAST-LIO에 연결하는 방법 |
|---|---|
| Livox CustomMsg | 운용 프로필의 `common.localization.input_format: custommsg`와 실제 `lidar_topic`, `imu_topic`을 지정합니다. SLAM의 `mid360.yaml`과 localization의 `rubi_custommsg.yaml`은 `preprocess.lidar_type: 1`입니다. |
| Livox PointCloud2 | 운용 프로필의 localization에 `input_format: pointcloud2`와 해당 점군 토픽을 함께 지정합니다. 기본 `mid360s_nav.yaml`은 타입 4입니다. 디코더는 FLOAT32 `x/y/z/intensity`, UINT8 `tag/line`, FLOAT64 `timestamp`(점별 절대 시각, ns)를 읽습니다. [입력 필드 정의](../src/FAST_LIO_LOCALIZATION2/include/livox_time.hpp) |
| Velodyne·Ouster | SLAM은 `common.slam.config_file`에서 `velodyne.yaml` 또는 `ouster64.yaml`을 선택하고 프로필의 LiDAR·IMU 토픽을 바꿉니다. RUBI localization 진입점은 타입 1·4만 허용합니다. 타입 2·3은 패키지의 `localization.launch.py`에서 `config_path`, `config_file`, `map`(PCD), `pcd_map_topic:=/cloud_pcd`, `publish_2d_map:=false`를 지정하고 장착 TF를 별도로 제공합니다. 선택한 센서 YAML의 `common.lid_topic/imu_topic`, `preprocess.scan_line`, `mapping.extrinsic_T/R`을 실측 구성에 맞춥니다. [SLAM 센서 설정](../src/FAST_LIO_ROS2/config) · [Localization 센서 설정](../src/FAST_LIO_LOCALIZATION2/config) · [Localization launch](../src/FAST_LIO_LOCALIZATION2/launch/localization.launch.py) |

로봇 URDF가 장착 TF를 이미 발행한다면 RUBI 프로필의 `common.localization.publish_mount: false`로 두고, FAST-LIO의 `body`에서 로봇 기준·LiDAR frame까지 연결되는지 확인합니다. Nav2 입력은 `common.navigation.cloud_topic`(PointCloud2)과 `lidar_frame`입니다. 새 로봇에 `BODY`가 없으면 `nav2_common.yaml`의 `costmap_sector_filter.ros__parameters.height_origin_frame`을 실제 높이 기준 frame으로 바꾸고 `min_body_height/max_body_height`(m)를 다시 정합니다. [Nav2 공통 설정](../src/legged_navigation/config/nav2_common.yaml)

### 뒤집힌 MID-360S를 정방향으로 재장착할 때

1. 현재 [MID360s_config.json](../src/livox_ros_driver2/config/MID360s_config.json)의 `lidar_configs[0].extrinsic_parameter`는 `roll: 180.0, pitch: 0.0, yaw: 0.0`입니다(도). 점군에는 장착 회전을 적용하고, 이 드라이버의 `rotate_imu`도 같은 회전으로 가속도·각속도를 변환합니다. 녹화 bag에는 발행 시점의 보정이 이미 들어 있습니다.
2. 실물 센서를 정방향으로 붙이고 센서의 +X 전방·+Y 좌측·+Z 상방이 기준축과 일치한다면, 새 장착용 JSON 사본에서는 `roll: 0.0, pitch: 0.0, yaw: 0.0`이 출발값입니다. 브래킷에 yaw나 pitch가 있으면 측정값을 사용합니다. 프로필의 `common.livox.config_file`로 그 JSON을 선택합니다. driver에서 반영한 회전을 static TF에 중복 적용하면 점군 축이 다시 뒤집힙니다.
3. [장착 TF launch](../src/FAST_LIO_LOCALIZATION2/launch/localization_nav_rubi.launch.py)는 회전을 0으로 두고, 프로필의 `common.localization.imu_to_base_{x,y,z}`와 `base_to_lidar_{x,y,z}`로 각각 `body → BODY`, `base_link → livox_frame`의 평행이동(m)을 설정합니다. 브래킷이 바뀌면 두 원점 사이 거리를 각 부모 좌표축으로 다시 측정합니다.
4. FAST-LIO의 `mapping.extrinsic_T/R`는 **LiDAR → IMU 내부 변환**입니다. 현재 세 프로필(`mid360.yaml` SLAM, `rubi_custommsg.yaml` 실물 localization, `mid360s_nav.yaml` PointCloud2 localization)은 회전 보정된 축 기준으로 `T=[-0.011, 0.02329, -0.04412]`m와 단위 회전을 씁니다. 출력 축에서 정확히 X축 180° 회전만 제거하면 같은 내부 변위의 표현은 `[-0.011, -0.02329, 0.04412]`m가 됩니다. 장비의 캘리브레이션과 좌표계가 이 가정에 맞는지 확인한 뒤 해당 프로필에 적용합니다. 과거 뒤집힌 장착의 bag을 재생할 때는 해당 bag용 보정값을 유지합니다.
5. 정지 상태에서 RViz 바닥이 수평이고 전방의 벽이 로봇 전방에 표시되는지 확인합니다. `ros2 topic echo /livox/imu --once`에서 정지 시 가속도의 주성분이 예상 +Z인지 보고, `ros2 run tf2_ros tf2_echo base_link livox_frame`으로 장착 위치를 비교합니다. 마지막으로 localization 점군과 PCD를 겹쳐 확인합니다.

드라이버의 IMU 메시지 `header.frame_id`는 현재 `livox_frame` 고정입니다. 프로필의 `lidar_frame` 기본값을 유지하면 점군·IMU·장착 TF가 일치합니다.

## 공통 RViz

`src/FAST_LIO_LOCALIZATION2/rviz/fastlio_localization.rviz`를 localization이 엽니다. 이 창에서 높이 지도·costmap·Nav2 Goal을 함께 사용합니다. `Global Options → Frame Rate`는 15이며 시각화 갱신률만 바꿉니다. Height mapper는 `rviz:=false`, navigation은 기본 false를 사용합니다. 별도 navigation 창은 명시적으로 `rviz:=true`를 지정할 때만 엽니다.

## Height Map 생성

| 목적 | 파일 / 파라미터 |
|---|---|
| 입력·거리(m) | 운용 프로필 `common.height_mapping.input_scan`은 `/cloud_registered_body`, `min_range / max_range`는 0.6 / 3.0입니다. 가까운 본체 반사점 또는 원거리 잡음을 조정할 때 씁니다. 동일 키가 local YAML에도 있으면 launch 값이 우선합니다. |
| 저장·출력 | 실물은 `common.height_mapping.save_dir`, replay는 `replay.height_mapping.save_dir` 아래 `run_<번호>/map.rghm`에 씁니다. `common.navigation.height_map`은 다음 Nav2 실행이 읽을 **파일**입니다. |
| local 격자 | [height_mapping.yaml](../src/rubi_height_mapping/config/height_mapping.yaml)의 `resolution=0.05`m, `map_length=8.0`m. 더 작은 셀은 단차 표현을 세밀하게 하고 같은 면적의 셀 수를 늘립니다. |
| 관측 융합·가시성 | 같은 파일의 `sensor_noise_factor=0.05`, `mahalanobis_thresh=2.0`는 관측 잡음·이상치 판정, `enable_visibility_cleanup=true`, `max_ray_length=3.0`m는 가시성 청소에 사용합니다. 청소용 ray 길이와 입력 점의 수용 거리를 독립적으로 확인합니다. |
| local / global 발행 | local `publish_fps=5.0`, [global_heightmap.yaml](../src/rubi_height_mapping/config/global_heightmap.yaml) `publish_fps=1.0`. 지연이 있으면 먼저 입력 scan 시각·TF를 확인합니다. |
| 전역 지도 범위 | global YAML의 `origin_x/y=-25/-25`m, `length_x/y=100/100`m, `resolution=0.05`m. 범위는 각 축에서 [−25, 75)m입니다. 범위 밖의 단차를 저장하려면 수집 전에 원점·크기를 맞춥니다. |
| 전역 셀 채택 | global `max_variance=0.04`는 local grid의 높이 분산에 대한 허용값, `invalid_cell_policy=retain`은 무효 관측 시 기존값 유지입니다. |
| TF·odom 지연 | 운용 프로필 `common.height_mapping.tf_wait_timeout` 기본 1.5s, `odom_max_gap` 기본 0.2s. 빈 `odom_topic`은 TF를 사용하고, 직접 odom 토픽을 선택하면 시간 간격 제한을 확인합니다. |

해상도를 변경하면 local/global mapper와 planner의 `map_resolution_m`을 함께 맞추고 지도를 다시 만듭니다. Navigation frozen publisher는 파일과 planner 해상도가 다르면 종료합니다. 영역 밖은 누적되지 않으므로 global 범위와 원점부터 설정합니다.

## Global planner: 별도 파일 두 개

- Hybrid: `src/legged_navigation/config/height_hybrid.yaml` → `planner_server.ros__parameters.Height`
- Height-only: `src/legged_navigation/config/height_only.yaml` → `planner_server.ros__parameters.HeightOnly`
- 실행 시 다른 YAML 사용: 각 launch의 `params_file:=/절대/경로.yaml`.

| 파라미터 | 현재 기본값 / 의미 |
|---|---|
| `grid_max_planning_time_ms` | 800 ms, 탐색 시간 예산. 시작 시 cache 준비 시간은 별도 |
| `grid_max_expanded_states` | 500000, 최대 탐색 상태 |
| `map_resolution_m` | 0.05 m, 저장 Height Map과 일치 |
| `evaluation.max_crossable_height_jump_m` | 0.11 m, 평가된 높이 변화 상한 |
| `evaluation.local_relief_threshold_m` | 0.11 m, 주변 국소 높이차 위험 판정 기준 |
| `evaluation.grid_terrain_clearance_enabled` | true, Height 기반 위험 영역 주변 여유 거리 적용 |
| `evaluation.grid_terrain_clearance_distance_m` | 0.20 m, 위험 영역으로부터 셀 중심의 최소 거리 |
| `evaluation.distance_weight` | 1.0, 평지에서도 경로 길이를 최소화하는 기준 |
| `evaluation.height_cost_weight` | 5.0, 높이 점수 전체의 배율 |
| `evaluation.grid_cot_height_cost_enabled` | true, 두 기본 planner에서 `cot_cubic.yaml`의 상승 모델 활성화 |
| `costmap_clearance_m` | **Hybrid만 0.30 m**. 2D 장애물·unknown·경계 주위 planner 추가 여유 |
| `evaluation.inflation_cost_weight` | **Hybrid만 5.0**. 2D inflation 비용 가중치 |

0.11 m는 “전역 z가 0.11 m보다 높은 셀”이라는 뜻이 아닙니다. **주변 지면과의 높이 변화/국소 relief**를 평가합니다. 위험 판정에는 유효 관측 수·quantile·Sobel 같은 설정도 관여합니다. 0.20 m clearance는 그 판정 결과 주변에 적용됩니다. Height-only에서도 이 기능은 지형으로부터 계산되므로 유지됩니다. 2D 장애물 clearance 0.30 m와 서로 다른 항목입니다.

Terrain clearance는 2D inflation 대체나 로봇 전체 footprint의 완전한 충돌 검증이 아닙니다. 끄려면 해당 파일의 `evaluation.grid_terrain_clearance_enabled: false`를 명시하고 경로가 위험 지형에 얼마나 접근하는지 확인합니다. 통로가 막히면 먼저 단차/관측 누락/2D obstacle/clearance 중 어떤 조건인지 나눠 봅니다.

## 로봇별 높이 비용

두 planner는 `src/legged_navigation/config/cot_cubic.yaml`의 보정 계수를 공유한다. `height_hybrid.yaml` 또는 `height_only.yaml`의 `evaluation.height_cost_weight`로 전체 배율을 조정한다. `evaluation.grid_cot_height_cost_enabled: false`로 바꾸면 같은 A*와 지도에서 vanilla 비용을 비교할 수 있다. 사용자 YAML은 전체 planner 파일을 복사해 `common.hybrid.params_file` 또는 `common.only.params_file`에 넣는다. 낮은 단차와 높은 단차에는 동일한 상승 함수가 적용된다. [수식, 검증 결과와 코드 위치](STEP_COST_TUNING.md)

## Local navigation: 두 구성 공통

`src/legged_navigation/config/nav2_common.yaml`을 사용합니다. `common_params_file:=...`로 교체할 수 있습니다.

| 목적 | YAML 위치 |
|---|---|
| 로봇 크기 | `local_costmap.local_costmap.ros__parameters.footprint`: ±0.275 m 정사각형, `footprint_padding`: 0.02 m |
| Local inflation | 위 `inflation_layer.inflation_radius`: 0.5 m, `cost_scaling_factor`: 3.0 |
| Hybrid global footprint/inflation | `height_hybrid.yaml`의 global_costmap: padding 0.01 m, radius 0.7 m, scaling 1.0 |
| 주행 속도·가속도 | 아래 DWB 표의 `FollowPath` 키를 수정합니다. |
| 제어 주기·궤적 평가 | `controller_frequency` 5 Hz, `FollowPath.sim_time`, `vx_samples`, `vtheta_samples` |
| 실시간 점군 높이·시야 필터 | `costmap_sector_filter.ros__parameters`: `min_body_height`, `max_body_height`, `front_half_angle_deg`, 거리 제한 |
| 속도 입력 | launch `use_fastlio_velocity:=true`: base_link 속도 변환 + LimitedAccelGenerator |
| Goal 오차·진행 감시 | `goal_checker.xy_goal_tolerance`, `yaw_goal_tolerance`, `progress_checker.*`, `failure_tolerance` |
| 재계획 | `config/navigate_hybrid.xml` / `navigate_only.xml`: `RateController hz="1.0"` |

현재 진행 감시·실패 허용 시간은 기존 운용값인 1000초로 길게 유지했습니다. 납품 장소에서 막힌 goal을 얼마 동안 재시도할지 정할 때 controller와 BT retry 값을 함께 검토합니다.

공통 local 설정은 `legged_navigation/config/nav2_common.yaml`, global 설정은 선택한 `height_hybrid.yaml` 또는 `height_only.yaml`을 수정합니다. **실행한 launch가 선택한 파일**을 확인한 뒤 재시작합니다.


### DWB 전진·회전 및 추종 튜닝

두 navigation 모두 [nav2_common.yaml](../src/legged_navigation/config/nav2_common.yaml)의 `controller_server.ros__parameters.FollowPath`를 사용합니다. 현장별 파일을 만들면 운용 프로필의 `common.navigation.common_params_file`로 전체 파일을 선택합니다. Hybrid global planner 파일은 `common.hybrid.params_file`로 별도 선택합니다.

| 바꿀 동작 | DWB 키와 RUBI 설정값 | 조정 방법 |
|---|---|---|
| 전진 속도 | `min_vel_x=0.0`, `max_vel_x=0.7` m/s | 전진 상한을 바꾸면 아래 `max_speed_xy`도 확인합니다. 현재는 전진 속도만 샘플링합니다. |
| 수평 합성 속도 | `min_speed_xy=0.0`, `max_speed_xy=0.7` m/s | X·Y 합성 속도의 상한입니다. Y=0일 때 전진 상한은 `max_vel_x`와 `max_speed_xy` 중 작은 값입니다. |
| 좌우 이동 | `min_vel_y=0.0`, `max_vel_y=0.0`, `vy_samples=0` | 현재는 횡이동을 사용하지 않습니다. 횡이동을 쓸 로봇은 Y축 범위·가감속·샘플 수와 하위 제어기 지원을 함께 봅니다. |
| 회전 속도 | `max_vel_theta=0.5` rad/s, `min_speed_theta=0.0` rad/s | yaw 회전 크기의 상한입니다. 0.5 rad/s는 약 29°/s입니다. |
| 가속·감속 | `acc_lim_x=1.0`, `decel_lim_x=-1.0` m/s², `acc_lim_theta=3.0`, `decel_lim_theta=-3.0` rad/s² | 실제 보행 가속·제동 능력에 맞춥니다. 감속 한계에는 음수를 씁니다. |
| 제어 주기·예측 | 부모의 `controller_frequency=5.0` Hz, `FollowPath.sim_time=1.5` s, `vx_samples=12`, `vtheta_samples=21` | 제어 주기는 0.2s입니다. `sim_time`은 후보 궤적을 평가하는 미래 시간입니다. 샘플 수를 늘리면 평가할 궤적 수가 증가합니다. |
| 경로 추종·장애물 | `PathAlign.scale=32.0`, `PathDist.scale=32.0`, `BaseObstacle.scale=0.02`, `ObstacleFootprint.scale=0.02` | 추종 오차와 장애물 근접에 대한 후보 궤적 점수입니다. 단차의 전역 비용은 위 Height planner 설정에서 조정합니다. |

`use_fastlio_velocity=true`이면 launch가 `LimitedAccelGenerator`를 선택하고 `sim_period=1/controller_frequency`를 적용합니다. 예를 들어 전진 속도를 0.30 m/s로 제한하는 실험이면 현장 설정의 `max_vel_x`와 `max_speed_xy`를 0.30으로 맞춘 후 navigation을 재시작합니다. 아래 명령으로 실행 중인 값을 확인합니다. LCM 브리지는 Nav2가 낸 속도를 전달하므로, DWB 속도는 위 controller 설정에서 조정합니다. [Nav2 Humble DWB 속도 파라미터](https://github.com/ros-navigation/navigation2/blob/humble/nav2_dwb_controller/dwb_plugins/src/kinematic_parameters.cpp)

```bash
ros2 param get /controller_server FollowPath.max_vel_x
ros2 param get /controller_server FollowPath.max_speed_xy
ros2 param get /controller_server FollowPath.max_vel_theta
ros2 param get /controller_server controller_frequency
```
