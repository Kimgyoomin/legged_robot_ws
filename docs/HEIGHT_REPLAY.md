# 실제 bag → FAST-LIO Localization → Height Nav2

**Localization과 Navigation을 각각 실행**합니다. 전체 설치 및 실물 실행은 [README](../README.md)를 참고하세요. `rubi_localization.launch.py`는 `fast_lio_localization/localization_nav_rubi.launch.py`를 포함하고, 선택한 site의 PCD를 넘기는 얇은 wrapper입니다. `fast_lio mapping.launch.py`를 실행하지 않습니다.

## 공통 — 새 터미널마다

```bash
cd ~/legged_robot_ws
source tools/navigation_env.sh
export ROS_DOMAIN_ID=13
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0
```

모든 터미널을 domain 13으로 통일합니다. 같은 domain의 기존 navigation_real·실물 Livox·다른 bag은 함께 실행하지 않습니다. 실물 노드를 유지해야 하면 재생용 모든 터미널을 domain 87 등으로 분리하세요. 이 실행에는 `height_env.sh`가 필요하지 않습니다. DDS는 공통 32MiB, 큰 PCD publisher 128MiB SHM과 UDP를 사용합니다. 소켓 TX/RX 요청 4MiB는 커널 상한의 영향을 받습니다. `taskset`은 코어 독점이나 메모리 격리가 아닙니다.

## 터미널 1 — FAST-LIO Localization + 기존 RViz

```bash
FASTLIO_DIAG_TIMING=1 taskset -c 2,3,4,5 \
  ros2 launch legged_bringup rubi_localization.launch.py \
  input_format:=custommsg \
  dds_profile:=navigation \
  use_sim_time:=true \
  use_rviz:=true
```

직접 기존 package를 실행해도 됩니다. **위와 아래 중 하나만** 실행합니다.

```bash
FASTLIO_DIAG_TIMING=1 taskset -c 2,3,4,5 \
  ros2 launch fast_lio_localization localization_nav_rubi.launch.py \
  pcd_map:="$PWD/maps/rubi_1f_legacy/data/1f_fast_lio_260731_segmented.pcd" \
  input_format:=custommsg dds_profile:=navigation \
  use_sim_time:=true use_rviz:=true \
  rviz_config:="$PWD/src/legged_bringup/rviz/rubi_height_navigation.rviz"
```

이제 Localization이 RViz를 소유합니다. navigation_real의 PCD·점군·Odometry·localization·costmap·경로 화면과 Orbit 카메라를 가져오고 `Frozen global height`를 추가했습니다. 지도만 뜨고 로봇 위치가 아직 안 보이면 다음 센서 재생을 시작하세요.

## 터미널 2 — 원시 센서 재생 + 초기 위치 지정

아래 명령은 domain 13으로 통일한 환경에서 센서 토픽만 재생합니다. bag 경로는 사용할 데이터로 바꿉니다.

```bash
ros2 run navigation_dds with_profile common \
  ros2 bag play "$HOME/rosbag_record/[26.11.06]Height_map_data" \
  --clock 100 \
  --rate 1.0 \
  --read-ahead-queue-size 3000 \
  --start-paused \
  --topics /livox/lidar /livox/imu /livox/lidar_pointcloud
```

| 입력 | 형식 | 소비자 |
|---|---|---|
| `/livox/lidar` | CustomMsg, 약10Hz | FAST-LIO Localization |
| `/livox/imu` | Imu, 약200Hz | FAST-LIO 적분·deskew·속도 추정 |
| `/livox/lidar_pointcloud` | PointCloud2, 약5Hz | BODY 높이/방향별 거리 필터 → local/global costmap |

이 bag은 1,043.75초, 각 토픽 10,437 / 208,749 / 5,219개입니다. `/clock`은 player가 생성합니다. 기록된 `/tf`, `/Odometry`, `/localization`, `/cmd_vel`, costmap, `/plan`은 재생하지 않습니다. 장착 회전 보정은 이미 기록에 포함되어 있어 뒤집기 노드를 추가하지 않습니다.

1. player에서 Space로 재생을 시작하고 처음 5~10초 데이터로 IMU를 초기화합니다.
2. 잠시 일시정지한 뒤 **Localization RViz의 2D Pose Estimate**로 녹화 시작 위치·방향을 지정합니다.
3. 재생을 재개해 `/map_to_odom`, `/localization` 및 지도·점군 정합이 안정적인지 확인합니다. 이 단계에는 Navigation이 필요하지 않습니다.

`use_sim_time:=true`에서는 player의 `--clock`이 필요합니다. bag 재생 전 ROS Time=0이거나 센서 입력이 없으면 위치 추정은 진행되지 않습니다. PCD 지도 자체는 먼저 볼 수 있습니다.

## 터미널 3 — 정합 안정화 후 Height Navigation

```bash
taskset -c 6,7 \
  ros2 launch legged_bringup rubi_height_navigation.launch.py \
  mode:=replay \
  dds_profile:=navigation \
  use_sim_time:=true \
  use_fastlio_velocity:=true \
  cloud_topic:=/livox/lidar_pointcloud \
  rviz:=false
```

기본 site는 `maps/rubi_1f_height_20261109_wall_gap_v6/site.yaml`, planning budget은 **800ms**, 높이 한도는 **0.11m**, 비용은 **geometry**입니다. `mode:=replay`는 sim time을 강제하고 명령을 `/legged/replay/cmd_vel`로 보냅니다. LCM은 실행하지 않습니다. Localization은 기본적으로 포함하지 않습니다(`launch_localization:=false`).

시작 후 `Frozen terrain READY`를 기다립니다. 지도·고정 파라미터의 최초 캐시 계산에 측정상 5~11초가 들었고, 그 후 경로 요청에 재사용됩니다. 준비 중 goal은 `cache is preparing`으로 거절될 수 있습니다. 캐시를 준비해도 움직이는 장애물 costmap은 매 요청 새로 반영합니다.

Navigation이 시작되면 이미 열린 Localization RViz에 `/legged/height`, `/map`, costmap과 경로가 들어옵니다. 추가 RViz는 기본적으로 열지 않습니다. 별도 창이 필요하면 `rviz:=true`를 명시하세요. `mode:=review`와 `launch_localization:=true`를 쓰는 통합 재생은 기본적으로 자체 RViz를 엽니다.

## 영상 촬영 / 표시

Localization 정합 → Navigation 시작 → Nav2 active 및 `Frozen terrain READY` 확인 → **Nav2 Goal** 도구로 목표 지정 순서입니다. 같은 창에서 localization, `/plan`, `/local_plan`, local/global costmap, 고정 height map을 함께 봅니다. 기존 `2D Goal Pose` 도구도 유지했으며 Nav2 action 목표에는 추가된 `Nav2 Goal`을 권장합니다.

- `Frozen global height`: Navigation이 로드한 고정 높이 지도. Z 색상, 2.5cm 사각 점, alpha 0.6.
- `Localisation/map`: 기존 노란 PCD 지도. 초기 위치 정합에 사용합니다.
- `Local costmap` / `Global costmap`: 센서 장애물에 따라 갱신됩니다. height와 겹쳐 보기 어려우면 `Frozen global height` 체크만 잠시 끕니다. 표시는 계산에 영향을 주지 않습니다.
- `/path`는 현재 FAST-LIO 설정에서 꺼져 있으므로 `FAST-LIO path (optional)` 표시도 기본 해제했습니다. 계획 경로는 `Global plan`과 `DWB local plan`입니다.
- Navigation 실행 전 `/map`, height, costmap의 No messages 경고와 Navigation 2의 inactive/unknown은 해당 publisher가 아직 없어 발생할 수 있습니다. PCD 정합과 `/localization`을 먼저 확인하세요.

다시 처음으로 되감으려면 player·localization·navigation을 종료하고 재시작합니다. FAST-LIO 누적 상태에 과거 시각을 다시 주입하지 않습니다.

관찰을 느리게 하려면 `--rate 0.5`를 쓸 수 있지만, 제어 루프 벽시계와 센서 시각의 비율이 달라져 실시간 성능 측정값으로 쓰지 않습니다. `Message queue starved`는 player 읽기 지연 경고이며 원본 녹화 누락을 곧바로 뜻하지 않습니다.

확인 명령:

```bash
ros2 topic hz /localization --window 30
ros2 topic echo /fastlio/odometry_base --field twist.twist --once
ros2 topic hz /livox/lidar_costmap --window 30
ros2 topic hz /local_plan --window 30
ros2 topic hz /legged/replay/cmd_vel --window 30
ros2 param get /controller_server odom_topic
ros2 param get /controller_server FollowPath.trajectory_generator_name
```

관측 지연은 `/clock`과 header stamp를 비교해야 합니다. 현재 PC 벽시계와 녹화 시각 차이를 지연으로 해석하지 않습니다.

## 적용된 기존 실물 기준

| 항목 | 값 |
|---|---|
| Local controller | DWB 5Hz, Rotation Shim 없음 |
| Feedback | FAST-LIO velocity → `/fastlio/odometry_base` |
| Trajectory generator | LimitedAccelGenerator, sim_period 0.2초, sim_time 1.5초 |
| 속도/가감속 | x 최대0.8m/s, ±1.0m/s²; yaw 최대1.0rad/s, ±3.0rad/s² |
| Footprint | 0.55×0.55m |
| Costmap 설정 | local camera_init 6×6m 5/5Hz, global map 1/1Hz, 해상도0.05m |
| 거리 필터 | 전방±60° 최소0.3m, 나머지 최소1.0m |
| 높이 필터 | BODY 원점·중력 방향, **−0.20~+1.0m**; 원래 stamp 유지 |
| Global planner | HeightPlanner, 11cm jump/relief, geometry cost |
| Clearance | terrain0.20m, occupancy 중심0.30m |

하한 −0.20m는 이전 성공본에서 이식한 현재값입니다. 과거 논의한 −0.05m로 새로 변경한 것이 아닙니다. 낮은 단차가 동적 obstacle layer에 표시되면 terrain 통과 한도와 별개로 DWB가 피할 수 있습니다.

**재생 궤적은 새 명령에 반응하지 않습니다.** 이는 실제 센서 기반 연동 시연이며 목표 도달·단차 보행의 실차 폐루프 검증은 아닙니다. 이번 1배속 짧은 시험에서 localization10Hz, local plan/명령5Hz였으나 local costmap 관측 수신률은 약3.49Hz였습니다. 전체 bag·RViz 포함 성능은 별도 확인이 필요합니다. [실측과 한계](FROZEN_PLANNER.md)를 참고하세요.

통합 실행이 필요한 경우에만 navigation에 `launch_localization:=true`를 추가합니다. 이때 터미널1은 실행하지 않습니다. 기본은 모듈별 실행입니다.

## PointCloud2 bag

위 명령은 `/livox/lidar`가 CustomMsg이며 별도 `/livox/lidar_pointcloud`가 기록된 bag용입니다. 현재 `rviz_MID360s_launch.py`로 녹화한 PointCloud2 bag은 다음 세 항목을 함께 바꿉니다.

- Localization: `input_format:=pointcloud2`.
- Player: `--topics /livox/lidar /livox/imu`.
- Navigation: `cloud_topic:=/livox/lidar`.

토픽 이름이 같아도 타입이 다를 수 있으므로 `ros2 bag info`로 먼저 확인합니다. `msg_MID360s_launch.py`는 별도 CustomMsg 구성입니다.

## Costmap이 갱신되지 않을 때

먼저 player가 일시정지 중인지 확인하고, 같은 환경을 불러온 진단 터미널에서 실행합니다.

```bash
ros2 topic info /clock
ros2 topic echo /clock --once
ros2 param get /laser_mapping use_sim_time
ros2 param get /controller_server use_sim_time
ros2 param get /local_costmap/local_costmap use_sim_time
ros2 param get /global_costmap/global_costmap use_sim_time
ros2 topic hz /livox/lidar_pointcloud --window 30
ros2 topic hz /livox/lidar_costmap --window 30
ros2 topic hz /local_costmap/costmap --window 30
ros2 topic hz /local_costmap/costmap_updates --window 30
```

`/clock` publisher가 없으면 player를 `--clock 100`과 함께 다시 시작합니다. 재생이 진행 중인데 `/localization`이나 `/livox/lidar_costmap`이 멈추면 입력 형식, 초기 정합, TF/중력 입력과 필터 로그를 차례로 확인합니다. costmap의 내부 갱신, 전체 지도 발행, 부분 갱신 토픽 수신률, RViz 표시율은 각각 다른 지표입니다.

`Message queue starved`가 계속되면 재생률을 낮추고 입력 토픽 수·디스크 읽기·CPU 부하를 확인합니다. SHM 크기만으로 이 경고의 원인을 확정하지 않습니다.

## 선택: 재생 wrapper

`tools/replay_height_bag.py`는 위 CustomMsg/IMU/PointCloud2 세 토픽의 존재와 타입을 검사합니다. 이 도구는 domain 0과 13을 거부하므로, 사용할 때는 **Localization·Navigation·RViz·player 모두** `ROS_DOMAIN_ID=87` 등 별도 domain으로 실행합니다. domain 13을 사용할 때는 앞의 직접 `ros2 bag play` 명령을 사용합니다.

```bash
export ROS_DOMAIN_ID=87
python3 tools/replay_height_bag.py \
  "$HOME/rosbag_record/[26.11.06]Height_map_data" \
  --rate 1.0
```
