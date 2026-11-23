# RUBI: 실제 rosbag → FAST-LIO localization → Height Nav2

2026-11-09. `navigation_real`의 성공한 DWB/필터/속도 피드백 설정을 기준으로 구성했다. 높이 지도와 global planner는 legged 패키지의 구현을 유지한다. 원본 지도, Livox 드라이버, navigation_real 코드는 변경하지 않는다.

## 시연의 의미

`mode:=review`는 사용자가 지정한 시작점으로 경로만 계산한다. **`mode:=replay`는 녹화한 원시 LiDAR·IMU를 FAST-LIO에 넣어 현재 pose/속도를 다시 추정하고, 이 결과로 HeightPlanner와 DWB를 실행한다.**

재생 중 계산한 명령은 `/legged/replay/cmd_vel`에만 나온다. LCM은 실행하지 않는다. 기록된 로봇 궤적은 새 명령에 반응하지 않으므로 이것은 **실데이터 기반 연동 시연(open-loop)**이다. 목표 도달/장애물 회피/단차 보행의 실차 폐루프 검증으로 표시하지 않는다. 녹화 궤적과 다른 목표를 주면 controller가 그 목표를 쫓아도 실제 재생 궤적은 바뀌지 않는다.

## 공통 환경 — 새 터미널마다

```bash
cd ~/legged_robot_ws
source tools/navigation_env.sh
export ROS_DOMAIN_ID=87
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0
```

다른 workspace의 overlay를 같은 터미널에 겹쳐 source하지 않는다. 모든 재생용 터미널은 domain 87로 통일한다. 실제 로봇 domain 13과 분리한다. Frozen height/Nav2 실행에는 CuPy나 `height_env.sh`가 필요 없다.

## 터미널 1 — localization + height navigation + RViz

```bash
ros2 launch legged_bringup rubi_height_navigation.launch.py \
  mode:=replay \
  dds_profile:=navigation \
  use_fastlio_velocity:=true \
  planning_time_ms:=5000 \
  height_limit_m:=0.11 \
  height_cost:=geometry
```

현재 기본 지도는 `maps/rubi_1f_height_20261109_wall_gap_v6/site.yaml`이다. 다른 지도는 `site:=/절대경로/site.yaml`로 교체한다. replay는 **모든 구성 노드의 use_sim_time을 true로 강제**한다. localization을 별도로 또 켜지 않는다.

CPU 제한이 필요한 경우:

```bash
FASTLIO_DIAG_TIMING=1 taskset -c 2,3,4,5 \
  ros2 launch legged_bringup rubi_height_navigation.launch.py \
  mode:=replay dds_profile:=navigation navigation_prefix:='taskset -c 6,7'
```

상위 taskset은 localization/RViz에 상속되고, navigation_prefix가 Nav2·필터·frozen_map 프로세스를 6,7로 지정한다. 코어 독점/메모리 격리는 아니며, bag player를 남은 코어에 강제로 가두지 않는다. 아래 기본 검증은 CPU pinning 없이 수행한 기능 검증이다.

## 터미널 2 — 원시 데이터 3개만 재생

`[26.11.06]Height_map_data`는 1,043.75초 분량이며 CustomMsg 10,437개, IMU 208,749개, PointCloud2 5,219개가 있다. 평균 약 10/200/5Hz다. 과거 IMU가 드물게 기록된 navigation bag보다 이번 연동 확인에 적합하다.

```bash
python3 tools/replay_height_bag.py \
  "$HOME/rosbag_record/[26.11.06]Height_map_data" \
  --rate 0.5
```

명령은 metadata의 메시지 형식과 개수를 검사하고, 일시정지 상태로 시작한다. 0.5배속으로 정합과 화면을 확인한 다음, 새로 시작할 때 `--rate 1.0`으로 실제 시간에 맞춘 영상을 촬영할 수 있다. Nav2의 일부 제어 루프는 벽시계 주기로 실행되므로 0.5배속은 성능 벤치마크로 사용하지 않는다. 직접 입력하려면 동일하게:

```bash
ros2 run navigation_dds with_profile common \
  ros2 bag play "$HOME/rosbag_record/[26.11.06]Height_map_data" \
  --clock 100 \
  --rate 0.5 \
  --read-ahead-queue-size 3000 \
  --start-paused \
  --topics /livox/lidar /livox/imu /livox/lidar_pointcloud
```

- `/livox/lidar` **CustomMsg** → FAST-LIO.
- `/livox/imu` → FAST-LIO의 IMU 적분·deskew·속도 추정.
- `/livox/lidar_pointcloud` **PointCloud2** → 전방/주변 거리 및 BODY 중력 높이 필터 → 두 costmap.
- `/clock`은 player가 새로 발행한다. 녹화된 `/tf`, `/Odometry`, `/localization`, `/cmd_vel`, costmap, `/plan`은 재생하지 않는다.
- 이 녹화는 드라이버에서 장착 방향 보정이 끝난 데이터다. 뒤집는 노드를 추가하지 않는다.
- read-ahead 3000은 읽기 여유분이며 원래 녹화 누락을 복구하지 않는다. 0.5배 재생에서 벽시계 기준 5Hz라도 센서 시각 기준 10Hz이면 정상이다.

## RViz 조작 / 영상 순서

1. bag 터미널에서 Space로 재생한다. 처음부터 재생해 IMU 초기화를 거친다.
2. 스캔·Odometry가 들어온 뒤(처음 약 5~10초의 bag 데이터) Space로 잠시 멈춘다.
3. RViz에서 **2D Pose Estimate**로 실제 녹화 시작 위치·방향을 PCD 지도에 지정한다. 이번에는 가상 시작점 선택이 아니라 global localization 초기화다.
4. Space로 재개하고 지도와 스캔이 정합되는지 본다. `/map_to_odom`이 갱신되고 Nav2가 active인지 확인한다.
5. **Navigation2 Goal** 도구로 녹화 이동 방향의 도달 가능한 목표를 선택한다. `/plan`, `/local_plan`, local costmap과 localization 화살표를 함께 촬영한다.
6. 녹화가 끝나거나 처음으로 되감으려면 이 launch와 player를 종료한 뒤 다시 시작한다. 기존 FAST-LIO 상태에 과거 시간을 주입하지 않는다.

Frozen height가 2D 지도에 가려지면 RViz `Occupancy` 표시를 끄거나 Alpha를 낮춘다. `Move Camera`/Orbit으로 3D 단차를 볼 수 있다. Nav2 route는 `/plan`, DWB local plan은 `/local_plan`이다.

확인용 명령(같은 환경/domain):

```bash
ros2 topic hz /localization --window 30
ros2 topic echo /fastlio/odometry_base --field twist.twist --once
ros2 topic hz /livox/lidar_costmap --window 30
ros2 topic hz /local_costmap/costmap --window 30
ros2 topic echo /legged/replay/cmd_vel --once
ros2 param get /controller_server odom_topic
ros2 param get /controller_server FollowPath.trajectory_generator_name
```

`ros2 topic delay /localization --window 30 --use-sim-time`처럼 측정 노드도 시뮬레이션 시각을 사용해야 한다. 현재 PC 시각과 녹화 센서 시각의 차이를 파이프라인 지연으로 해석하지 않는다.

## 성공했던 navigation_real과 맞춘 값

| 항목 | RUBI 전용 설정 |
|---|---|
| Local controller | DWB, Rotation Shim 없음, 5Hz |
| 실제 속도 입력 | `/fastlio/odometry_velocity` → 장착 위치 보정 → `/fastlio/odometry_base` |
| 가속 제한 | LimitedAccelGenerator, sim_period 0.2초 |
| 선속도/가감속 | 최대 0.8m/s, +1.0/−1.0m/s² |
| yaw 속도/가감속 | 최대 1.0rad/s, +3.0/−3.0rad/s² |
| DWB 예측 | 1.5초, vx 12개, vtheta 21개 |
| Footprint | 0.55×0.55m, local/global padding 0.02/0.01m |
| Local costmap | camera_init, 6×6m, 0.05m, update/publish 5/5Hz |
| Global costmap | map, 0.05m, update/publish 1/1Hz, static→obstacle→inflation |
| 거리 필터 | 센서 +X 기준 ±60°는 최소 0.3m, 나머지는 최소 1.0m |
| 높이 필터 | BODY 원점, `/fastlio/up_body` 중력 위쪽, **−0.20~+1.0m** |
| 시각 | 원래 관측 stamp 유지; 해당 시각 TF/중력 사용 |
| TF 장착 | body→BODY=(-0.1105,0.02329,0.09338), BODY→base_link=identity, base_link→livox=(0.0995,0,-0.1375) |
| DDS | 공통32MiB, 큰 PCD 발행128MiB, UDP 송수신4MiB 요청 |
| 바뀐 global planner | 기존 2D A* 대신 `legged_navigation/HeightPlanner` |
| Height 제한/비용 | jump/relief 0.11m, geometry; terrain clearance 0.20m |
| Occupancy clearance | 중심 기준 0.30m; 0.45m는 시작 방 출입구 연결을 끊음 |
| Planning 예산 | 5,000ms, BT replan 목표 1Hz. 계산이 1초를 넘으면 실제 1Hz 불가 |

BODY 하한은 현재 navigation_real YAML의 **−0.20m**를 복사했다. 과거 논의한 −0.05m로 임의 변경하지 않았다. 실제 배치에 따라 낮은 단차가 실시간 obstacle layer에도 표시되면 HeightPlanner가 통과 가능하다고 해도 DWB는 피할 수 있다.

0.30m는 전역 중심점 clearance이며 회전하는 사각 몸체의 전체 충돌 여부를 보장하는 값은 아니다. DWB는 별도로 footprint를 검사한다. unknown/높이 증거 부족/29cm 단차 등은 계속 막는다. cost를 낮췄다고 통과 불가 조건이 풀리는 것은 아니다.

## 패키지와 프로필 구분

- `navigation_dds/`: DDS XML과 wrapper. 시스템 설정은 바꾸지 않는다.
- `FAST_LIO_LOCALIZATION2/`: 성공본 센서 처리 실행 구조, C++ transform fusion, velocity/up 출력. legged의 MID360S PointCloud2 timestamp decoder는 보존.
- `legged_navigation/`: HeightPlanner + 기존 추종 도구 + 이식한 `costmap_sector_filter`, `fastlio_velocity_feedback`. planner 복제품을 추가하지 않았다.
- `legged_bringup/config/rubi/{nav2.yaml,planner.yaml,navigate.xml}`: RUBI 실물 기준 전용 파라미터/BT. 일반 데모 설정과 구분.
- `legged_bringup/legged_bringup/rubi_navigation.py`: live/replay가 같은 설정을 사용하는 launch 구성.
- `tools/replay_height_bag.py`: raw 3-topic 재생 도구.
- `maps/.../site.yaml`: 지도 세트 선택. 지도 후처리와 이 연동 변경은 독립적.

기존 generic `navigate.launch.py`의 RPP·collision_monitor·watchdog 경로도 그대로 남아 있다. 새 RUBI profile은 성공한 pongbot DWB launch와 맞추어 그 generic 경로를 사용하지 않는다. RUBI replay 명령 출력을 실제 로봇으로 연결하지 않는다.

## localization / Nav2를 따로 실행하려면

동일 replay domain과 use_sim_time으로:

```bash
# localization 터미널: driver 대신 bag의 CustomMsg 사용
FASTLIO_DIAG_TIMING=1 taskset -c 2,3,4,5 \
  ros2 launch fast_lio_localization localization_nav_rubi.launch.py \
  input_format:=custommsg dds_profile:=navigation \
  use_sim_time:=true use_rviz:=false
```

별도 localization + Nav2 분리는 실제 운용용 `mode:=nav2`가 제공하지만 `/cmd_vel`을 발행하므로 영상용으로는 위 `mode:=replay` 통합 명령을 권장한다. 통합 replay와 standalone localization을 동시에 실행하면 중복 노드/TF가 된다.

실제 legged MID360S 드라이버의 기본 출력이 `/livox/lidar` PointCloud2이면 standalone localization의 기본 `input_format:=pointcloud2`를 쓰고, `mode:=nav2 cloud_topic:=/livox/lidar`로 필터 입력도 맞춘다. navigation_real처럼 CustomMsg/PointCloud2 이중 발행이면 `input_format:=custommsg`, `cloud_topic:=/livox/lidar_pointcloud` 조합이다. **토픽 이름이 아니라 메시지 타입을 확인한다.**

## 선택 빌드

```bash
cd ~/legged_robot_ws
source tools/navigation_env.sh
CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2 \
  colcon build --base-paths src --symlink-install --executor sequential \
  --packages-select navigation_dds fast_lio_localization legged_navigation legged_bringup \
  --packages-ignore fast_lio genz_icp \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source install/local_setup.bash
```

이번 변경 전 소스는 `backups/rubi_integration_20261109_155401/`에 있다. `backups/COLCON_IGNORE`로 중복 패키지 검색을 막았다. Git commit/push는 실행하지 않았다.

## 이번 실제 데이터 검증

빌드와 단위/launch 검증을 통과했다. `[26.11.06]Height_map_data`를 1배속으로 약108초 재생하여 실제 이동 구간까지 확인했다. Localization·속도 출력 약10Hz, 필터 점군 약5Hz였으며 HeightPlanner와 DWB의 replay 명령까지 연결됐다. 이전에 실패한 전체 정적 경로도1,652 poses/약2.85초로 성공했다.

다만 전체 bag·RViz·taskset 조건의 검증은 아직 아니다. 5Hz로 설정한 local costmap 수신 평균은3.64Hz였고, 기록된 이동과 고정 목표의 관계가 바뀐 구간에서 경로 실패·복구·명령 공백이 발생했다. 정적 review 성공을 실제 동적 주행 성공으로 해석하지 않는다. [측정값·로그·그림과 남은 확인 사항](../archive/20261109/validation/rubi_replay_20261109/README.md)을 참고한다.
