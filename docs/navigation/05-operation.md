# 05. 실행·점검·재현

[문서 처음으로](README.md)

## 빌드와 환경

실행 기준은 Ubuntu 22.04 / ROS 2 Humble이다. 최초 SDK·ROS 의존성 설치는 [기존 설치 안내](../INSTALL.md)를 따른다. 아래는 기존 workspace에 변경을 가져온 뒤의 증분 빌드다.

```bash
cd ~/legged_robot_ws
source /opt/ros/humble/setup.bash
source install/local_setup.bash
colcon build --symlink-install --executor sequential \
  --packages-select fast_lio_localization legged_navigation legged_bringup legged_range_trial \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/local_setup.bash
```

깨끗한 checkout은 기존 install이 없으므로 의존 패키지까지 먼저 빌드해야 한다. PC build/install 바이너리를 Jetson에 복사하지 말고 대상 장치에서 빌드한다. 저장된 height만 읽어 주행하는 경로는 GPU height 생성 노드를 시작하지 않는다.

navigation/localization 터미널은 이 workspace가 마지막 overlay가 되게 source한다. 모든 노드가 같은 ROS_DOMAIN_ID를 사용해야 한다. 현재 실험은 45다.

```bash
source /opt/ros/humble/setup.bash
source ~/legged_robot_ws/install/local_setup.bash
export ROS_DOMAIN_ID=45
```

## 터미널 1: Gazebo와 보행 정책

시뮬레이터는 아직 별도 `/home/kim/rubi_ws`의 `rubi_gazebo_sim`과 외부 world를 사용한다. 새 PC에는 별도 설치가 필요하다.

```bash
source /opt/ros/humble/setup.bash
source /home/kim/rubi_ws/install/local_setup.bash
export ROS_DOMAIN_ID=45

ros2 launch rubi_gazebo_sim rubi_gazebo_terrain_lidar.launch.py \
  world:=/home/kim/cot_experiment/navigation_cost_comparison/worlds_v3_lower10/rubi_cost_comparison_lower10.world \
  contact_logging:=true gui:=true paused:=false \
  auto_demo:=false launch_joy_node:=false
```

보행 정책의 활성 상태와 입력 선택이 NAV2인지 확인한다. 이 navigation launch가 별도 로봇 policy/LCM bridge를 자동 실행하는 것은 아니다. Gazebo의 `/clock`·센서가 들어오는지 먼저 확인한다.

## 터미널 2: FAST-LIO localization 직접 실행

위 공통 navigation 환경을 source한 뒤:

```bash
ros2 launch fast_lio_localization localization_nav_rubi.launch.py \
  use_sim_time:=true publish_2d_map:=false use_rviz:=true \
  pcd_map:=/home/kim/legged_robot_ws/maps/cost_wall_lower10_v3/localization.pcd
```

기존 `maps/cost_wall_main_v2/localization.pcd`도 동일한 파일이라 사용 가능하다. `legged_bringup localize`를 함께 실행하지 않는다. RViz에서 초기 위치·방향을 맞추고 map/센서/몸체가 정합되었는지 확인한다.

이 launch의 `mid360.yaml`은 simulation의 `/livox/lidar` CustomMsg, `/livox/imu`를 읽는다. 실제 센서용 PointCloud2 구성과 다르다. `use_sim_time=true`만으로 잘못된 입력 타입·stamp·clock reset 문제가 해결되지는 않는다.

## 터미널 3: RPP 비교

```bash
ros2 launch legged_range_trial navigate.launch.py \
  site:=/home/kim/legged_robot_ws/maps/cost_wall_lower10_v3/site.yaml \
  use_sim_time:=true \
  cloud_topic:=/livox/lidar_PointCloud2 \
  controller:=rpp \
  fastlio_velocity:=true \
  estimate_velocity:=false
```

DWB 비교는 `controller:=dwb`로만 바꾼다. Gazebo·localization은 유지하고 기존 navigation launch를 Ctrl+C로 종료한 다음 새 controller를 띄운다. 같은 domain에서 두 navigation을 중복 실행하지 않도록 실행 잠금이 있다.

Nav2에서는 RViz **Nav2 Goal** 도구를 사용한다. 기존 `mode:=pursuit`의 `/goal_pose` 방식과 구분한다. navigation이 map_server와 frozen_map을 시작하므로 별도 mapper/frozen publisher를 같이 띄우지 않는다.

추가 선택값:

```text
final_guard:=true       # 추가 costmap swept guard 복원; 기본 false
rviz:=false            # 이미 필요한 RViz가 있을 때
filter:=/absolute/path/range.yaml
rpp:=/absolute/path/rpp.yaml
nav2:=/absolute/path/nav2.yaml
planner:=/absolute/path/planner.yaml
costmap:=/absolute/path/costmap.yaml
collision:=/absolute/path/collision.yaml
```

`mode:=pursuit`는 Nav2 RPP가 아닌 기존 custom tracker다. 그 모드에서는 final guard가 항상 켜진다. trial 기본값만으로 실행하면 `controller=dwb`, `fastlio_velocity=false`이므로 위 비교 명령을 그대로 기록해 사용한다.

## 실행 후 실제 적용값 확인

```bash
ros2 param get /controller_server FollowPath.plugin
ros2 param get /controller_server odom_topic
ros2 param get /controller_server failure_tolerance
ros2 param get /frozen_map file
ros2 param get /map_server yaml_filename
ros2 param get /obstacle_cloud front_half_angle_deg
ros2 param get /obstacle_cloud front_min_range
ros2 param get /obstacle_cloud min_range
ros2 param get /obstacle_cloud min_body_height
ros2 param get /global_costmap/global_costmap inflation_layer.inflation_radius
ros2 param get /global_costmap/global_costmap inflation_layer.cost_scaling_factor
ros2 param get /local_costmap/local_costmap footprint
ros2 topic info /cmd_vel --verbose
```

RPP라면 plugin은 `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController`, odom은 `/legged/controller_odom`이어야 한다. footprint는 ±.275m 사각형이다. `/cmd_vel`에 테스트 publisher가 동시에 있으면 추종 평가가 성립하지 않는다.

필터만 확인할 때는 다음 preview를 사용할 수 있다. preview는 속도 명령을 만들지 않고, 현재 navigation의 입력에도 자동 연결되지 않는다.

```bash
ros2 launch legged_range_trial preview.launch.py use_sim_time:=true
```

RViz에서 `/range_trial/obstacles`와 원시 `/livox/lidar_PointCloud2`를 비교한다.

## 멈춤을 진단하는 순서

1. `/clock`이 증가하는지, 센서·odom이 새 stamp로 오는지 확인한다. Pause 후 wall-time watchdog 정지를 controller 실패로 해석하지 않는다.
2. goal이 active인지 확인한다. 이미 aborted된 목표는 센서가 복구돼도 자동으로 새 goal이 되지 않는다.
3. global path 생성 성공 여부와 robot/start/goal cost를 확인한다. planner가 실패해 FollowPath가 취소됐으면 DWB/RPP가 회피 명령을 내지 않는 것이 정상이다.
4. `/legged/cmd_raw` → `/legged/cmd_safe` → `/cmd_vel`을 비교해 어느 단계에서 0이 되는지 찾는다.
5. `/legged/motion_status`와 `/legged/collision_input_state`, TF age, filtered cloud를 함께 확인한다.
6. map에서 footprint가 장애물과 겹치지만 Gazebo에는 여유가 있다면 map/world/localization 정합을 먼저 검토한다.

```bash
ros2 topic echo /legged/collision_input_state
ros2 topic echo /legged/motion_status
ros2 topic hz /fastlio/odometry_velocity
ros2 topic hz /legged/controller_odom
```

RViz costmap 발행은 2Hz지만 내부 갱신은 5Hz, controller는 10Hz다. 화면 프레임이 적다고 그만큼만 제어하는 것은 아니다. 반대로 controller가 10Hz라고 매 tick에 새 cloud/costmap이 있다는 뜻도 아니다. 센서 latency·TF age·update 주기를 같이 기록한다.

## 비교 실험에 남길 것

각 run마다 controller/config/site hash, world, 시작/목표, 큐브 위치, 보행 입력 source, sim time 범위와 wall time을 저장한다. 다음을 같은 조건에서 최소 여러 번 비교한다.

- 직선 속도·완주시간, 경로 오차 p50/p95/max.
- 코너 진입 전 cmd vx/wz와 측정 vx/wz, 회전 시작 시점·경로 접선 변화.
- 최소 obstacle/footprint 여유, `No valid trajectories`, planner failure, CM 정지 횟수·지속시간.
- 사람이 아닌 큐브로 통로를 잠깐 막았다 치운 뒤 goal 유지·재계획·재개 여부.
- callback CPU·메모리·message age, Jetson에서는 전력 모드·온도·throttling도 함께.

권장 bag 주제 예시(실제 topic 이름은 `ros2 topic list`로 확인):

```bash
ros2 bag record -o /tmp/rubi_nav_comparison \
  /clock /tf /tf_static /Odometry /fastlio/odometry_velocity \
  /legged/controller_odom /legged/cmd_raw /legged/cmd_safe /cmd_vel \
  /legged/motion_status /legged/collision_input_state \
  /plan /local_plan /global_costmap/costmap /local_costmap/costmap \
  /legged/obstacles /legged/collision_points
```

raw cloud·GT·policy source는 사용하는 simulator의 실제 토픽을 확인해 추가한다. 대형 bag은 Git 소스 commit에 넣지 말고 별도 보관하며, run 요약·해시·그림을 이 문서 형식으로 남긴다.

## 종료·복귀

navigation 터미널에서 Ctrl+C로 종료한다. controller를 비교할 때 simulator/world를 매번 재시작하면 초기화 조건이 바뀌므로 구분해서 기록한다. simulator clock을 0으로 재시작했다면 이전 시간대의 state/TF/queue가 남은 localization/navigation도 새 세션으로 시작한다.

native velocity 연결만 되돌릴 때는 navigation을 재실행하면서 `fastlio_velocity:=false estimate_velocity:=false`를 사용한다. 이때 기존 `/Odometry` twist가 0인 구성으로 돌아간다는 점을 알고 비교한다. RPP↔DWB 비교에서는 native velocity를 계속 켜서 한 변수만 바꾼다.
