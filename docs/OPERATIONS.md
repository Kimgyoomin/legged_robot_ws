# 지도와 주행 운영

## A. 사이트 준비

기준은 기존에 정합한 **localization PCD + occupancy YAML/PGM의 map frame**입니다. 바닥의 높이 기준(z datum), gravity 방향, 로봇 장착 translation을 먼저 확인합니다. 새 height map은 이 PCD에 localization한 pose로 취득하면 됩니다.

PCD/occupancy만 있는 첫 mapping 세션에서는 `site.yaml`의 height 항목은 아직 없어도 됩니다. 아래 형태로 두 지도와 이미지의 SHA-256을 기록합니다. `alignment.verified`는 실제 확인 결과를 기입하며, false 상태로 실제 navigation을 시작하지 않습니다.

```yaml
format: 1
id: site-a-v1
frame: map
height_resolution: 0.05
alignment:
  verified: true
  method: "기존 PCD와 occupancy의 정합 결과 및 검사 기록"
assets:
  pcd: {path: localization.pcd, sha256: "실제 SHA-256"}
  occupancy: {path: occupancy.yaml, sha256: "실제 SHA-256"}
  image: {path: floor.pgm, sha256: "실제 SHA-256"}
```

해시 값은 `sha256sum 파일명`으로 얻습니다. `maps/demo/site.yaml`은 구조를 보여주는 합성 예시이며, 실물 localization에 사용하지 않습니다.

## B. 센서와 localization

```bash
source ~/legged_robot_ws/install/setup.bash
ros2 launch legged_bringup sensor.launch.py
```

별도 터미널:

```bash
source ~/legged_robot_ws/install/setup.bash
ros2 launch legged_bringup localize.launch.py \
  site:=/absolute/path/site.yaml method:=fastlio
```

RViz로 초기 pose를 지정합니다. `/localization`은 전역 정합이 준비된 뒤 발행됩니다. 이 단계는 2D map server를 시작하지 않으며, navigation launch가 `/map`을 소유합니다. 처음 mapping하는 동안 occupancy를 RViz에 표시할 필요가 있으면 map server를 별도로 띄울 수 있지만, navigation을 시작할 때 중복 map server를 제거합니다.

GenZ 대안은 `method:=genz`입니다. 두 localizer를 동시에 띄워 같은 TF child의 소유자가 둘이 되지 않게 합니다. `frames.yaml`은 실제 기구 값으로 수정합니다. 기존 robot_state_publisher가 동일한 장착 TF를 이미 발행한다면 `publish_mount:=false`를 사용하고 필요한 body/base 연결이 있는지 확인합니다.

## C. Height 취득

**맞습니다. Height mapper는 LiDAR 점군과 localization에서 얻은 odometry를 함께 받아야 합니다.** B 단계에서 센서와 선택한 localizer를 먼저 실행하고, 초기 위치 정합이 된 뒤 이 단계를 별도 터미널에서 실행합니다. `mapping.launch.py`는 height mapper와 전역 누적 wrapper를 실행하며, localizer를 자동으로 시작하지 않습니다.

| 선택한 localization | `input_scan` — 관측한 점 | `odom_topic` — map 기준 위치·자세 |
|---|---|---|
| FAST-LIO | `/cloud_registered_body` — 움직임 보정 점군 | `/localization` — `map → body` |
| GenZ-ICP | `/livox/lidar` — 원시 점군 | `/genz/localization_odom` — 현재 설정의 `map → base_link` |

두 odometry 토픽의 ROS 메시지 타입은 `nav_msgs/msg/Odometry`입니다. 토픽 이름이 `/localization`이어도 odometry 메시지입니다. Mapper는 점군의 timestamp에 해당하는 pose와 고정 장착 변환을 사용하여 점들을 map 좌표로 옮깁니다.

FAST-LIO 기반 시뮬레이션은 이제 다음 명령 하나로 실행합니다.

```bash
source ~/legged_robot_ws/install/setup.bash
ros2 launch legged_bringup mapping.launch.py
```

운영 설정은 [mapping.yaml](../src/legged_bringup/config/mapping.yaml)에 모았습니다.
기본은 `use_sim_time: true`, `/cloud_registered_body`, `/localization`, `map`,
`base_link`, 0.6–3.0 m, `ray_mode: all`, RViz 켜짐입니다. 저장 위치는
`~/legged_robot_ws/maps/sim_lio_snapshot`입니다. **실물에서는 YAML의
`use_sim_time`을 false로 변경**하고 저장 경로를 사이트에 맞춥니다.
이 launch가 FAST-LIO 자체를 시작하는 것은 아닙니다.

- `mapping.yaml`: 입력 토픽·frame·거리·clock·RViz·저장 경로.
- `height.yaml`: GPU 추정 세부 설정.
- `global.yaml`: 전역 누적 크기·해상도 등.

우선순위는 명시한 CLI 인자 → mapping.yaml → 상세 node 설정입니다.
기존 긴 명령도 유효합니다. YAML 수정은 다음 실행부터 반영되며 실행 중인 지도를
자동 초기화하지 않습니다. `--symlink-install` 설치에서는 source YAML 수정이
설치 경로에도 연결됩니다. 다른 설정 파일은 아래처럼 선택합니다. 파일 경로 항목의
상대 경로는 선택한 YAML 디렉터리 기준이고 `~`는 사용자 홈으로 확장됩니다.

```bash
ros2 launch legged_bringup mapping.launch.py params_file:=/absolute/path/mapping.yaml
```

GenZ-ICP를 선택했으면 B 단계의 `method:=genz`와 함께 다음 입력을 사용합니다.

```bash
ros2 launch legged_bringup mapping.launch.py \
  use_sim_time:=false \
  input_scan:=/livox/lidar \
  odom_topic:=/genz/localization_odom \
  map_frame:=map base_frame:=base_link rviz:=true
```

FAST-LIO의 `/Odometry`와 GenZ의 `/genz/odometry`는 `camera_init` 기준의 local odometry입니다. 기존 PCD·occupancy와 맞는 **map 기준 전역 height map**을 만들 때는 위 표의 전역 localization 결과를 사용합니다. Local odometry만 사용하려면 그 좌표계를 기준으로 지도를 만들고 나중에 map에 정합해야 합니다.

GenZ 예시는 전역 pose로 원시 점군을 변환합니다. 현재 GenZ 설정은 `deskew:false`이므로 FAST-LIO의 움직임 보정 점군과 같은 조건은 아닙니다. 정지 취득부터 확인합니다.

시뮬레이터에서 기존 raw PointCloud2와 Gazebo GT를 비교할 때:

```bash
ros2 launch legged_bringup mapping.launch.py \
  use_sim_time:=true rviz:=true input_scan:=/livox/lidar_PointCloud2 \
  odom_topic:=/ground_truth/odom map_frame:=world base_frame:=BODY
```

이 world 기준 snapshot은 그대로 실물 `map` 지도 세트에 넣지 않습니다. Frame 이름을 바꾸는 것만으로 정합되지 않습니다.

### C.1 원시 점군으로 FastDEM과 비교하는 시뮬레이션 실행

현재 실행 중인 height mapping launch만 Ctrl+C로 종료합니다. Simulator와 FAST-LIO localization은 유지하고, 아래 명령으로 mapper를 하나만 실행합니다. 기존 저장 지도는 유지하며 새 결과는 `maps/sim_raw`에 저장합니다.

```bash
source /home/kim/legged_robot_ws/install/setup.bash
export ROS_DOMAIN_ID=45
ros2 launch legged_bringup mapping.launch.py \
  use_sim_time:=true \
  input_scan:=/livox/lidar_PointCloud2 \
  odom_topic:=/localization \
  map_frame:=map base_frame:=base_link \
  min_range:=0.6 max_range:=3.0 \
  save_dir:=/home/kim/legged_robot_ws/maps/sim_raw \
  rviz:=true
```

`ROS_DOMAIN_ID=45`는 현재 시뮬레이터 세션 값이며, 다른 환경에서는 센서·localizer와 같은 값으로 맞춥니다. 명령은 어느 디렉터리에서도 실행할 수 있습니다. 별도의 `elevation_mapping_cupy` 실행은 필요하지 않습니다. 독립 `rubi_height_mapping`의 CUDA/CuPy 추정기와 C++ 전역 누적 wrapper가 실행됩니다.

원시 점군도 scan timestamp의 `/localization` pose를 기다립니다. 점군 경로에서 FAST-LIO deskew를 우회할 뿐 localization 의존성을 없애지 않습니다. Mapper에는 점별 deskew가 없으므로 실제 보행 중 지도 품질은 보정 점군과 별도로 비교해야 합니다. 기본 입력은 보정 점군으로 유지합니다. 원시 입력의 거리 원점은 LiDAR, 보정 입력은 IMU body이므로 두 입력의 ROI도 엄밀하게 동일하지 않습니다.

FastDEM과의 공정한 성능 비교는 같은 녹화 데이터·point 수·range·resolution·map 크기·발행 주기·pose 시각 정책을 맞춰야 합니다. FastDEM의 latest TF와 mapper의 timestamp 보간 대기를 섞으면 추정기 계산시간 비교가 아닙니다. 비교 시 두 mapper를 동시에 돌리면 자원 경쟁이 생기므로 동일 기록을 따로 재생하는 편이 좋습니다. 실측은 입력→출력 지연, 누락률, CPU·GPU·메모리, 높이 오차를 함께 기록합니다.

### C.2 GPU ray 선택 최적화 비교

기본값 `ray_mode:=all`은 모든 ray를 검사합니다. `ray_mode:=voxel`은 같은 3D voxel의 실제 관측점 하나만 visibility cleanup에 사용합니다. 두 모드 모두 높이 융합에는 전체 입력점을 전달합니다. 발행 주기와 TF/odometry 정책은 바꾸지 않습니다.

기존 mapping launch만 종료한 뒤, 현재 시뮬레이션에서 다음처럼 비교합니다. Simulator·localizer는 유지합니다.

```bash
source /home/kim/legged_robot_ws/install/setup.bash
export ROS_DOMAIN_ID=45
ros2 launch legged_bringup mapping.launch.py \
  use_sim_time:=true \
  input_scan:=/livox/lidar_PointCloud2 \
  odom_topic:=/localization \
  map_frame:=map base_frame:=base_link \
  min_range:=0.6 max_range:=3.0 \
  ray_mode:=voxel ray_voxel_size:=0.05 \
  save_dir:=/home/kim/legged_robot_ws/maps/sim_voxel \
  rviz:=true
```

기존 ray 방식으로 돌아가려면 `ray_mode:=all`로 다시 시작합니다. 지도 상태가 섞이지 않게 두 mapper를 동시에 실행하지 않습니다. 실제 센서에서는 해당 드라이버의 PointCloud2 토픽과 시각/장착 계약을 사용합니다.

`ray_voxel_size:=0.0`은 지도 해상도를 따릅니다. 현재 5 cm 지도에서는 1.25–5 cm 범위의 명시적 값만 허용합니다. XY만 묶지 않고 Z도 포함하며, 실제 점 하나를 선택하므로 같은 XY의 서로 다른 높이가 무조건 합쳐지지는 않습니다. 관측된 높이의 수치적 완전 일치까지 보장하는 옵션은 아닙니다.

현재 PC 시험은 조밀한 중복 입력에서만 뚜렷한 이득이 있었습니다. 저장된 3.2만 점에서는 이득이 거의 없었고 잔상 제거가 느려지는 경우가 있습니다. 초기 구현에서 실패했던 고정 pose 반복성 시험은 0.3.1 snapshot fusion에서 통과했지만, all/voxel의 삭제 정책까지 같아진 것은 아닙니다. 이 옵션은 격리된 시뮬레이션/기록 데이터 비교용이며 기본값은 all입니다. 상세 조건은 [검증 기록](VALIDATION.md)을 참조합니다.

### C.3 읽기·쓰기 분리 적용본 (0.3.1)

새 파라미터 없이 기존 launch에 적용됩니다. 시작 로그에
`Fusion: immutable scan snapshot -> proposal buffers -> per-cell finalization`
문구가 나타납니다. 이미 실행 중인 mapper는 파일 수정만으로 바뀌지 않습니다.

현재 지도를 보관하려면 먼저 `save_map`을 호출하고, 기존 mapping launch를
종료한 뒤 새 터미널에서 아래처럼 시작합니다. Simulator와 localizer의 재시작은
필요하지 않습니다. mapper와 누적 wrapper는 새 세션으로 함께 시작합니다.

```bash
source /home/kim/legged_robot_ws/install/setup.bash
export ROS_DOMAIN_ID=45
ros2 launch legged_bringup mapping.launch.py \
  use_sim_time:=true \
  input_scan:=/livox/lidar_PointCloud2 \
  odom_topic:=/localization \
  map_frame:=map base_frame:=base_link \
  min_range:=0.6 max_range:=3.0 ray_mode:=all \
  save_dir:=/home/kim/legged_robot_ws/maps/sim_snapshot \
  rviz:=true
```

현재 설치에는 빌드가 반영되어 있습니다. 다른 PC로 소스를 옮기면 설치 문서의
빌드를 수행합니다. 원래 workspace의 `/home/kim/rubi_mapping_ws` 코드는 이번
변경 대상이 아닙니다. Localization 입력 오차·deskew 부재를 교정하는 변경도 아닙니다.

## D. 동결 snapshot과 후처리

```bash
ros2 service call /rubi/global_elevation/save_map std_srvs/srv/Trigger '{}'
```

반환된 파일 경로를 기록합니다. 저장 후 온라인 mapping은 계속될 수 있으므로, QC에는 저장된 snapshot 파일을 입력합니다.

```bash
ros2 run rubi_heightmap_postprocess compare \
  --input /absolute/path/map.rghm \
  --output /absolute/path/qc-v1
```

출력은 `candidate/map.rghm`, `candidate/surface.pcd`, `baseline`, `metrics.json`, 비교 보고서입니다. Candidate를 자동으로 승인된 navigation map으로 간주하지 않습니다. 다음을 비교합니다.

- 평면 내부의 높이 분산과 P95 오차가 줄었는가?
- 단차 높이와 경계 주변 통과 판정이 유지되는가?
- Unknown mask가 새 자유공간으로 바뀌지 않았는가?
- PCD/occupancy landmark와 height의 XY가 일치하는가?
- 대표 출발·도착점의 경로가 의도한 통로를 지나는가?

## E. 다른 frame의 높이 지도 정렬

같은 map pose로 만든 지도는 우선 추가 rigid 변환 없이 QC합니다. 다른 frame에서 취득했다면 먼저 구조물 대응점 또는 PCD registration으로 `T_map_from_height`를 측정합니다. 변환 방향은 **height의 점을 map 좌표로 옮기는 행렬**입니다.

```yaml
# T.yaml — 예시 identity는 실제 정합 결과가 아닙니다.
T_map_from_height:
  - [1.0, 0.0, 0.0, 0.0]
  - [0.0, 1.0, 0.0, 0.0]
  - [0.0, 0.0, 1.0, 0.0]
  - [0.0, 0.0, 0.0, 1.0]
```

```bash
python3 tools/align_height.py --input /path/map.rghm \
  --transform /path/T.yaml --output /path/aligned-v1
```

이 도구는 측정한 transform의 적용과 재격자화만 수행합니다. 알려지지 않은 transform을 찾지 않으며, 기울기 오차를 제거하기 위해 임의로 바닥을 눕히지 않습니다. 변환 후 holes와 경계 셀 손실, collision 제외 셀을 확인하고 QC합니다.

## F. Frozen site 배포

`maps/qc.template.yaml`을 복사하여 검사 결과를 기록합니다. 실제 검사 후 `verified:true`, `method`, 잔차와 representative path 결과를 작성합니다.

```bash
python3 tools/package_site.py \
  --id site-a-v1 \
  --pcd /path/localization.pcd \
  --occupancy /path/occupancy.yaml \
  --height /path/qc-v1/candidate/map.rghm \
  --qc /path/qc.yaml \
  --output /path/site-a-v1
```

새 디렉터리 안에는 PCD·occupancy YAML·이미지·height·QC·`site.yaml`이 모입니다. 한 파일만 교체하고 기존 manifest를 계속 사용하면 hash 검사에서 거부됩니다. 수정된 지도를 배포할 때 새 site version을 만듭니다.

## G. Navigation 모드 선택

시뮬레이션 terrain 정책의 `/cmd_vel` 배율은 2026-10-15에 전 축 1배로
수정했습니다. 전진 2배, 횡이동·회전 1.5배로 증폭하던 변환을 제거했습니다.
입력 제한은 X ±0.7 m/s, Y ±0.5 m/s, yaw ±1.0 rad/s이며, 제한 안에서는 입력값이 그대로
보행 정책에 전달됩니다. 이는 `/home/kim/rubi_ws`의 `rubi_gazebo_plugins`
변경이며, `legged_robot_ws`만 복사해서는 시뮬레이터에 적용되지 않습니다.
[변경 패치](patches/terrain_cmd_unity.diff)를 함께 보관합니다.

이 수정본을 빌드한 뒤에는 Gazebo 재시작이 필요합니다. 기존 navigation,
localization, Gazebo를 종료하고 Gazebo → localization → navigation 순으로
다시 실행합니다. 시뮬레이션 시간이 초기화되므로 localization 초기 위치를
확인하고 새 목표를 지정합니다. 기존 `site.yaml`과 frozen height map을 그대로
사용합니다. Joystick 축 변환 및 학습 정책의 observation 정규화는 별개이며
변경하지 않았습니다. 최신 Pure Pursuit 설정은 아래 속도·정지 영역 조정 절을 따릅니다.

```bash
# Nav2: RViz Nav2 Goal / NavigateToPose action
ros2 launch legged_bringup navigate.launch.py \
  site:=/path/site-a-v1/site.yaml mode:=nav2

# 기존 PP: RViz 2D Goal Pose /goal_pose
ros2 launch legged_bringup navigate.launch.py \
  site:=/path/site-a-v1/site.yaml mode:=pursuit
```

GenZ는 `odom_topic:=/genz/odometry`, 별도 cloud 이름은 `cloud_topic:=...`로 지정합니다. 실행 도중 모드를 바꾸려면 이전 navigation launch를 종료하고 새 모드를 시작합니다. 최종 `/cmd_vel` 발행자는 `command_watchdog` 하나가 되어야 합니다.

Nav2 DWB는 장애물 costmap을 고려하는 지역 controller입니다. 현재 DWB가 모든 후보 trajectory의 발판 높이를 직접 검사하는 것은 아닙니다. 좁은 높이 경계 옆에서 전역 경로를 벗어나는 정도는 실물·시뮬레이션에서 확인해야 하며, height 기반 발 디딤 planner를 구현했다고 설명하지 않습니다. PP는 경로가 막히면 정지 후 전역 우회하며, DWB 같은 지역 velocity sampling을 하지 않습니다.

현재 costmap의 obstacle height gate는 map 지면 z=0 기준, 근접 정지 gate는 base_link 기준입니다. 현장 지면 datum과 보행 자세 변화에 맞게 조정합니다. 거리 필터로 생긴 0.6 m 내부 사각지대를 외부 센서나 제어기의 정지 정책 없이 완전히 감지 가능한 영역으로 취급하지 않습니다.

## H. PCD를 새로 만드는 세션

기존 `fast_lio` mapping 소스도 포함했습니다.

```bash
ros2 launch legged_bringup mapping_pcd.launch.py output:=/path/new-survey.pcd
ros2 service call /map_save std_srvs/srv/Trigger '{}'
```

 Localizer와 동시에 같은 입력/TF를 소유하도록 띄우지 않습니다. `mapping_pcd.launch.py`로 별도 취득하고, 결과 PCD를 occupancy·height와 정합한 뒤 site version을 만듭니다. PCD mapping을 기존 frozen navigation과 동시에 실행하는 기본 launch는 제공하지 않습니다.

## I. 반복 검증

```bash
colcon test --packages-select fast_lio_localization rubi_height_mapping \
  rubi_global_heightmap_wrapper rubi_heightmap_postprocess \
  rubi_heightmap_step_wavefront_planner legged_navigation legged_bringup \
  --executor sequential
colcon test-result --verbose

ROS_DOMAIN_ID=174 FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
  python3 tools/smoke_navigation.py pursuit
ROS_DOMAIN_ID=175 FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
  python3 tools/smoke_navigation.py nav2
```

통합 시험은 지정된 격리 domain에서 합성 TF/odom/cloud를 발행합니다. 실제 로봇의 domain에서 실행하지 않습니다. 하드웨어 actuation bridge는 이 workspace에서 자동 시작하지 않습니다.

### 시뮬레이션 Nav2 표시 및 TF 대기 (2026-10-15)

Gazebo를 재생한 상태에서 navigation을 시작합니다. 초기화 중 서비스 응답
타임아웃으로 lifecycle이 inactive/unconfigured에 남으면 원인을 확인한 뒤
navigation launch만 재시작합니다. Frozen map을 별도로 실행했다면 종료합니다.
Navigation launch가 저장 지도 발행기를 포함합니다.

RViz 설정에는 Navigation 2 패널과 `/map`, `/plan`, frozen height가 포함됩니다.
Height 표시의 체크를 끄면 occupancy만 확인할 수 있습니다. Nav2 모드에서는
`Nav2 Goal`을 사용합니다. `2D Goal Pose`는 pursuit 모드용입니다.

현재 시뮬레이션의 raw cloud 수신 → map TF 준비 대기시간은 56개 표본에서
중앙값 0.438초, p95 0.445초, 최대 0.465초(벽시계)였습니다.
Humble ObstacleLayer의 PointCloud2 MessageFilter는 costmap의
`transform_tolerance`를 TF 대기 제한으로 사용하므로 이를 0.3초에서 0.6초로
조정했습니다. 사용되지 않는 `tf_filter_tolerance` 항목은 제거했습니다.
이는 cloud stamp 변경이나 latest TF 대체가 아닙니다. 실물에서 지연을 재측정해야 합니다.

수정 후 모든 lifecycle 노드 active, 464×454 occupancy 수신,
`Height` planner의 ComputePathToPose 성공(445 poses)을 확인했습니다.
검증 요청은 경로 생성만 수행했으며 로봇 주행 목표를 보내지는 않았습니다.

최종 명령 흐름:
`controller_server → /legged/cmd_raw → collision_monitor → /legged/cmd_safe → command_watchdog → /cmd_vel`.
Gazebo의 `legacy_policy_controller`가 `/cmd_vel`을 구독합니다.
목표가 없거나 입력이 오래되면 최종 명령은 0입니다. 실제 추종 성공은 별도 확인합니다.

### Pure Pursuit raw-path 시험 (2026-10-15)

`navigate.launch.py mode:=pursuit`는 Height grid planner의 tracking_path를
custom Pure Pursuit에 연결합니다. 기본 planner.yaml은 smoothing을 끈 상태입니다.
목표는 RViz의 `2D Goal Pose`(`/goal_pose`)로 지정합니다. Nav2 Goal 액션은
이 모드의 입력이 아닙니다. 기존 navigation launch를 종료한 뒤 하나만 실행합니다.

pursuit.yaml: nominal/max linear 0.5 m/s, max angular 0.8 rad/s,
linear acceleration 0.5 m/s², angular acceleration 1.0 rad/s².
곡률 k에 대해 목표 선속도는 0.5/(1+|k|)로 감소하며 설정된 최저/최고값으로
제한됩니다. heading_slowdown_threshold_rad는 현재 구현에서 선언만 하고
사용하지 않으므로 감속 효과를 조정하는 파라미터로 사용하지 않습니다.

선택적인 planner_smooth.yaml은 기존 refiner를 켭니다. 이 refiner는
평활화한 후보의 노드·연결 구간과 비용을 검사하며 조건을 만족하지 못하면
raw 경로로 돌아갑니다. 현재 raw 시험에는 이 파일을 적용하지 않았으며,
별도의 smoothed path를 동시에 발행하는 기능을 추가한 것은 아닙니다.

Pure Pursuit 자체는 장애물 회피기가 아닙니다. 실시간 costmap을 반영한
전역 재계획과 collision monitor를 유지합니다. Frozen height만으로는
동적 장애물과 관측되지 않은 구조물을 충분히 처리할 수 없습니다.
최종 /cmd_vel 끊김 문제는 컨트롤러 변경과 독립적으로 검증해야 합니다.

### Smoothed path 비교 실행

기본 `planner.yaml`은 raw 추종을 유지합니다. 아래 선택 설정은 3점 평균 1회로
XY 경로를 다듬고, 이동한 위치의 높이를 다시 조회합니다. 각 점과 연결 선분을
기존 costmap·높이 증거·지형 clearance 기준으로 검사하며, 전체 비용이 raw보다
5% 넘게 증가하면 raw 경로로 복귀합니다. 기존 height map 자체를 평활화하는
것은 아닙니다. 급한 모서리는 검증 조건에 따라 남을 수 있습니다.

현재 navigation launch만 종료한 후 다음과 같이 실행합니다. Simulator와
localization은 그대로 둡니다. 이 선택 설정은 기본 설정과 smoothing 활성화만
다르고, 동일한 경로 이탈 복구·pose freshness 기준을 사용합니다.

```bash
source ~/legged_robot_ws/install/setup.bash
ros2 launch legged_bringup navigate.launch.py \
  site:=/home/kim/legged_robot_ws/maps/sim_site_v1/site.yaml \
  use_sim_time:=true cloud_topic:=/livox/lidar_PointCloud2 \
  odom_topic:=/Odometry mode:=pursuit rviz:=true \
  planner:=/home/kim/legged_robot_ws/src/legged_bringup/config/planner_smooth.yaml
```

RViz에서 `Raw planned route`를 켜면 주황색 원본과 녹색 `Pure Pursuit route`
(검증을 통과한 추종 경로 또는 raw fallback)를 비교할 수 있습니다.
`[TRACKING PATH REFINER]` 로그에서 후보 수락/거부와 fallback을 확인합니다.

최초 smoothing 비교에서는 `v = 0.5 / (1 + |curvature|)`를 사용했습니다.
이후 요청에 따라 아래 속도·정지 영역 설정으로 변경했습니다. Smoothing 선택은
계속 `planner_smooth.yaml`로 지정합니다.

### 속도·정지 영역 조정 — 2026-10-16 최신 설정

현재 custom `mode:=pursuit`의 다음 실행용 설정입니다. 앞의 시험 이력보다 이 절이 우선합니다.

| 항목 | 직전 | 현재 |
|---|---:|---:|
| Pure Pursuit nominal/max 전진 속도 | 0.6 m/s | 0.7 m/s |
| 추종 중 yaw 상한 | 0.8 rad/s | 1.0 rad/s |
| 제자리 방향 정렬 yaw 상한 | 0.4 rad/s | 0.5 rad/s |
| Gazebo X 명령 상한 | 0.6 m/s | 0.7 m/s |
| 즉시 정지 사각형: 전후 길이 × 좌우 폭 | 0.62 × 0.62 m | 1.0 × 0.9 m |

`config/pursuit.yaml`에서 속도를, `config/collision.yaml`의 `Stop.points`에서
사각형을 바꿉니다. 현재 base_link 중심에서 앞/뒤 각각 0.50 m, 좌/우 각각
0.45 m입니다. `navigate.launch.py`는 이 좌표에서 전후·좌우 반폭을 읽어
지도 예측 검사에도 전달합니다. 더 이상 costmap 반경에서 정사각형을 강제로
생성하지 않습니다. 지원 형태는 원점 중심의 축 정렬 직사각형이며 다른 형태는
시작 시 오류를 내어 두 검사가 서로 다른 영역을 사용하지 않도록 합니다.

현재 목표 선속도는 `v = 0.7 / (1 + 0.5 * |curvature|)`를 최저/최고값으로
제한한 값입니다. Lookahead는 0.65 m입니다. 전진 가속도 0.5 m/s²와 회전
가속도 1.0 rad/s²를 유지합니다. 방향 오차가 1.2 rad(약 69°)를 넘으면 정지 후
정렬하고, 0.5 rad(약 29°) 이내에 들어오면 이동하며 추종합니다. 기존 40° 진입/
14° 해제보다 불필요한 정렬 정지를 줄이는 설정입니다.
Gazebo command 배율은 모든 축에서 1배입니다. 이는 명령 상한이며 실제 보행
속도 달성을 보장하지 않습니다. Nav2 DWB/RPP의 자체 속도 설정은 변경하지 않았습니다.
`planner.yaml`은 raw, `planner_smooth.yaml`은 높이·장애물을 재검증한 smoothing입니다.

빨간 사각형은 물리적인 robot_radius와 별도의 정지 여유입니다. 실제 costmap의
robot_radius 0.30 m, padding 0.01 m는 유지하고, **경로 계산용 복사본**의 여유는
`planner.yaml` / `planner_smooth.yaml`의 `costmap_clearance_m: 0.45`로 정합니다.
Launch는 이 값을 덮어쓰지 않습니다. 0은 추가 마스크만 비활성화하며 기존 Nav2
inflation이나 충돌 감시기를 끄지 않습니다. Pure Pursuit 전역 planner와 Nav2
Height plugin에 같은 값이 전달됩니다.

10월 16일 먼저 적용했던 외접원 반경 `hypot(0.50, 0.45) = 0.6727 m` 강제값은
실제 지도에서 통로를 끊어 경로 생성을 실패시켰으므로 철회했습니다. 같은 지도·목표의
격리 재생에서 0.6727 m는 `frontier_exhausted`, 0.45 m는 경로 생성에 성공했습니다.
현재 값은 사각형의 좌우 반폭 수준의 **탐색 여유**이지, 모든 방향으로 사각형을
회전할 수 있는 공간의 보장이 아닙니다.

실제 장애물/미관측(254/255) 주변 및 지도 경계에 planner 전용 금지 셀(253)을
만들며, 기존 inflation 1~253을 seed로 다시 확장하지 않습니다. 셀 면적과
smoothing된 점 위치를 고려한 격자 여유도 포함하므로 0.45 m는 기존 0.31 m에
단순히 더하는 값이 아닙니다. Custom planner는 원본 지도 내용이 바뀔 때만
이 복사본을 갱신합니다.

최종 충돌 감시기는 계속 **원본 costmap과 1.0 × 0.9 m 사각형**으로 현재 자세와
예상 진행 궤적을 검사합니다. 회전 모서리·추종 오차·새 장애물 때문에 최종 정지는
여전히 가능합니다. 경로가 나오는 것과 실제 통과 성공은 다릅니다. 전체 회전
footprint를 계획 단계에서 보장하려면 yaw를 포함한 탐색/궤적 검증이 추가로 필요합니다.
이번 수정은 그런 기능을 구현하거나 실제로 겹친 사각형을 무시하는 변경이 아닙니다.

정상적인 soft replan은 이전 속도 명령과 가속도 제한 상태를 이어받습니다.
반면 empty path를 보내는 hard replan, 잘못된 경로, 도착 후 새 경로는 0부터
시작합니다. 잦은 유효 경로 교체만으로 매번 속도가 0부터 다시 올라가는 현상을 줄였습니다.

### 장애물 인식부터 최종 명령까지

현재 `mode:=pursuit`는 다음 구조입니다. 물체의 종류를 분류하는 AI 인식은 없으며,
점의 좌표·높이·거리와 지도상의 점유 여부로 판단합니다.

```mermaid
flowchart TD
  L[Livox PointCloud2] --> F[유효 XYZ + 센서 거리 0.6~5 m]
  F --> C[센서 좌표 점군 /legged/obstacles]
  F --> B[고정 장착 TF → 몸체 점군 /legged/collision_points]
  O[저장된 2D Occupancy Map] --> G[전역 costmap]
  C --> G
  H[동결 height map] --> P[높이 기반 전역 경로 · 재계획]
  G --> P
  P --> S[선택한 smoothing · 경로 재검증]
  S --> T[Pure Pursuit → /legged/cmd_raw]
  B --> M[Collision Monitor · 빨간 사각형]
  T --> M
  M --> W[Watchdog · /legged/cmd_safe 검사]
  G --> W
  W --> V[/cmd_vel]
```

1. `obstacle_cloud`는 유한한 XYZ를 확인하고 센서 원점에서의 **3차원 거리**
   0.6~5.0 m에 해당하는 점을 남깁니다. 점군 stamp는 유지합니다.
2. Costmap용 `/legged/obstacles`는 센서 좌표계를 유지하여 센서 원점부터
   ray clearing을 할 수 있게 합니다. 기존 occupancy와 합쳐 벽·기둥 및 새 관측을
   경로 계산에 반영합니다. 현재 신규 장애물 marking은 최대 4 m, map Z
   0.2~2.0 m의 점을 사용합니다. 이는 **국소 바닥 위 20 cm**라는 뜻이 아닙니다.
   지도 높이 원점이 달라지면 gate를 조정해야 합니다. 기존 static map의 점유는
   이 신규 점군 높이 gate와 별개입니다.
3. 근접 정지용 점군은 `/tf_static`의 센서 장착 변환으로 base_link에 옮깁니다.
   Collision Monitor는 몸체 Z -0.35~1.5 m 범위의 점 중 빨간 사각형 안에
   **3개 이상** 들어오면 선속도·각속도 모두 0으로 만듭니다. 현재 Humble의
   `max_points: 2`는 “2개 초과” 조건입니다.
4. 최종 watchdog는 같은 직사각형이 현재 및 향후 **1초** 동안 지나갈 영역을
   기존 global costmap에서 확인합니다. Cost 254(장애물), 255(미관측), 지도 밖은
   정지 조건입니다. Inflation 1~253을 다시 장애물로 취급하지 않아 여유가
   이중 적용되지 않습니다. 회전하는 모서리와 샘플 사이의 이동도 검사합니다.
5. 오래된 점군·odom·costmap·TF 또는 끊긴 상위 명령도 최종 `/cmd_vel`을 0으로
   만듭니다. 현재 pointcloud age 제한은 0.4초입니다.

따라서 빨간 사각형 밖이라도 **예상 진행 영역의 충돌로 먼저 정지**할 수 있습니다.
0.7 m/s 직진이면 중심의 0.7 m 이동과 전방 반길이 0.5 m를 합쳐 약 1.2 m
앞까지 검사합니다(격자·샘플 여유 별도). 이는 일정한 속도 명령을 가정한 검사이며
측정된 보행 제동 모델은 아닙니다. 0.6 m 거리 필터 안의 장애물은 **지도에 이미
알려진 경우** 이 검사로 보완합니다. 사각지대에 새로 들어온 미관측 물체까지
감지하는 기능은 아닙니다.

Pure Pursuit는 주어진 경로를 추종하며 자체적으로 옆 우회 궤적을 만들지 않습니다.
새 장애물의 우회는 전역 costmap을 반영한 전역 재계획이 담당하고, 충돌 감시는
정지를 담당합니다. 현재 custom pursuit에는 별도 Nav2 local costmap 서버가 없습니다.
동결 height map은 지면 높이와 단차 비용을 제공하며, 이 점군 기반 장애물 감시와
함께 사용합니다. 실시간 점군에서 바닥을 찾아 상대 높이를 추정하는 단계는 아직 없습니다.

```bash
ros2 topic echo /legged/motion_status
```

`costmap_collision`: 현재 직사각형과 장애물/미관측 셀 겹침.
`predicted_costmap_collision`: 예상 진행 영역에서 겹침.
`collision_or_upstream_stop`: 최근 nonzero raw 명령에 대해 safe 명령이 zero인 상태.
`collision_cloud_timeout`, `costmap_pose_stale`: 센서 또는 pose가 오래됨.

Simulator library도 바뀌었으므로 **navigation → localization → Gazebo**를 종료하고
**Gazebo → localization → navigation** 순서로 재실행해야 0.7 m/s 상한까지 적용됩니다.
기존 world, PCD, occupancy, frozen height map과 smoothing 실행 명령을 사용합니다.
위 Gazebo 재시작은 10월 15일 0.7 m/s 변경에 대한 안내입니다. **10월 16일의
경로 여유·추종 수정만 적용할 때는 navigation만 종료 후 같은 명령으로 재실행**하면
됩니다. 이번 작업에서 사용자 프로세스를 재시작하거나 목표를 전송하지 않았습니다.

근거: [점군 전처리](../src/legged_navigation/src/obstacle_cloud.cpp),
[costmap 설정](../src/legged_bringup/config/costmap.yaml),
[공통 정지 영역](../src/legged_bringup/config/collision.yaml),
[Humble Collision Monitor의 점 개수 비교](https://github.com/ros-navigation/navigation2/blob/humble/nav2_collision_monitor/src/collision_monitor_node.cpp#L378).

### 추종·충돌 감시 복구 적용본 (2026-10-15)

이 항목은 위의 초기 Pure Pursuit 시험 설명 중 `no_forward_target` 정지와
미래 TF 요청 문제를 보완한 현재 동작입니다. 백업은
`backups/navigation_recovery_20261015`에 있습니다.

- **Heading recovery:** 경로의 진행 방향을 따라 lookahead를 선택합니다.
  목표가 뒤에 있거나 방향 오차가 0.7 rad를 넘으면 감속 후 제자리 정렬합니다.
  오차가 0.25 rad 이하일 때 추종으로 복귀합니다. 정렬 회전은 최대 0.4 rad/s,
  기존 각가속도 제한을 유지합니다. 경로 이탈 0.4 m 제한과 stale TF 정지는 유지합니다.
  목표가 뒤라는 이유로 경로의 먼 전방점을 찾아 중간 구간을 건너뛰지 않습니다.
- **Collision input:** `/legged/obstacles`는 기존 센서 좌표와 원점을 유지하므로
  costmap ray clearing에 사용합니다. 별도로 `/tf_static`의 장착 변환만 이용해
  `/legged/collision_points`를 base_link로 발행합니다. 입력 scan stamp와 거리 필터는
  유지합니다. 동적 localization TF를 장착 TF 대신 사용하지 않습니다.
- **Near-body stop:** Collision monitor는 변환된 몸체 점군을 사용하고
  `base_shift_correction: false`입니다. 이는 취득 시각 이후의 몸체 이동 보정을
  생략하는 선택이며, 충돌 검사 비활성화가 아닙니다. 0.4초 age 제한과 기존
  Stop polygon 크기를 유지합니다. 0.5 m/s에서 0.4초 동안 0.2 m 이동 가능하므로
  실물 배포 시 제동거리·회전·데이터 나이를 포함해 정지 여유를 재검증해야 합니다.
  수학적으로 완전한 움직임 보상이나 실물 안전 인증을 제공하지 않습니다.
- **Fail closed:** 장착 TF 없음, 유효 반환점 없음, 오래된 점군이면 collision
  입력을 준비된 것으로 취급하지 않습니다. Watchdog는 변환에 성공한 점군과
  입력 상태를 함께 확인합니다. 오래된 safe command/odom/costmap도 계속 정지시킵니다.
- **Single instance:** 같은 ROS domain에서 이 launch를 두 번 실행하면 두 번째는
  시작 전에 거부합니다. 다른 launch나 외부 /cmd_vel 발행기까지 막는 것은 아닙니다.

현재 raw-path 추종과 nominal/max 0.5 m/s, lookahead 0.35 m는 유지합니다.
Smoothing과 lookahead 튜닝은 이 수정에 포함하지 않습니다. `planner_smooth.yaml`은
계속 선택 사항이며 현재 실행에는 적용하지 않았습니다.

진단:

```bash
ros2 topic echo /legged/motion_status
ros2 topic echo /legged/tracking_state
ros2 topic echo /legged/collision_input_state
```

`aligning_heading`은 제자리 정렬, `braking_for_alignment`는 정렬 전 감속입니다.
`cross_track_limit`은 경로 이탈, `missing_static_mount`는 장착 TF 누락,
`collision_cloud_timeout`은 신선한 충돌 점군 없음입니다. 목표가 없는 idle 상태에서는
Nav2 monitor가 zero 발행을 쉬어 safe_command_timeout으로 최종 0을 유지할 수 있습니다.
`collision_or_upstream_stop`은 최근 nonzero raw 명령과 zero safe 명령의 관측 결과이며,
개별 장애물의 종류를 판별했다는 뜻은 아닙니다.

RViz에 Stop zone 표시와 선택 가능한 Collision input 점군 표시를 추가했습니다.
가까운 장애물 때문인지 확인할 때 사용합니다. /goal_pose를 다시 지정하기 전에는
재시작 이전 목표를 자동 재개하지 않습니다.

## Local costmap + Pure Pursuit 선택 — 2026-10-15

상단 통로 정지 로그에서 `Stop polygon` 정지 후 `cross_track_limit`
(0.409–0.415 m > 0.4 m)이 확인됐습니다. 기존 재계획기는 남은 경로 자체가
유효하면 더 비싼 새 경로를 거절했기 때문에, 로봇이 경로 밖에서 멈출 수 있었습니다.
이제 custom pursuit에서 이탈 한도를 넘으면 현재 위치부터 동일 목표로 강제
재계획합니다. 기존 경로를 비우고 높이·costmap 검사를 통과한 새 경로만 채택합니다.
막힌 시작점이면 정지를 유지하며 재시도합니다. 지도 내용이 변하지 않아도 작동합니다.
`replanning.max_tracking_error_m`은 pursuit의 `max_cross_track_error_m` 이하로
설정해야 합니다. 두 값의 기본값은 0.4 m입니다.

Nav2의 local costmap을 사용하는 Pure Pursuit는 별도 선택할 수 있습니다.
기존 navigate launch를 종료한 뒤 실행합니다. Gazebo와 localization은 유지합니다.

```bash
cd /home/kim/legged_robot_ws
source install/setup.bash
ros2 launch legged_bringup navigate.launch.py \
  site:=/home/kim/legged_robot_ws/maps/sim_site_v1/site.yaml \
  use_sim_time:=true cloud_topic:=/livox/lidar_PointCloud2 \
  odom_topic:=/Odometry mode:=nav2 controller:=rpp
```

RViz에서는 **Nav2 Goal**을 사용합니다. **Local costmap** 표시를 켜면 6×6 m
주변 지도를 볼 수 있습니다. custom `mode:=pursuit`는 기존 **2D Goal Pose**를
사용하며 local costmap을 소비하지 않습니다. `mode:=nav2 controller:=dwb`는 기존 DWB입니다.

```mermaid
flowchart LR
  H[동결 height map + 전역 costmap] --> P[높이 기반 전역 경로 · 1 Hz 재계획]
  L[LiDAR] --> C[Nav2 local costmap]
  P --> R[Nav2 Regulated Pure Pursuit]
  C --> R
  R --> M[Collision Monitor]
  L --> M
  M --> W[입력 신선도 watchdog]
  W --> V[/cmd_vel]
```

`config/rpp.yaml`은 목표 속도 0.5 m/s, 속도별 lookahead 0.4–0.8 m,
곡률·장애물 근접도 감속, 예상 진행 궤적 충돌 검사를 설정합니다.
raw 전역 경로를 유지합니다. RPP가 자체적으로 옆으로 우회하는 궤적을 찾지는 않으며,
전역 재계획기가 우회 경로를 제공해야 합니다. local 궤적 최적화가 필요하면 MPPI가
별도 선택지입니다. RPP는 RUBI의 관절·보행 동역학 모델을 대체하지 않습니다.

**정지 범위는 이번 변경에서 축소하지 않았습니다.** Stop 영역은 앞 0.9 m,
뒤 0.7 m, 좌우 0.45 m이며, costmap 반경 0.3 m와 다릅니다. RPP를 선택해도
이 영역의 장애물은 최종 명령을 정지시킵니다. footprint, 보행 중 몸체 범위,
센서 지연·제동 여유를 측정해 두 기준을 맞추는 작업은 별도입니다.
costmap의 높이 필터는 map Z 기준이므로 현장의 바닥 기준 높이도 확인해야 합니다.

이번 검증은 분리된 ROS domain의 합성 입력입니다. 실제 시뮬레이션에서의
주행 흔들림 감소와 해당 기둥 옆 통과는 아직 확인하지 않았습니다.
공식 참고: [Humble RPP](https://api.nav2.org/nav2-humble/html/md_nav2_regulated_pure_pursuit_controller_README.html),
[Nav2 controller 선택](https://docs.nav2.org/rolling/configuration_and_development/tuning_guide/).

후속 live 측정에서는 Stop 사각형의 **후방 좌측 모서리**에 139개 점이 들어온
프레임을 확인했습니다(x 약 -0.70 m, y 약 +0.42 m). 전진 중에도 이 고정 사각형
어디에든 점이 있으면 전체 명령을 멈춥니다. 이후 RPP 자체의 예상 궤적 충돌로도
목표가 중단됐습니다. 따라서 RPP 추가만으로 해당 구간 통과가 해결됐다고 볼 수 없습니다.
[측정한 충돌 범위](../archive/20261109/validation/navigation_recovery/rpp_stop_cloud.png)를 참고하세요.
주행 방향·속도를 반영한 충돌 여유와 실제 footprint를 통일하고, 경로 주변에서
직접 우회 궤적을 찾아야 한다면 MPPI 같은 local controller를 별도로 검증해야 합니다.
