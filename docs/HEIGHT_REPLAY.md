# Rosbag 재생

## RViz 한 창으로 발표용 bag 재생

[발표용 전체 녹화](OPERATIONS.md)로 만든 bag에서 실험 당시의 주행 결과를 표시합니다. 아래 환경을 **두 Bash 터미널 모두**에 적용합니다. 발표에는 RViz와 player 두 프로세스만 실행하면 됩니다.

```bash
source /opt/ros/humble/setup.bash
source ~/legged_robot_ws/install/setup.bash
export ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
CFG="$HOME/legged_robot_ws/src/navigation_dds/config"
```

```bash
# A · 실물 localization과 같은 RViz 화면
RVIZ="$(ros2 pkg prefix --share fast_lio_localization)/rviz/fastlio_localization.rviz"
ros2 run navigation_dds with_profile common \
  rviz2 -d "$RVIZ" --ros-args -p use_sim_time:=true
```

```bash
# B · BAG에 발표할 녹화 폴더의 실제 경로를 지정
BAG="$HOME/rosbag_record/rubi_demo_YYYYMMDD_HHMMSS"
mapfile -t VIEW < "$CFG/presentation_rviz_topics.txt"
ros2 run navigation_dds with_profile large \
  ros2 bag play "$BAG" --clock 50 --start-paused --read-ahead-queue-size 100 \
  --qos-profile-overrides-path "$CFG/presentation_map_qos.yaml" --topics "${VIEW[@]}"
```

RViz가 열리면 B에서 **Space**를 누릅니다. Fixed Frame은 `map`입니다. **Frozen global height**는 `/legged/height`, **Global plan**은 `/plan`, **DWB local plan**은 `/local_plan`을 표시합니다. 새 지도를 생성하는 장면은 **Live global height (mapping only)**를 켜면 보입니다. 현장에서 저장한 시점과 레이아웃을 쓰려면 A의 `RVIZ`를 해당 `.rviz` 파일로 지정합니다.

[화면용 재생 목록](../src/navigation_dds/config/presentation_rviz_topics.txt)은 RViz 표시와 TF에 필요한 토픽만 발행합니다. 녹화된 `/cmd_vel`과 조이스틱 입력은 bag에 보관되며 발표 domain으로 발행되지 않습니다. 실물 녹화의 재생 시계는 player의 `--clock 50`이 제공합니다. 이때 player가 `/cloud_pcd`도 송신하므로 `large` 프로필을 씁니다.

기존 `[26.11.02]RUBI_nav_second_success`의 metadata에는 `/map` 1건, `/plan` 126건, `/local_plan` 873건이 있고 `/legged/height`는 없습니다. 이 bag만 재생하면 당시 경로를 볼 수 있고, 현재 확정 높이 지도를 올려 새 경로를 검증하려면 아래 navigation 재계산 절차를 사용합니다.

## PointCloud2 + IMU 두 토픽으로 localization·navigation 재계산

11월 2일 RUBI bag에서 LiDAR driver 역할을 player로 대체합니다. 각 터미널에서 아래 환경을 먼저 적용합니다. 실물 운용은 domain 13, 이 재생은 88입니다.

```bash
source /opt/ros/humble/setup.bash
source ~/legged_robot_ws/install/setup.bash
export LEGGED_WS="$HOME/legged_robot_ws"
export OPS="$LEGGED_WS/src/navigation_dds/config/operation_rubi.yaml"
export BAG="$HOME/rosbag_record/[26.11.02]RUBI_nav_second_success"
export ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

```bash
# A · 정지 상태에서 player 준비
ros2 bag play "$BAG" --clock 50 --start-paused --read-ahead-queue-size 100 \
  --qos-profile-overrides-path "$LEGGED_WS/src/navigation_dds/config/replay_livox_qos.yaml" \
  --topics /livox/lidar_pointcloud /livox/imu

# B · FAST-LIO와 RViz
FASTLIO_DIAG_TIMING=1 ros2 launch fast_lio_localization localization_nav_rubi.launch.py \
  operation:=replay operation_file:="$OPS"

# C · B에서 초기 정합을 확인한 뒤 시작
ros2 launch legged_navigation height_hybrid_navigation.launch.py \
  operation:=replay operation_file:="$OPS"
```

1. B를 띄우고 A에서 **Space**를 눌러 센서를 공급합니다. B의 RViz에서 **2D Pose Estimate**로 초기 위치·방향을 정하고, 실시간 점군과 PCD의 벽을 겹쳐 봅니다.
2. C에서 **Frozen terrain READY**와 Nav2 활성화를 확인한 뒤 같은 RViz에서 **Nav2 Goal**을 찍습니다. 새 경로는 `/plan`, 속도는 `/legged/replay/cmd_vel`입니다. 실물 Livox·LCM은 종료한 상태에서 진행합니다.
3. Player가 `/clock`을 생성합니다. FAST-LIO가 TF·odom·중력·속도를 새로 발행하고, Nav2는 같은 PointCloud2에서 `/livox/lidar_costmap`을 생성합니다. 녹화 TF·odom·속도 명령은 이 재생 입력에서 제외합니다.

11월 2일 bag의 PointCloud2 발행 QoS는 `best_effort`, 이 workspace의 FAST-LIO PointCloud2 구독은 `reliable`입니다. 위 [player QoS 파일](../src/navigation_dds/config/replay_livox_qos.yaml)을 지정해야 양쪽이 연결됩니다. 빠뜨리면 `ros2 topic hz`에는 점군이 보여도 FAST-LIO에는 `RELIABILITY_QOS_POLICY` 경고가 뜨고 `/Odometry`가 나오지 않습니다. 이 옵션은 재생할 때만 발행 정책을 선택합니다. 원본 bag에 빠진 IMU 샘플은 복구하지 않습니다.

`replay.localization.input_format: pointcloud2`와 `replay.localization.lidar_topic: /livox/lidar_pointcloud`가 한 쌍입니다. FAST-LIO는 `mid360s_nav.yaml`의 MID-360 디코더를 선택합니다. 이 bag에서 확인한 필드는 `x/y/z/intensity`(FLOAT32), `tag/line`(UINT8), 스캔 header와 같은 시간 기준의 절대 ns `timestamp`(FLOAT64)입니다. XYZ만 남긴 일반 점군은 이 모드로 역왜곡할 수 없습니다. 로봇이 RViz에서 튀면 `/Odometry`, `/fastlio/up_body`와 IMU 시각부터 확인합니다. 이 bag에는 원본 IMU 공백이 있어 센서를 재계산한 위치가 녹화 위치와 달라질 수 있습니다.

두 토픽 검증 결과는 로컬 전용 ROS domain에서 IMU 384개, PointCloud2 10개를 수신한 시점에 FAST-LIO `/Odometry` 7개가 발행된 것입니다. 이는 입력 연결과 로컬 추정 출력을 점검한 결과입니다. 현장 PCD 정합과 전체 경로 추종은 위 B·C 절차에서 별도로 확인합니다.

## 실물과 같은 CustomMsg 입력으로 대조할 때 (11월 2일)

각 터미널에서 위 두 토픽 절차와 같은 환경을 적용합니다(`OPS`, domain 88 포함). `operation:=replay`는 launch의 시뮬레이션 시계·지도·출력을 설정합니다. Bag에서 공급할 입력은 player의 `--topics`로 선택합니다.

```bash
# 터미널 A · Livox driver를 대체하는 player
ros2 bag play "$HOME/rosbag_record/[26.11.02]RUBI_nav_second_success" \
  --clock 50 --start-paused --read-ahead-queue-size 100 \
  --topics /livox/lidar /livox/imu /livox/lidar_pointcloud

# 터미널 B · FAST-LIO localization과 RViz
FASTLIO_DIAG_TIMING=1 ros2 launch fast_lio_localization localization_nav_rubi.launch.py operation:=replay operation_file:="$OPS" \
  input_format:=custommsg lidar_topic:=/livox/lidar

# 터미널 C · 확정 Height Map + 2D 지도, B의 RViz 사용
ros2 launch legged_navigation height_hybrid_navigation.launch.py operation:=replay operation_file:="$OPS" rviz:=false
```

B를 띄운 다음 A에서 **Space**로 재생을 시작합니다. RViz **2D Pose Estimate**로 초기 위치를 지정해 실시간 점군과 PCD가 정합되면 C를 실행합니다. FAST-LIO는 `/livox/lidar`(CustomMsg)와 `/livox/imu`를 사용하고, Nav2 장애물 처리는 `/livox/lidar_pointcloud`(PointCloud2)를 사용합니다. TF·odom·속도 추정은 실행 중인 노드가 발행하고 `/clock`은 player가 생성합니다.

이 bag의 이전 센서 재계산 시험에서는 32초 시점에 녹화 위치와 약 26.9m 차이가 있었습니다. 동일한 복도 출발점에서 planner만 비교할 때는 아래의 녹화 위치 재생 방식을 사용할 수 있습니다.

## 11월 2일 녹화 위치로 navigation만 검증

두 터미널에서 위 두 토픽 절차와 같은 환경을 적용합니다(`OPS`, domain 88 포함).

```bash
# 터미널 A
ros2 launch legged_navigation height_hybrid_navigation.launch.py operation:=replay operation_file:="$OPS" rviz:=true

# 터미널 B
ros2 bag play "$HOME/rosbag_record/[26.11.02]RUBI_nav_second_success" \
  --clock 50 --read-ahead-queue-size 100 \
  --topics /tf /tf_static /Odometry /localization /livox/lidar_pointcloud \
           /fastlio/up_body /fastlio/odometry_velocity
```

위 재생은 녹화 위치·TF와 센서 입력을 공급합니다. Nav2가 확정 높이 지도와 2D 지도로 새 `/plan`을 계산하며 새 속도 출력은 `/legged/replay/cmd_vel`입니다. 실물 driver·localization·LCM은 실행하지 않습니다. 원래 복도 출발을 비교하려면 재생 약 80초에 player 터미널에서 **Space**로 멈추고 RViz **Nav2 Goal**을 찍습니다. Publish Point로 goal을 잡거나 79.9초 자동 정지가 필요할 때는 아래의 재현 도구를 사용합니다.

## 확정 지도 검증 · 11월 2일 복도 출발

`[26.11.02]RUBI_nav_second_success`의 녹화 TF와 localization을 사용합니다. 두 터미널에서 ROS 환경과 재생 domain을 맞춥니다.

```bash
source /opt/ros/humble/setup.bash
source ~/legged_robot_ws/install/setup.bash
export ROS_DOMAIN_ID=88 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export OPS="$HOME/legged_robot_ws/src/navigation_dds/config/operation_rubi.yaml"
```

**터미널 A**에서 navigation과 RViz 한 창을 실행합니다.

```bash
ros2 launch legged_navigation height_hybrid_navigation.launch.py operation:=replay operation_file:="$OPS" rviz:=true
```

**터미널 B**에서 기록된 위치 재생과 경로 비교를 시작합니다.

```bash
ros2 run legged_navigation replay_corridor --operation-file "$OPS"
```

79.9초에 bag이 멈추면 도구가 원래 goal `(48.09, 41.66)m`으로 경로를 요청합니다. 같은 RViz의 **Publish Point**로 다른 goal도 비교할 수 있습니다. 설정은 [운용 YAML](../src/navigation_dds/config/operation_rubi.yaml)에 있습니다.

| 바꿀 항목 | YAML 키 |
|---|---|
| 장소 및 확정 높이 지도 | `site`, `common.navigation.height_map` |
| 2D 지도, 입력 cloud, CPU | `common.hybrid.map_yaml`, `common.navigation.cloud_topic`, `common.navigation.cpu_affinity` |
| RViz, 속도 출력, clock | `replay.navigation.rviz`, `replay.navigation.cmd_vel_topic`, `replay.all.use_sim_time` |
| Bag, 자동 정지 시각, 목표 좌표, 재생 토픽과 속도 | `corridor_replay.bag / pause_at_s / goal_xy_m / topics / rate` |
| Player CPU 및 DDS 전송 | `corridor_replay.cpu_affinity / dds_profile` |

별도 YAML을 쓸 때는 A에 `operation_file:=/path/to/operation.yaml`, B에 `--operation-file /path/to/operation.yaml`을 전달합니다. 속도 출력은 `/legged/replay/cmd_vel`입니다.

## 11월 19일 센서 bag으로 FAST-LIO부터 실행

대상: `/home/rclab/rosbag_record/[26.11.19]rubi_pointfoot_nav`.
2026-11-23에 metadata와 실제 메시지를 확인했습니다. 길이는 **555.684초(약 9분 16초)**이며, 전체 191,933개 메시지 중 다음 센서 입력을 사용합니다.

| 토픽 | 타입 | 기록 개수 |
|---|---|---:|
| `/livox/lidar` | `livox_ros_driver2/msg/CustomMsg` | 5,557 |
| `/livox/lidar_pointcloud` | `sensor_msgs/msg/PointCloud2` | 2,779 |
| `/livox/imu` | `sensor_msgs/msg/Imu` | 40,597 |

**Bag player가 Livox driver를 대신합니다.** 아래 재생 중에는 실물 driver를 함께 실행하지 않습니다. Localization·height mapping·navigation은 각 패키지의 launch를 직접 실행합니다.

## 0. 모든 터미널에서 공통 환경

```bash
source /opt/ros/humble/setup.bash
source ~/legged_robot_ws/install/setup.bash
export ROS_DOMAIN_ID=87
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0
export LEGGED_WS="$HOME/legged_robot_ws"
export OPS="$LEGGED_WS/src/navigation_dds/config/operation_rubi.yaml"
export BAG="$HOME/rosbag_record/[26.11.19]rubi_pointfoot_nav"
```

87은 재생용 domain입니다. 실물 운용 domain 13과 구분하며, RViz를 포함한 모든 재생 터미널에서 같아야 합니다. `ROS_LOCALHOST_ONLY=0`은 native DDS profile 사용 조건입니다. 이 장비에는 위 지도 세 파일이 존재합니다. 지정한 PCD의 XYZ 565,765개는 bag의 첫 `/cloud_pcd` 메시지와 순서까지 일치함을 확인했습니다. Git clone만 받은 장비에는 실제 지도 데이터를 별도 복사해야 합니다. `[]`가 있는 bag 경로는 항상 큰따옴표로 감쌉니다.



## 운용 파일

[운용 YAML](../src/navigation_dds/config/operation_rubi.yaml)의 `common`은 장비의 PCD·토픽·frame, `live`와 `replay`는 clock·속도 출력·지도 선택을 담습니다. 재생은 `operation:=replay`, 실물은 `operation:=live`를 각 패키지에 전달합니다. 인자 우선순위는 명시한 CLI 값 → 선택한 운용 모드 → common → 패키지 기본값입니다. 장비별 사본은 같은 명령에 `operation_file:=/절대경로/현장.yaml`을 한 번 더 붙입니다. 이 안내의 프로필은 `$OPS`입니다.
이 센서 재생 예제도 같은 확정 지도를 사용합니다. Localization에서 RViz를 열기 때문에 다음 4A·4B 명령에서는 navigation의 RViz를 끕니다.

## 1. Player 준비 — 터미널 A

```bash
ros2 run navigation_dds with_profile common \
  taskset -c 0,1 \
  ros2 bag play "$BAG" \
  --clock 100 --rate 1.0 --start-paused --read-ahead-queue-size 500 \
  --topics /livox/lidar /livox/imu /livox/lidar_pointcloud
```

처음에는 일시정지 상태입니다. Player 터미널의 **Space**로 시작/일시정지합니다. `/clock`은 player가 생성합니다.

Bag에는 `/tf`, `/tf_static`, `/Odometry`, `/localization`, `/map`, `/plan`, `/cmd_vel`도 기록되어 있지만 이 실행에서는 재생하지 않습니다. 새 FAST-LIO 결과·TF·Nav2 결과와 중복시키지 않기 위해 입력 세 토픽만 선택합니다. 녹화된 `/initialpose`는 0개이므로 재생 후 초기 위치를 새로 지정합니다.

## 2. FAST-LIO Localization — 터미널 B

```bash
FASTLIO_DIAG_TIMING=1 ros2 launch fast_lio_localization localization_nav_rubi.launch.py operation:=replay operation_file:="$OPS" \
  input_format:=custommsg lidar_topic:=/livox/lidar
```

**RViz는 localization에서 띄운 한 창을 계속 사용합니다.** 기본 구성 `src/FAST_LIO_LOCALIZATION2/rviz/fastlio_localization.rviz`에 Nav2 패널·**Nav2 Goal**, `/plan`, local/global costmap, frozen `/legged/height`를 포함합니다. Navigation의 `rviz` 기본값은 false이며 이 프로필도 같은 값을 사용합니다. 별도 창이 필요할 때만 navigation에 `rviz:=true`를 지정합니다.

Height mapping 중에는 Displays에서 **Live global height (mapping only)**를 켜고, 저장 후 navigation으로 전환하면 끕니다. Height-only에서는 **Occupancy map (hybrid)**와 **Global costmap (hybrid)**를 끕니다. 이 체크는 시각화에만 영향을 줍니다.

이미 열린 localization RViz는 **File → Open Config**로 `~/legged_robot_ws/src/FAST_LIO_LOCALIZATION2/rviz/fastlio_localization.rviz`를 불러오면 됩니다. Localization 프로세스를 재시작할 필요는 없습니다.

A에서 Space로 재생을 시작합니다. IMU 초기화와 scan 수신 후 RViz의 **2D Pose Estimate**로 PCD 상의 로봇 위치·방향을 지정합니다. 시작 부근 녹화 pose는 대략 `x=0.225 m`, `y=-0.128 m`, yaw `-0.079 rad`입니다. 초기 위치를 고르는 참고값이며 새 정합 결과의 검증을 대체하지 않습니다. 벽·기둥에 실시간 점군이 맞는지 확인합니다.

```bash
ros2 run tf2_ros tf2_echo map base_link
ros2 topic info /cloud_registered_body
ros2 topic hz /map_to_odom --window 10
```

`/localization`이나 TF가 나온다는 사실만으로 전역 정합 성공을 판단하지 않습니다. 초기 pose를 지정하고 `/map_to_odom` 갱신 및 RViz의 실제 정합을 확인한 뒤 mapper를 시작합니다.

TF가 준비된 뒤 3번 또는 4번을 실행합니다. 기록된 정합 TF를 그대로 재생하는 별도 방식과 혼용하지 않습니다.

## 3. 이번 bag으로 Height Map을 새로 만들 때 — 터미널 C

기존 frozen map으로 navigation만 바로 확인하려면 **이 단계를 건너뛰고 4번**으로 갑니다.

```bash
source "$LEGGED_WS/tools/height_env.sh"
ros2 launch rubi_height_mapping mapping.launch.py operation:=replay operation_file:="$OPS"
```

원하는 구역이 관측된 뒤 별도 공통 환경 터미널에서 저장합니다:

```bash
ros2 run navigation_dds with_profile common \
  ros2 service call /rubi/global_elevation/save_map std_srvs/srv/Trigger '{}'
```

`success: true`와 응답의 `run_<번호>` 폴더를 확인합니다. Player를 Space로 잠시 멈추고 mapper를 Ctrl+C로 종료합니다. 새 지도를 사용할 때는 운용 YAML의 경로를 저장 응답과 맞춥니다.

```yaml
replay:
  navigation:
    height_map: ${LEGGED_WS}/src/rubi_height_mapping/maps/replay/run_실제번호/map.rghm
```

`surface.pcd`가 아니라 `map.rghm`을 넣습니다. 재생을 다 끝냈다면 새 검증을 시작할 때 localization과 navigation도 재시작하고 bag을 처음부터 재생합니다. 기존 노드를 그대로 둔 채 `--loop`로 시간을 되돌리지 않습니다. 새 Height Map의 좌표계는 localization이 정합한 `map`입니다.

## 4A. Height + 2D 지도 navigation — 터미널 C

```bash
ros2 launch legged_navigation height_hybrid_navigation.launch.py operation:=replay operation_file:="$OPS" rviz:=false
```

## 4B. Height-only navigation — 4A 대신 실행

```bash
ros2 launch legged_navigation height_only_navigation.launch.py operation:=replay operation_file:="$OPS" rviz:=false
```

4A와 4B는 한 번에 하나만 실행합니다. Height-only의 global planner에는 2D map/inflation을 넣지 않고, local LiDAR costmap과 DWB는 유지합니다. 처음 비교할 때는 확정 Height Map으로 두 구성을 실행하고, 새로 만든 지도 비교는 별도 실행에서 합니다.

Localization RViz의 고정 frame은 `map`입니다. Player를 재개하고 `/map_to_odom`의 정합 갱신, `terrain READY` 및 lifecycle 활성화를 확인한 뒤 같은 창의 **Nav2 Goal**을 지정합니다.

## 이 bag의 IMU 기록 공백과 확인된 오류

2026-11-23에 sqlite 원본의 IMU header stamp를 확인했습니다. `/livox/imu` 40,597개 사이에 **100 ms 초과 공백 1,149개**, **500 ms 초과 공백 16개**가 있으며 최대는 **1.019894초**입니다. 최대 공백은 bag 시작 후 약 **139.64초**에 끝납니다. LiDAR stamp는 같은 전체 구간에서 약 10 Hz로 이어집니다.

실행 로그에서는 height terrain이 **889.9 ms에 READY**가 되고 Nav2 lifecycle도 활성화됐습니다. 이후 localization의 `Fitness score ... less than localization threshold 0.8`이 0.0까지 떨어졌고, 약 1.3–1.6초 늦은 속도·중력 입력이 폐기되면서 costmap 관측 갱신이 끊겼습니다. RViz `exit code -11`은 Ctrl+C 종료 뒤에 기록됐습니다. 이를 Height Map 로딩 실패로 해석하지 않습니다.

이 공백 안에서도 당시 FAST-LIO `/Odometry`는 약 10Hz로 기록됐고 수신 시각과 header 차이는 대부분 8–11ms(한 샘플 약 23ms)였습니다. 당시 코드가 scan 종료 시각까지의 IMU 수신을 기다리는 구조이므로 센서 전체 중단보다는 recorder 수신·기록 경로 누락 가능성이 큽니다. Recorder 로그에는 이 구간의 정지/누락 원인이 남아 있지 않아 DDS·버퍼·디스크 중 어느 단계인지 확정하지 않았습니다. 이 기록 공백은 재계산 불안정의 의심 원인이며, 단독 원인으로 확정한 것은 아닙니다. RViz를 하나로 줄이거나 `--rate 0.5`로 낮추면 처리 부하는 줄일 수 있지만 누락된 IMU를 복구하지는 않습니다. 앞서 확인한 40초 연결 검증은 이 구간 이전입니다. 현재 bag 전체에 대해 raw 센서 → 새 localization이 정상이라고 간주하지 않습니다.

Global planning만 분리해 검증할 때는 녹화된 `/tf`, `/tf_static`, `/Odometry`, `/fastlio/up_body`, `/fastlio/odometry_velocity`와 LiDAR를 재생하는 방식을 사용하고, 새 localization 및 장착 TF 발행 노드는 함께 실행하지 않아야 합니다. 위 기본 절차는 새 localization을 검사하는 방식이므로 두 방식을 섞지 않습니다.

## 재생 검증에서 볼 것

```bash
ros2 topic echo /clock --once
ros2 topic hz /livox/lidar_pointcloud --window 30
ros2 topic hz /livox/lidar_costmap --window 30
ros2 topic hz /local_costmap/costmap --window 30
ros2 topic echo /legged/replay/cmd_vel
```

- `FASTLIO_DIAG_TIMING`의 `callback_start_age`/`callback_end_age`는 현재 시스템 시각과 녹화 stamp의 차이이므로 bag에서는 수일에 해당하는 큰 값이 나올 수 있습니다. 재생 처리 시간은 `callback_work`, `mapping timing ... total`, planner의 탐색 ms를 확인합니다.
- Hybrid 로그: `Height plan: ... total=... ms`.
- Height-only 로그: `HeightOnly plan: search=... ms`.
- Bag의 로봇 움직임은 녹화된 궤적으로 고정됩니다. 새 `/legged/replay/cmd_vel`에 따라 움직이는 시뮬레이터가 아니므로 경로·비용·속도 출력은 확인할 수 있지만 새 goal의 실제 도달 여부를 검증하는 것은 아닙니다.
- `/clock`이 멈추면 sim-time costmap 갱신도 멈춥니다. 일시정지를 먼저 확인합니다.
- `Fitness score ... less than localization threshold 0.8`이 지속되면 localization 정합이 실패한 것입니다. `/map_to_odom` 갱신과 점군 정렬부터 확인합니다.
- `Rejecting gravity sample`, `Dropping stale...velocity`, `observation buffer has not been updated`가 함께 나오면 FAST-LIO/TF 입력 지연을 확인합니다. 관측 timeout을 늘리거나 height clearance를 낮추는 것으로 해결하지 않습니다. RViz 한 창 구성으로 재시도하고, 지연이 지속되면 전체 재생을 `--rate 0.5`로 다시 시작합니다.
- CPU/디스크 부하로 `Message queue starved`나 입력 지연이 지속되면 `--rate 0.5`로 처음부터 다시 실행합니다. 모든 노드의 sim time/domain/토픽 타입을 같이 확인합니다.

실물 절차는 [OPERATIONS.md](OPERATIONS.md), 설정 파일은 [TUNING.md](TUNING.md), profile 의미는 [DDS.md](DDS.md)를 사용합니다.
