# 실물 운용 순서 — 패키지 직접 실행

장착한 MID-360S로 새 장소를 매핑하는 순서입니다. 새 PCD는 **SLAM으로 만들고**, 저장된 PCD에 위치를 맞추는 단계에서 **localization**을 실행합니다. 두 FAST-LIO 프로세스는 같은 TF·토픽을 사용하므로 동시에 실행하지 않습니다.

확정 지도로 홀 입구 단차를 확인하려면 [11월 2일 복도 재생](HEIGHT_REPLAY.md)을 사용합니다. 별도로 새 FAST-LIO 정합을 실험할 때 실제 LiDAR 대신 `[26.11.19]rubi_pointfoot_nav`를 재생하려면 [bag 전용 명령](HEIGHT_REPLAY.md)을 사용합니다. Driver 대신 player, 모든 노드는 sim time으로 실행합니다.

## 0. 모든 터미널의 환경

```bash
source /opt/ros/humble/setup.bash
source ~/legged_robot_ws/install/setup.bash
export LEGGED_WS="$HOME/legged_robot_ws"
export OPS="$LEGGED_WS/src/navigation_dds/config/operation_rubi.yaml"
export ROS_DOMAIN_ID=13 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

별도 ROS PC도 domain 13과 통신 설정을 맞춥니다. `taskset`은 CPU 사용 가능 범위를 자식 프로세스에 상속하며 코어를 독점 예약하지 않습니다. 운용 YAML의 `cpu_affinity`가 Livox·SLAM·localization·height mapping·navigation 자식 프로세스에 taskset을 적용합니다. 빈 문자열이면 부모 affinity를 상속합니다. 아래 값은 CPU 0–7이 있는 장비의 배치입니다. `lscpu -e=CPU,ONLINE`으로 확인합니다.

| 프로세스 | CPU |
|---|---|
| Livox driver | 0,1 |
| FAST-LIO SLAM **또는** localization | 2,3,4,5 |
| Height mapper **또는** navigation | 6,7 |

`dds_profile`의 의미와 파일 수정은 [DDS.md](DDS.md)에 있습니다. 실행 순서는 아래와 같습니다.



## 실험에 맞는 순서 선택

| 목적 | 실행 순서 | 사용하거나 만드는 지도 |
|---|---|---|
| 확정 지도로 실물 주행 | 아래 A → B에서 초기 정합 → C → D → Nav2 Goal | `maps/rubi_1f/map.rghm`, 같은 장소의 PCD와 2D YAML/PGM |
| 기존 PCD로 새 높이 지도 생성 | A → B에서 초기 정합 → 4번 Height Mapping → 저장 → mapper 종료 → 새 파일로 6A 실행 | `src/rubi_height_mapping/maps/run_<번호>/map.rghm` |
| 처음 방문한 장소 | A → 2번 SLAM 및 PCD 저장 → SLAM 종료 → 새 PCD로 B → 4번 높이 지도 → 5번 2D 지도 → 6A | PCD·높이·2D를 동일한 `map` 좌표계로 준비 |
| 기록된 PointCloud2와 IMU로 검증 | [rosbag의 두 토픽 재생 절차](HEIGHT_REPLAY.md) | 같은 확정 지도, replay 전용 속도 토픽 |

각 launch에 `operation_file:="$OPS"`를 전달합니다. 이 안내의 `OPS`는 [RUBI 운용 프로필](../src/navigation_dds/config/operation_rubi.yaml)입니다. 생략하면 패키지가 별도의 기본 파일 `operation.yaml`을 읽습니다. 두 운용 모드의 launch 인자는 실물 `operation:=live`, 재생 `operation:=replay`입니다. 명시한 CLI 값이 모드별 값과 `common`보다 우선합니다.

| 변경할 값 | 운용 프로필의 키 |
|---|---|
| 현장 폴더 | `site` (기본 `${LEGGED_WS}/maps/rubi_1f`) |
| Nav2가 읽을 높이 지도 | `common.navigation.height_map` (기본 `${SITE}/map.rghm`) |
| 기준 PCD / 2D 지도 | `common.localization.pcd_map` / `common.hybrid.map_yaml`. 2D YAML이 참조하는 PGM도 함께 둡니다. |
| GPU mapper가 쓸 상위 폴더 | 실물 `common.height_mapping.save_dir`, 재생 `replay.height_mapping.save_dir` |

새 장소를 인수인계할 때는 프로필에 지정한 장소 폴더를 통째로 전달합니다. 확정 지도 세트의 파일명과 해시는 [지도 명세](../maps/rubi_1f/site.yaml)에 있습니다.

## 확정 지도에서 실물 실험

모든 ROS 터미널은 위의 실물 domain 13을 사용합니다. 로봇 LAN 케이블을 연결한 뒤 workspace 루트에서 라우팅을 한 번 설정합니다.

```bash
cd ~/legged_robot_ws
./run_lcm_start.sh
```

| 터미널 | 실행 명령 |
|---|---|
| A · LiDAR | `LIVOX_DIAG_TIMING=1 ros2 launch livox_ros_driver2 msg_MID360s_launch.py operation:=live operation_file:="$OPS"` |
| B · 위치 추정과 RViz | `FASTLIO_DIAG_TIMING=1 ros2 launch fast_lio_localization localization_nav_rubi.launch.py operation:=live operation_file:="$OPS"` |
| C · 높이 + 2D Nav2 | `ros2 launch legged_navigation height_hybrid_navigation.launch.py operation:=live operation_file:="$OPS"` |
| D · 로봇 LCM | `ros2 launch pongbot_lcm joy_lcm.launch.py operation:=live operation_file:="$OPS"` |

A의 센서 IP는 `192.168.1.145`, host IP는 `192.168.1.5`입니다. B의 RViz에서 **2D Pose Estimate**로 초기 정합 후 점군과 지도를 확인합니다. C가 `terrain READY` 및 lifecycle active 상태가 되고 D와 메인 제어기가 준비되면 같은 창에서 **Nav2 Goal**을 보냅니다. CPU affinity, DDS, clock, RViz, 지도 및 토픽은 [운용 YAML](../src/navigation_dds/config/operation_rubi.yaml)에 있습니다.

D는 ROS `/cmd_vel`과 `/joy_wireless`를 구독해 LCM `CMD_VEL`로 전송합니다. route 스크립트는 Main PC용 NIC에 multicast 경로만 설정합니다. 브리지의 원본 동작상 `/cmd_vel`이 끊겨도 마지막 속도를 계속 전송합니다. 실험 종료 때는 goal을 취소하고 속도 0 및 로봇 정지를 확인한 뒤 브리지를 종료합니다.

## 실물 주행을 발표용 rosbag으로 녹화하기

발표에서 실험 당시의 높이 지도와 주행 경로를 RViz 한 창으로 보여주려면 센서와 함께 **지도·TF·localization·Nav2 출력**을 녹화합니다. 위의 실물 환경을 적용한 별도 Bash 터미널에서 실행합니다. Livox를 켠 뒤, localization과 navigation을 띄우기 전에 녹화를 시작하면 초기 정합과 첫 경로도 남습니다.

```bash
CFG="$HOME/legged_robot_ws/src/navigation_dds/config"
mkdir -p "$HOME/rosbag_record"
BAG="$HOME/rosbag_record/rubi_demo_$(date +%Y%m%d_%H%M%S)"
mapfile -t TOPICS < "$CFG/presentation_record_topics.txt"
ros2 run navigation_dds with_profile common \
  ros2 bag record -o "$BAG" --storage sqlite3 --max-cache-size 268435456 \
  --compression-mode none --include-hidden-topics \
  --qos-profile-overrides-path "$CFG/presentation_map_qos.yaml" "${TOPICS[@]}"
```

| 확인할 대상 | 녹화되는 내용 |
|---|---|
| 확정 2.5D 높이 지도 | Navigation이 `map.rghm`에서 읽은 표면을 `/legged/height`에 PointCloud2로 한 번 발행합니다. 기록 1건이면 전체 표면이 들어 있습니다. |
| 새 높이 지도를 만드는 실험 | Mapper 실행 중의 전역 점군은 `/rubi/global_elevation/cloud`, 로컬 그리드는 `/rubi/local_elevation/grid_map`입니다. 둘 다 녹화 목록에 있습니다. |
| Localization 및 주행 결과 | `/tf`, `/tf_static`, `/cloud_pcd`, `/cur_scan_in_map`, `/submap`, `/localization`, `/Odometry`, `/map`, `/plan`, `/local_plan`, 양쪽 costmap을 기록합니다. |
| 사후 분석 | 원본 Livox 세 토픽, 필터 점군, 속도 명령, 실물 조이스틱 `/joy_wireless`, action feedback/status 및 진단 로그도 포함합니다. DWB의 `publish_trajectories`, `publish_evaluation`이 꺼진 실험에서는 해당 디버그 출력이 비어 있을 수 있습니다. |

[전체 녹화 목록](../src/navigation_dds/config/presentation_record_topics.txt)은 한 줄에 토픽 하나입니다. [지도 QoS 파일](../src/navigation_dds/config/presentation_map_qos.yaml)은 `/legged/height`, `/map`, `/tf_static`에 `reliable + transient_local`을 지정해 recorder가 나중에 구독해도 실행 중인 발행자가 보관한 메시지를 받게 합니다. 센서 QoS는 recorder가 발행자에 맞춥니다. 실물은 시스템 시각을 사용하므로 `/clock` 기록이 0건이어도 됩니다.

실험 후 Ctrl+C로 녹화를 끝내고 다음 명령에서 **높이 지도·정적 TF·2D 지도·경로의 Count**를 확인합니다. 로봇이 DWB로 경로를 추종한 구간이면 `/local_plan`도 확인합니다.

```bash
ros2 bag info "$BAG" | grep -E 'Topic: /(legged/height|tf_static|cloud_pcd|map|plan|local_plan) [|]'
```

발표 때는 [RViz 한 창으로 녹화 결과 재생](HEIGHT_REPLAY.md)을 따릅니다. 카메라 시점과 표시 항목을 바꿨다면 실험 중 RViz의 **File → Save Config As**로 `.rviz` 파일을 bag 폴더와 함께 보관합니다.

## 센서 재계산과 위치 대조용으로 녹화하기

실물 환경(domain 13)을 녹화 터미널에도 적용합니다. Livox를 켜고 recorder를 시작하면 초기 센서 상태부터 남길 수 있습니다. 로봇을 정지시킨 상태에서 `ros2 topic hz /livox/imu --window 200`과 `ros2 topic delay /livox/imu --window 200`을 확인합니다. 이 장비의 정상 구간에서는 약 200 Hz였고 FAST-LIO에서 잰 평균 IMU 지연은 수 ms 이내였습니다. 약 1초 이상 지난 샘플이 계속 들어오면 기다린 시간만으로 준비됐다고 판단하기 어렵습니다. 수신이 안정되면 localization을 시작하고, RViz에서 초기 정합을 확인한 뒤 navigation을 실행합니다.

```bash
# 실물 A~D와 별도 터미널에서 실행
source /opt/ros/humble/setup.bash
source ~/legged_robot_ws/install/setup.bash
export ROS_DOMAIN_ID=13 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp

mkdir -p "$HOME/rosbag_record"
BAG="$HOME/rosbag_record/rubi_nav_$(date +%Y%m%d_%H%M%S)"
ros2 run navigation_dds with_profile common \
  ros2 bag record -o "$BAG" \
  /livox/lidar /livox/lidar_pointcloud /livox/imu \
  /tf /tf_static /Odometry /localization /fastlio/up_body /fastlio/odometry_velocity \
  /initialpose /plan /cmd_vel /rosout
```

| 녹화 대상 | 이후 검증에 쓰는 방법 |
|---|---|
| `/livox/lidar`, `/livox/lidar_pointcloud`, `/livox/imu` | 센서만 기록할 때는 이 세 개로 끝내도 됩니다. 실물과 같은 CustomMsg 경로 및 PointCloud2+IMU 두 토픽 경로를 모두 재현할 수 있습니다. 두 토픽 모드만 검증할 때는 PointCloud2와 IMU만 기록해도 됩니다. |
| `/tf`, `/tf_static`, `/Odometry`, `/localization`, `/fastlio/up_body`, `/fastlio/odometry_velocity` | 기록 당시의 위치·자세와 속도로 planner를 다시 비교합니다. |
| `/initialpose`, `/plan`, `/cmd_vel`, `/rosout` | 초기 위치 입력, 성공한 경로, 명령 속도, Nav2의 목표 좌표 및 실패 로그를 대조합니다. 센서로 재계산할 때는 player의 `--topics`로 센서만 선택합니다. |

`with_profile common`은 recorder 프로세스에 32 MiB SHM과 UDP를 구성합니다. 녹화 구독의 reliability는 Humble rosbag2가 발견한 발행자에 맞춰 선택합니다. 이 드라이버의 보조 PointCloud2는 `best_effort`이고 CustomMsg와 IMU는 `reliable`입니다. 두 토픽 **재생**에 쓰는 `replay_livox_qos.yaml`은 player 전용입니다. 그 파일을 recorder에 지정하면 PointCloud2 구독이 `reliable`을 요구해서 실물 점군을 받지 못합니다. [ROS 2 Humble rosbag2 QoS 구현](https://github.com/ros2/rosbag2/blob/humble/rosbag2_transport/src/rosbag2_transport/qos.cpp)

recorder에 센서 세 토픽의 `Subscribed to topic`이 뜬 뒤 실험을 진행합니다. Nav2를 나중에 시작해도 recorder의 기본 discovery가 요청한 토픽을 발견합니다. 실험 후 녹화 터미널에서 Ctrl+C로 종료한 뒤 같은 터미널에서 파일과 토픽별 개수를 확인합니다.

```bash
ros2 bag info "$BAG"
```

IMU 결측을 볼 때는 총 메시지 수와 함께 연속 header stamp의 간격도 확인합니다. 다음 명령은 Humble 기본 SQLite3 bag에서 분할 파일을 녹화 순서대로 읽어, 최대 간격과 100 ms를 넘은 횟수를 출력합니다. 정상 200 Hz의 기준 간격은 약 5 ms입니다. 이 수치는 bag에서 관측한 공백으로, 센서·전송·recorder 중 어느 단계에서 발생했는지는 실행 로그와 대조합니다.

```bash
/usr/bin/python3 - "$BAG" <<'PYBAG'
import sqlite3, sys, yaml
from pathlib import Path
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu

bag = Path(sys.argv[1]).resolve()
meta = yaml.safe_load((bag / 'metadata.yaml').read_text())['rosbag2_bagfile_information']
previous = None
count = gaps = backwards = max_ns = 0
for name in meta['relative_file_paths']:
    conn = sqlite3.connect((bag / name).as_uri() + '?mode=ro', uri=True)
    rows = conn.execute("SELECT m.data FROM messages m JOIN topics t ON t.id=m.topic_id "
                        "WHERE t.name='/livox/imu' ORDER BY m.timestamp, m.id")
    for (payload,) in rows:
        stamp = deserialize_message(payload, Imu).header.stamp
        ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        if previous is not None:
            delta = ns - previous
            max_ns = max(max_ns, delta)
            gaps += delta > 100_000_000
            backwards += delta < 0
        previous = ns
        count += 1
    conn.close()
if count < 2:
    sys.exit(f'IMU {count}개: 간격을 계산하려면 2개 이상 필요합니다.')
print(f'IMU {count}개, 최대 간격 {max_ns/1e6:.1f} ms, 100 ms 초과 {gaps}회, 역행 {backwards}회')
PYBAG
```

## 1. Livox driver — 터미널 A

```bash
LIVOX_DIAG_TIMING=1 ros2 launch livox_ros_driver2 msg_MID360s_launch.py operation:=live operation_file:="$OPS"
```

RViz를 띄우지 않습니다. 확인용 터미널에서:

```bash
ros2 topic type /livox/lidar
ros2 topic type /livox/lidar_pointcloud
ros2 topic type /livox/imu
```

| 발행 토픽 | 형식 | 구독자와 용도 |
|---|---|---|
| `/livox/lidar` | `livox_ros_driver2/msg/CustomMsg` | 실물 FAST-LIO SLAM·localization이 스캔 기준 시각과 점별 `offset_time`으로 움직임을 보정합니다. 설정상 10 Hz입니다. |
| `/livox/lidar_pointcloud` | `sensor_msgs/msg/PointCloud2` | 같은 스캔 중 매 두 번째 것을 표준 점군으로 발행합니다(10 Hz 설정에서 약 5 Hz). Nav2가 로봇 주변 장애물을 보고, 두 토픽 replay에서는 FAST-LIO도 이 점군을 공유합니다. |
| `/livox/imu` | `sensor_msgs/msg/Imu` | FAST-LIO가 각속도·가속도로 스캔 사이의 자세와 움직임을 추정합니다. 센서의 IMU 패킷마다 발행합니다. |

Nav2의 `/livox/lidar_costmap`은 sector filter가 PointCloud2를 거리·중력 방향 기준으로 거른 출력입니다. Height Mapping은 localization의 `/cloud_registered_body`를 받습니다. 장착을 정방향으로 바꿀 때 점검할 JSON·TF·내부 보정은 [튜닝 안내](TUNING.md)에 있습니다.

현재 이 workspace의 `rviz_MID360_launch.py`/`rviz_MID360s_launch.py`는 별도 PointCloud2 전용 구성입니다. 이전 ros2_ws에서 사용한 동명의 수정 launch와 다르므로, 이 안내에서는 이미 이중 발행과 `dds_profile`을 지원하는 `msg_MID360s_launch.py`를 사용합니다. `LIVOX_DIAG_TIMING=1`은 driver 진단, `FASTLIO_DIAG_TIMING=1`은 localization 진단 로그를 켭니다.

## 2. 처음 한 번: SLAM으로 localization PCD 생성 — 터미널 B

이미 같은 장소·좌표계의 PCD가 있으면 3번으로 갑니다.

```bash
ros2 launch fast_lio mapping.launch.py operation:=live operation_file:="$OPS"
```

처음 잠시 정지하여 IMU가 초기화된 뒤 원하는 구역을 이동하며 수집합니다. 지도와 `/cloud_registered`가 정상인지 확인하고 저장합니다:

```bash
ros2 run navigation_dds with_profile common \
  ros2 service call /map_save std_srvs/srv/Trigger '{}'
ls -lh "$LEGGED_WS/src/FAST_LIO_LOCALIZATION2/maps/site_a.pcd"
```

응답의 `success: true`와 경로를 확인합니다. 빈 누적 지도, 저장 불가 경로, 이미 존재하는 파일은 실패를 반환합니다. 기존 파일을 갱신하려는 경우에만 시작할 때 `pcd_overwrite:=true`를 추가합니다. 기본값은 false입니다. SLAM 종료 시에도 같은 경로로 저장을 시도하므로 이미 서비스로 저장한 파일은 기본값에서 덮어쓰지 않습니다. **최종 위치에서 한 번 저장하고 SLAM을 Ctrl+C로 종료**합니다.

SLAM은 `camera_init` 기준 PCD를 기록하고, localization은 이 PCD 좌표를 `map` 기준으로 사용합니다. Height Mapping은 아래 localization이 정합한 `map` 좌표에서 새로 수행합니다.

## 3. 저장 PCD로 localization — 터미널 B

Driver를 계속 실행합니다. 새로 PCD를 저장했다면 B를 시작하기 전에 현장 프로필의 `common.localization.pcd_map`을 방금 저장한 `common.slam.pcd_map`과 같은 파일로 설정합니다. 기존 RUBI 현장은 확정 PCD를 그대로 사용합니다. SLAM을 종료한 뒤 B를 실행합니다.

```bash
FASTLIO_DIAG_TIMING=1 ros2 launch fast_lio_localization localization_nav_rubi.launch.py operation:=live operation_file:="$OPS"
```

**RViz는 localization에서 띄운 한 창을 계속 사용합니다.** 기본 구성 `src/FAST_LIO_LOCALIZATION2/rviz/fastlio_localization.rviz`에 Nav2 패널·**Nav2 Goal**, `/plan`, local/global costmap, frozen `/legged/height`를 포함합니다. Navigation의 `rviz` 기본값은 false이며 이 프로필도 같은 값을 사용합니다. 별도 창이 필요할 때만 navigation에 `rviz:=true`를 지정합니다.

Height mapping 중에는 Displays에서 **Live global height (mapping only)**를 켜고, 저장 후 navigation으로 전환하면 끕니다. Height-only에서는 **Occupancy map (hybrid)**와 **Global costmap (hybrid)**를 끕니다. 이 체크는 시각화에만 영향을 줍니다.

이미 열린 localization RViz는 **File → Open Config**로 `~/legged_robot_ws/src/FAST_LIO_LOCALIZATION2/rviz/fastlio_localization.rviz`를 불러오면 됩니다. Localization 프로세스를 재시작할 필요는 없습니다.

RViz의 **2D Pose Estimate**로 초기 위치·방향을 지정하고 실시간 점군이 PCD 벽·기둥에 맞는지 확인합니다. 고정 Frame은 `map`입니다. `map → camera_init → body → BODY → base_link` TF와 `/cloud_registered_body`가 준비된 다음 height mapping을 시작합니다.

```bash
ros2 run tf2_ros tf2_echo map base_link
ros2 topic info /cloud_registered_body
ros2 topic hz /map_to_odom --window 10
```

이 launch가 장착 TF도 발행합니다. 측정값을 바꿔야 한다면 [튜닝 표](TUNING.md)의 mount 인자를 확인합니다. Localization이 불안정한 상태에서 얻은 높이는 잘못된 좌표에 누적될 수 있으므로 정합부터 맞춥니다.

## 4. Height Mapping — 터미널 C

A의 LiDAR와 B의 위치 추정을 켜 두고 초기 정합을 확인합니다. C에서 0번의 공통 설정을 적용한 뒤 CUDA 환경과 mapper를 불러옵니다.

```bash
source "$LEGGED_WS/tools/height_env.sh"
ros2 launch rubi_height_mapping mapping.launch.py operation:=live operation_file:="$OPS"
```

로봇을 이동시켜 사용할 바닥·문 통과 구역·단차를 관측합니다. local mapper는 CUDA로 높이를 추정하고 global wrapper가 `map`에 누적합니다. 스캔 거리 기본값은 0.6–3.0 m입니다. 필요 시 `min_range`, `max_range`와 대응 YAML 값을 함께 확인합니다.

**터미널 E · 높이 지도 저장.** C의 mapper는 계속 실행합니다. 별도 터미널 E를 열어 0번의 공통 설정을 적용한 뒤 다음 명령을 실행합니다.

```bash
ros2 run navigation_dds with_profile common \
  ros2 service call /rubi/global_elevation/save_map std_srvs/srv/Trigger '{}'
```

응답의 `success`가 `true`이면 `message`에 나온 실제 `.../run_<번호>` 폴더를 기록합니다. 이 폴더에 두 파일이 생성됩니다.

- `map.rghm`: 높이, 분산, 관측·사용 가능 상태, 좌표계·해상도를 보존한 **navigation 입력**.
- `surface.pcd`: 사용 가능한 높이 표면을 시각화하는 점군.

예를 들어 저장 응답이 `…/src/rubi_height_mapping/maps/run_123`이면, 다음 실행에 사용할 프로필의 `common.navigation.height_map` 값을 `${LEGGED_WS}/src/rubi_height_mapping/maps/run_123/map.rghm`으로 지정합니다. 실제 응답의 번호를 사용하세요. 확정 파일과 실험 파일을 보존하려면 현장 프로필을 복사해 선택 경로만 바꿉니다. 새 장소에서는 `common.localization.pcd_map`과 `common.hybrid.map_yaml`도 그 장소의 파일로 맞춥니다.

저장 성공 후 **터미널 C의 mapper를 Ctrl+C로 종료**합니다. Driver와 localization은 계속 실행합니다. Height-only는 C에서 6B를 실행하고, Hybrid는 같은 장소의 2D 지도를 준비한 뒤 C에서 6A를 실행합니다.

## 5. Hybrid를 사용할 때: 같은 좌표계의 2D Occupancy 준비

Height-only는 이 단계가 필요 없습니다. 기존의 정합된 YAML/PGM이 있으면 그대로 지정합니다. 새 장소라면 다음 도구로 초안을 생성할 수 있습니다. `run_실제번호`를 저장 응답 값으로 바꿉니다.

```bash
python3 "$LEGGED_WS/tools/pcd_to_occupancy.py" \
  --pcd "$LEGGED_WS/src/FAST_LIO_LOCALIZATION2/maps/site_a.pcd" \
  --height "$LEGGED_WS/src/rubi_height_mapping/maps/run_실제번호/map.rghm" \
  --output "$LEGGED_WS/src/rubi_height_mapping/maps/site_a_2d" \
  --min-obstacle-height 0.15 --max-obstacle-height 1.0
```

높이 표면이 관측된 셀만 free로 두고, PCD에서 그 표면보다 0.15–1.0 m 높은 점이 있으면 occupied로 만듭니다. 지면 미관측 영역은 unknown으로 남깁니다. 인접 지면 참조 범위는 `--ground-support-radius`(기본 0.20 m)이며 inflation을 생성하는 인자가 아닙니다.

출력은 `map.yaml`, `map.pgm`, `projection.json`입니다. 도구는 좌표 정합을 수행하지 않으므로 PCD·Height·2D 맵을 RViz에서 겹쳐 문·벽·통로를 확인합니다. 문 위치가 바뀌었거나 여러 층이 겹친 환경은 단일 표면 투영으로 표현할 수 있는지 검토해야 합니다. 초안은 필요에 따라 수정하고 실제 사용 버전을 따로 보관합니다.

## 6A. 기존 방식: Height + 2D 지도 navigation — 터미널 C

0번 환경에서 운용 YAML의 Height Map과 2D 지도 경로를 확인합니다.

```bash
ros2 launch legged_navigation height_hybrid_navigation.launch.py operation:=live operation_file:="$OPS"
```

기존 `HeightPlanner`를 사용합니다. Static Occupancy, 실시간 LiDAR global 장애물/inflation, 추가 장애물 clearance 0.30 m가 반영됩니다.

## 6B. 별도 구성: Height-only navigation — 터미널 C

6A를 실행 중이면 먼저 종료합니다:

```bash
ros2 launch legged_navigation height_only_navigation.launch.py operation:=live operation_file:="$OPS"
```

별도 `HeightOnlyPlanner`입니다. `map_yaml` 인자가 없고 `/map`도 요구하지 않습니다. Global planner는 외부 2D costmap을 읽지 않으며 **경로 길이 + 높이 비용**으로 탐색합니다. 미관측 셀·큰 단차는 통과하지 않습니다. Height 기반 위험 지형 clearance 0.20 m는 유지합니다.

실제 주행에는 두 구성 모두 local LiDAR obstacle/inflation + DWB를 사용합니다. Height-only에서 이동 장애물을 global로 우회하는 기능은 없습니다. local 주행이 막히면 정지·재시도할 수 있습니다. 로봇의 기존 `/cmd_vel` 수신 bridge가 연결되어 있어야 실제 구동됩니다. 이 launch는 별도 로봇 하드웨어 bridge를 시작하지 않습니다.

## 7. Goal·상태·종료

- 로그의 `Frozen terrain READY` 또는 `HeightOnly terrain READY`, lifecycle 활성화를 확인한 뒤 RViz **Nav2 Goal**을 지정합니다. 처음 지도를 읽을 때의 cache 준비 시간과 goal당 탐색 시간은 다릅니다.
- 탐색 로그: hybrid는 `Height plan: ... total=... ms`, height-only는 `HeightOnly plan: search=... ms`. 현재 기본 budget은 800 ms이며 목표 탐색/재계획 주기는 1 Hz입니다. 성능 보장은 아닙니다.
- `/plan`, `/local_costmap/costmap`, `/cmd_vel`, `/fastlio/odometry_base`를 확인합니다. Height-only에서는 같은 RViz의 hybrid 전용 map 표시 두 개를 끕니다.
- `use_fastlio_velocity:=false`는 기존 `/Odometry` 속도 입력과 StandardTrajectoryGenerator를 사용합니다. 기본 true는 `/fastlio/odometry_velocity`를 base_link 기준으로 변환하여 DWB에 넣습니다.
- 위 절차는 localization의 RViz 한 창을 사용합니다. GUI가 전혀 필요 없으면 localization도 `use_rviz:=false`로 실행합니다. msg driver는 원래 RViz가 없습니다.
- 종료 순서: goal 취소 및 정지 확인 → LCM 브리지 → navigation → localization → driver. 설정 파일을 고친 뒤 해당 프로세스를 재시작합니다. Planner 높이/clearance 파라미터는 실행 중 `ros2 param set`으로 바뀌지 않도록 되어 있습니다.

문제별 확인 파일은 [TUNING.md](TUNING.md), 통신은 [DDS.md](DDS.md)를 참고합니다.
