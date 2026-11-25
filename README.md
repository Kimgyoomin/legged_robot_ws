# legged_robot_ws 인수인계

RUBI의 Livox MID-360S와 IMU로 위치를 추정하고, 높이 지도를 만들어 Nav2로 주행하는 ROS 2 Humble 워크스페이스입니다. **새 장비는 설치부터, 이미 설치된 장비는 [터미널 환경](#터미널-환경)부터** 시작하세요.

## 설치

기준 장비는 Jetson Orin NX, Ubuntu 22.04, ROS 2 Humble, Python 3.10입니다. 새 높이 지도를 만들 때는 CUDA 12.6과 CuPy를 사용합니다. 저장된 지도로 주행하는 단계는 CPU에서 실행됩니다. ROS가 없는 장비는 Ubuntu용 Humble Desktop을 먼저 설치하세요. [ROS 공식 설치 안내](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) · [JetPack 버전과 설치 상세](docs/INSTALL.md)

**1. 소스와 의존성.** 새 장비의 Bash 터미널에서 실행합니다. 워크스페이스 폴더를 이미 전달받았다면 `git clone` 줄은 건너뜁니다.

```bash
source /opt/ros/humble/setup.bash
sudo apt update
sudo apt install -y build-essential cmake git pkg-config liblcm-dev liblcm-bin \
  python3-dev python3-pip python3-venv python3-colcon-common-extensions python3-rosdep \
  ros-humble-navigation2 ros-humble-rmw-fastrtps-cpp
git clone --branch jetson/real https://github.com/Kimgyoomin/legged_robot_ws.git ~/legged_robot_ws
cd ~/legged_robot_ws
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro humble -y
python3 -m pip install --user -r requirements-localization.txt
```

**2. Livox SDK2와 ROS 패키지 빌드.** 같은 터미널에서 실행합니다. 드라이버는 워크스페이스의 `.sdk`에서 SDK를 찾습니다. `grid_map_msgs`를 apt로 받을 수 없는 장비는 [소스 설치 절차](docs/INSTALL.md#grid_map_msgs를-소스로-설치할-때)를 적용한 뒤 빌드하세요.

```bash
cmake -S third_party/Livox-SDK2 -B build/sdk -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_SAMPLES=OFF -DCMAKE_INSTALL_PREFIX="$PWD/.sdk"
cmake --build build/sdk -j2 && cmake --install build/sdk
CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2 \
  colcon build --base-paths src --symlink-install --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
source install/setup.bash
ros2 pkg prefix legged_navigation
python3 -c 'import open3d, ros2_numpy, tf_transformations'
```

`ros2 pkg prefix`가 이 워크스페이스의 `install/legged_navigation`을 출력하고 Python import가 성공하면 됩니다.

**3. 높이 지도를 생성할 장비.** 같은 폴더에서 실행하고 마지막 출력의 `gpu_compile_and_inference`가 `PASS`인지 확인합니다.

```bash
python3 -m venv --system-site-packages .venv-height
.venv-height/bin/python -m pip install --only-binary=:all: -r requirements-height.txt
source tools/height_env.sh
ros2 run rubi_height_mapping check_environment
```

## 터미널 환경

**새 Bash 터미널마다** 아래 명령을 실행합니다. 지도 저장용 터미널에도 같은 설정을 적용합니다. `OPS`는 각 패키지가 읽을 센서·지도·토픽 설정 파일입니다. 다른 위치에 설치했다면 `install/setup.bash` 경로와 `LEGGED_WS`를 함께 바꿉니다.

```bash
source /opt/ros/humble/setup.bash
source ~/legged_robot_ws/install/setup.bash
export LEGGED_WS="$HOME/legged_robot_ws"
export OPS="$LEGGED_WS/src/navigation_dds/config/operation_rubi.yaml"
export ROS_DOMAIN_ID=13 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

## 기존 지도로 주행

RUBI 1층에서는 별도 전달받은 **`maps/rubi_1f/` 폴더 전체**를 복원하고 A → B → C → D를 각기 다른 터미널에서 실행합니다. 위치 추정은 `data/*.pcd`, 경로 계획은 `map.rghm`, Hybrid는 추가로 `data/`의 2D YAML과 그 YAML이 가리키는 PGM을 읽습니다. 세 지도는 같은 장소의 `map` 좌표계를 사용해야 합니다. [파일명과 지도 명세](maps/README.md) · [새 지도를 만들 때](#새-지도를-만들-때)

**A · LiDAR.** 센서에 연결된 PC의 유선 IP는 기본 `192.168.1.5`, MID-360S는 `192.168.1.145`입니다. 드라이버가 FAST-LIO용 `/livox/lidar`, Nav2용 `/livox/lidar_pointcloud`, 공통 `/livox/imu`를 발행합니다.

```bash
LIVOX_DIAG_TIMING=1 ros2 launch livox_ros_driver2 msg_MID360s_launch.py operation:=live operation_file:="$OPS"
```

**B · 위치 추정.** 실행 후 RViz의 **2D Pose Estimate**로 로봇의 현재 위치와 방향을 지정합니다. 실시간 점군이 PCD의 벽과 겹치는지 보고, 별도 터미널에서 `ros2 topic hz /map_to_odom`으로 정합 결과가 갱신되는지 확인합니다.

```bash
FASTLIO_DIAG_TIMING=1 ros2 launch fast_lio_localization localization_nav_rubi.launch.py operation:=live operation_file:="$OPS"
```

**C · 내비게이션.** 저장된 높이 지도로 주행할 때 아래 두 명령 중 하나를 선택합니다. 두 모드 모두 가까운 장애물은 실시간 LiDAR와 DWB로 피합니다. 새 높이 지도를 수집할 때는 C에서 먼저 mapper를 실행합니다. [높이 지도 생성과 저장](#새-지도를-만들-때)

```bash
# Hybrid — 전역 경로에 높이 지도, 2D 점유 지도, 실시간 장애물을 반영
ros2 launch legged_navigation height_hybrid_navigation.launch.py operation:=live operation_file:="$OPS"
```

```bash
# Height-only — 전역 경로를 저장된 높이 지도만으로 계산
ros2 launch legged_navigation height_only_navigation.launch.py operation:=live operation_file:="$OPS"
```

**D · RUBI 실물 구동.** 메인 제어기와 로봇 LAN을 연결한 뒤 `"$LEGGED_WS/run_lcm_start.sh"`를 한 번 실행합니다. 기본 인터페이스는 `enP7p1s0`입니다. 다른 포트를 쓰면 먼저 `LCM_IF`를 해당 이름으로 export합니다. 스크립트에서 `multicast route ready`를 확인한 뒤 브리지를 실행하면 Nav2의 `/cmd_vel`이 로봇의 LCM `CMD_VEL`로 전달됩니다.

```bash
ros2 launch pongbot_lcm joy_lcm.launch.py operation:=live operation_file:="$OPS"
```

C 로그에 **terrain READY**가 나오고 `ros2 lifecycle get /controller_server`가 `active`이면 B의 RViz에서 **Nav2 Goal**을 지정합니다. 종료할 때는 goal을 취소하고 속도 0과 실제 정지를 확인한 뒤 D → C → B → A 순서로 Ctrl+C를 누릅니다. RUBI 브리지는 마지막으로 수신한 속도를 반복 송신합니다.

## 새 지도를 만들 때

새 장소에서는 `operation_rubi.yaml`을 복사해 `site`를 새 지도 폴더로, 모든 터미널의 `OPS`를 그 사본 경로로 설정합니다. A의 LiDAR를 먼저 켭니다. 기존 PCD가 있으면 `common.localization.pcd_map`에 그 경로를 지정하고 2번부터 시작합니다.

1. **3D PCD 생성.** 프로필의 `common.slam.pcd_map`과 `common.localization.pcd_map`에 `${SITE}/data/localization.pcd`처럼 같은 새 PCD 경로를 지정합니다. B에서 SLAM을 실행하고 잠시 정지해 IMU를 초기화한 뒤 구역을 이동합니다.

   ```bash
   ros2 launch fast_lio mapping.launch.py operation:=live operation_file:="$OPS"
   ```

   별도 터미널에서 저장하고 `success: true`를 확인한 뒤 B의 SLAM을 종료합니다.

   ```bash
   ros2 run navigation_dds with_profile common \
     ros2 service call /map_save std_srvs/srv/Trigger '{}'
   ```

2. **C · 높이 지도 생성.** A의 LiDAR를 유지하고 B에서 위의 위치 추정을 실행합니다. B의 RViz에서 점군이 PCD에 정합됐는지 확인한 뒤, C에서 GPU 환경을 설정하고 mapper를 실행합니다. 로봇을 이동하며 바닥과 단차를 관측합니다.

   ```bash
   source "$LEGGED_WS/tools/height_env.sh"
   ros2 launch rubi_height_mapping mapping.launch.py operation:=live operation_file:="$OPS"
   ```

3. **E · 높이 지도 저장.** C의 mapper를 켜 둔 채 별도 터미널 E를 엽니다. E에서 [터미널 환경](#터미널-환경)을 적용하고 저장 서비스를 호출합니다.

   ```bash
   ros2 run navigation_dds with_profile common \
     ros2 service call /rubi/global_elevation/save_map std_srvs/srv/Trigger '{}'
   ```

   응답의 `success`가 `true`이면 `message`에 나온 `run_<번호>` 폴더를 기록합니다. 그 안의 **`map.rghm`**이 Nav2 입력이고 `surface.pcd`는 높이 표면 시각화용입니다. 상위 저장 경로는 `common.height_mapping.save_dir`입니다.

4. **C · 저장된 지도로 주행.** A와 B는 계속 실행합니다. 저장 성공 후 C에서 Ctrl+C로 mapper를 종료하고, `$OPS`의 `common.navigation.height_map`에 방금 저장한 `map.rghm` 경로를 지정합니다. Hybrid를 쓸 때는 `common.hybrid.map_yaml`도 같은 좌표계의 2D YAML/PGM에 맞춥니다. C에서 선택한 Nav2를 실행하고 D의 브리지로 실물 주행을 시작합니다. [Nav2 실행 명령](#기존-지도로-주행) · [2D 지도 생성](docs/OPERATIONS.md#5-hybrid를-사용할-때-같은-좌표계의-2d-occupancy-준비)

## 센서·로봇을 바꿀 때

아래의 **운용 프로필**은 `$OPS`입니다. 실행 시 명시한 `키:=값`이 프로필보다 우선하며, 프로필에서는 `live` 또는 `replay`의 값이 `common`을 덮어씁니다. 수정 후 해당 패키지를 다시 실행하세요. [RUBI 운용 프로필](src/navigation_dds/config/operation_rubi.yaml)

| 바뀐 대상 | 수정할 값 | 파일 |
|---|---|---|
| LiDAR IP·설치 방향 | `Mid360s.host_net_info[0].host_ip`, `lidar_configs[0].ip`, `extrinsic_parameter.roll/pitch/yaw`(도). RUBI의 거꾸로 장착한 센서는 roll 180°입니다. 드라이버가 점군과 IMU에 적용한 회전을 장착 TF에 중복 적용하지 않도록 합니다. | [MID360s_config.json](src/livox_ros_driver2/config/MID360s_config.json) |
| 로봇에 센서를 붙인 위치 | `common.localization.imu_to_base_{x,y,z}`(`body → BODY`)와 `base_to_lidar_{x,y,z}`(`base_link → livox_frame`)에 각 부모 좌표계에서 실측한 거리(m)를 넣습니다. | [운용 프로필](src/navigation_dds/config/operation_rubi.yaml) |
| 다른 LiDAR·IMU | 운용 프로필의 `common.slam`·`common.localization`에서 `lidar_topic`, `imu_topic`과 localization의 `input_format`을 맞춥니다. Nav2의 PointCloud2 입력은 `common.navigation.cloud_topic`입니다. 센서 YAML에서 `preprocess.lidar_type`, `preprocess.scan_line`, LiDAR→IMU 내부 보정 `mapping.extrinsic_T/R`을 확인합니다. RUBI localization의 PointCloud2 입력에는 Livox 점별 시각과 필드가 필요합니다. | [SLAM 설정](src/FAST_LIO_ROS2/config/mid360.yaml) · [CustomMsg 설정](src/FAST_LIO_LOCALIZATION2/config/rubi_custommsg.yaml) · [PointCloud2 설정](src/FAST_LIO_LOCALIZATION2/config/mid360s_nav.yaml) |
| 높이 지도의 관측 거리·범위 | 운용 프로필의 `common.height_mapping.min_range/max_range`(m), 로컬 YAML의 `map_length`, 전역 YAML의 `origin_x/y`, `length_x/y`를 바꿉니다. 격자 크기를 바꾸면 두 YAML의 `resolution`과 선택한 planner의 `map_resolution_m`을 맞춰 지도를 새로 생성합니다. | [로컬 높이 지도](src/rubi_height_mapping/config/height_mapping.yaml) · [전역 높이 지도](src/rubi_height_mapping/config/global_heightmap.yaml) |
| 차체 크기·속도·장애물 | 실측 외곽을 `local_costmap`의 `footprint`에 m 단위 꼭짓점으로 입력하고 Hybrid의 `global_costmap`에도 반영합니다. `FollowPath.max_vel_x/max_speed_xy`(m/s), `max_vel_theta`(rad/s), `acc_lim_*`·`decel_lim_*`을 로봇에 맞춥니다. 센서 높이가 바뀌면 `costmap_sector_filter.min_body_height/max_body_height`도 확인합니다. | [Nav2 공통 설정](src/legged_navigation/config/nav2_common.yaml) · [Hybrid 설정](src/legged_navigation/config/height_hybrid.yaml) |
| 넘을 수 있는 단차·여유 거리 | 선택한 planner의 `evaluation.max_crossable_height_jump_m`, `evaluation.local_relief_threshold_m`, `evaluation.grid_terrain_clearance_distance_m`을 로봇의 보행 성능에 맞춥니다. RUBI 값은 각각 0.11, 0.11, 0.20 m입니다. | [Hybrid](src/legged_navigation/config/height_hybrid.yaml) · [Height-only](src/legged_navigation/config/height_only.yaml) · [로봇별 단차 비용](docs/STEP_COST_TUNING.md) |
| 주행 제어기·컴퓨터 | 다른 구동기는 `live.navigation.cmd_vel_topic`의 `geometry_msgs/msg/Twist`를 연결합니다. RUBI LCM은 `live.lcm.cmd_vel_topic/lcm_channel/lcm_url`을 맞춥니다. CPU 코어 수가 다르면 프로필의 각 `cpu_affinity`를 사용 가능한 코어 번호 또는 빈 문자열로 설정합니다. | [운용 프로필](src/navigation_dds/config/operation_rubi.yaml) · [LCM 브리지](src/pongbot_lcm/README.md) |

토픽·TF 확인 명령, 다른 센서의 입력 형식, 파라미터별 단위는 [튜닝 상세](docs/TUNING.md)에 있습니다. 저장된 rosbag은 별도 ROS domain에서 `operation:=replay`로 실행하며 속도 출력은 `/legged/replay/cmd_vel`입니다. [rosbag 재생](docs/HEIGHT_REPLAY.md)

[전체 패키지와 보조 도구](src/README.md) · [DDS 설정](docs/DDS.md) · [검증 기록](docs/DELIVERY_VALIDATION.md) · [지도 도구](tools/README.md) · [구성 보고서](docs/REPORT.md)
