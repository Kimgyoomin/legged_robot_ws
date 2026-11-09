# Installation

[README](../README.md)의 실물 Livox / rosbag 실행에 필요한 설치 절차입니다. 기준 환경은 **Ubuntu 22.04, ROS 2 Humble, Python 3.10**입니다. 높이 지도 생성에는 CUDA/CuPy가 추가로 필요하고, 저장한 높이 지도를 이용하는 navigation은 CPU에서 실행됩니다.

ROS 2 Humble 자체는 먼저 설치되어 있어야 합니다. 기존 Jetson의 JetPack·CUDA·시스템 Python은 유지합니다. 아래 절차에는 `apt upgrade`나 시스템 CUDA 교체가 포함되지 않습니다.

## Base dependencies

새 터미널에서 필요한 도구와 라이브러리를 설치합니다. 이미 준비된 장비에서는 해당 설치를 생략합니다.

```bash
source /opt/ros/humble/setup.bash
sudo apt update
sudo apt install \
  build-essential cmake git \
  python3-dev python3-pip python3-venv \
  python3-colcon-common-extensions python3-rosdep \
  python3-numpy python3-scipy python3-matplotlib python3-yaml python3-open3d \
  libeigen3-dev libpcl-dev libyaml-cpp-dev \
  ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-nav2-collision-monitor \
  ros-humble-rviz2 ros-humble-rmw-fastrtps-cpp \
  ros-humble-pcl-ros ros-humble-pcl-conversions \
  ros-humble-tf-transformations ros-humble-sensor-msgs-py
```

현재 `global_localization.py`는 점군을 읽을 때 **`ros2_numpy`를 import**합니다. ROS manifest만으로 설치되지 않는 의존성이므로 별도로 준비합니다. 아래 버전은 NumPy 1.24.2 의존성을 포함하며 현재 Python 3.10 구성에 맞춘 것입니다.

```bash
python3 -m pip install --user 'ros2-numpy==0.0.5'
```

`grid_map_msgs`는 전역 높이 누적 wrapper의 빌드에 필요합니다. 바이너리 패키지를 사용할 수 있으면 다음을 실행합니다.

```bash
sudo apt install ros-humble-grid-map-msgs
```

해당 바이너리를 제공하지 않는 환경에서는 아래 [로컬 소스 빌드](#grid-map-source-build)를 사용합니다. 두 방법을 중복 설치할 필요는 없습니다.

소스를 받은 뒤 나머지 ROS 의존성을 확인합니다.

```bash
cd ~/legged_robot_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
```

`rosdep`를 처음 설치한 장비에서 초기화가 필요하다고 안내하면 **한 번만** `sudo rosdep init`을 실행하고 `rosdep update`부터 다시 진행합니다. `grid_map_msgs`를 아래 로컬 underlay로 준비했다면, 해당 환경을 source하고 마지막 설치 명령에 `--skip-keys "grid_map_msgs grid_map_cmake_helpers"`를 추가합니다.

## Livox SDK2

SDK 소스는 `third_party/Livox-SDK2`에 포함되어 있습니다. 드라이버는 workspace 내부 `.sdk`를 참조하므로 ROS 패키지보다 먼저 빌드합니다.

```bash
cd ~/legged_robot_ws
cmake -S third_party/Livox-SDK2 -B build/sdk \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_SAMPLES=OFF \
  -DCMAKE_INSTALL_PREFIX="$PWD/.sdk"
cmake --build build/sdk -j2
cmake --install build/sdk
```

시스템 `/usr/local`의 다른 Livox SDK를 변경하지 않습니다. 다른 장치로 소스를 옮겼으면 그 장치에서 SDK와 workspace를 다시 빌드합니다.

## Workspace build

```bash
cd ~/legged_robot_ws
source /opt/ros/humble/setup.bash
if [ -f .deps_install/local_setup.bash ]; then
  source .deps_install/local_setup.bash
fi

CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2 \
  colcon build --base-paths src --symlink-install --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

source tools/navigation_env.sh
ros2 pkg prefix livox_ros_driver2
ros2 pkg prefix fast_lio_localization
ros2 pkg prefix legged_bringup
python3 -c 'import open3d, ros2_numpy, tf_transformations; print("Localization Python imports OK")'
```

패키지 prefix가 이 workspace의 `install/` 아래인지 확인합니다. `--base-paths src`는 활성 패키지만 선택하며 `archive/`의 대안 소스와 과거 실험 자료는 빌드하지 않습니다. `-j2`와 sequential executor는 빌드 메모리 사용을 줄이기 위한 값입니다.

실행 환경은 매 터미널에서 `source ~/legged_robot_ws/tools/navigation_env.sh`로 불러옵니다. 이미 다른 workspace를 source한 터미널 대신 새 터미널을 사용합니다.

## Grid map source build

`ros-humble-grid-map-msgs` 바이너리를 사용할 수 없을 때만 필요한 대안입니다. 기존 `.deps/grid_map`이 있으면 clone을 생략하고 사용할 소스 버전을 확인합니다.

```bash
cd ~/legged_robot_ws
mkdir -p .deps
git clone --branch humble https://github.com/ANYbotics/grid_map.git .deps/grid_map
git -C .deps/grid_map checkout --detach d9dbb58fb3e070de000775cefcfd8951515abee8
touch .deps/COLCON_IGNORE

source /opt/ros/humble/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2 \
  colcon --log-base .deps_log build \
  --base-paths .deps/grid_map/grid_map_cmake_helpers .deps/grid_map/grid_map_msgs \
  --build-base .deps_build \
  --install-base .deps_install \
  --packages-select grid_map_cmake_helpers grid_map_msgs \
  --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

source .deps_install/local_setup.bash
ros2 interface show grid_map_msgs/msg/GridMap
```

이후 [Workspace build](#workspace-build)를 진행합니다. `navigation_env.sh`와 `height_env.sh`는 `.deps_install`이 있으면 자동으로 불러옵니다.

## Optional: height mapping with CuPy

높이 지도를 **새로 생성할 장치**에만 설치합니다. 현재 Jetson 구성은 Python 3.10, CUDA 12.6이며 다음 예시는 CUDA 12.x용입니다. 다른 CUDA 버전에서는 호환 wheel을 먼저 확인합니다.

```bash
nvcc --version
python3 --version

cd ~/legged_robot_ws
python3 -m venv --system-site-packages .venv-height
.venv-height/bin/python -m pip install --only-binary=:all: \
  'cupy-cuda12x==13.6.0' 'numpy>=1.22,<2'
touch .venv-height/COLCON_IGNORE

source tools/height_env.sh
ros2 run rubi_height_mapping check_environment
```

`height_env.sh`는 시스템 Python shebang을 사용하는 ROS 실행 파일에서도 workspace의 CuPy를 찾도록 `PYTHONPATH`를 설정합니다. venv 활성화만으로 대체하지 않습니다. GPU mapper에 PyTorch는 필요하지 않습니다.

DDS 환경을 유지하면서 height mapper를 사용할 때는 순서대로 불러옵니다.

```bash
source ~/legged_robot_ws/tools/navigation_env.sh
source ~/legged_robot_ws/tools/height_env.sh
export ROS_DOMAIN_ID=13
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0
```

수동 height 편집 GUI를 사용할 때 Tk가 없으면 `sudo apt install python3-tk`로 준비합니다. 지도 생성·저장은 [OPERATIONS.md](OPERATIONS.md)를 참고하세요.

## Sensor configuration

README의 실물 명령은 다음 파일을 사용합니다.

- Driver launch: `src/livox_ros_driver2/launch_ROS2/rviz_MID360s_launch.py`
- 센서 설정: `src/livox_ros_driver2/config/MID360s_config.json`
- Localization PointCloud2 설정: `src/FAST_LIO_LOCALIZATION2/config/mid360s_nav.yaml`

현재 IP는 센서 `192.168.1.145`, 호스트 `192.168.1.5`이며 `roll: 180`, `pitch: 0`, `yaw: 0` 장착 회전을 사용합니다. 실제 센서·포트·장착 방향에 맞춰 수정합니다. 회전된 LiDAR/IMU에 추가 axis-flip을 적용하지 않습니다.

이 launch의 `/livox/lidar`는 PointCloud2입니다. 점별 `timestamp`가 없는 임의의 XYZ-only 점군은 FAST-LIO의 MID-360 PointCloud2 파서 입력과 같지 않습니다. `/livox/lidar`가 CustomMsg인 기존 bag/driver 구성에는 `input_format:=custommsg`를 사용합니다.

`msg_MID360s_launch.py`는 현재 `xfer_format=1`인 별도 CustomMsg 구성입니다. RViz만 제거하려고 이름을 바꿔 실행하면 센서 메시지 타입도 달라집니다. 두 파일을 동일한 출력 형식으로 가정하지 않습니다.

## Maps

실제 지도와 bag 데이터는 Git clone에 포함되지 않습니다. 기본 site를 사용하려면 [maps/README.md](../maps/README.md)에 명시된 PCD·occupancy·height 파일을 복원해야 합니다. 새로운 장소는 별도 `site.yaml`로 관리하며 localization과 navigation에 같은 site를 전달합니다.

이전 `navigation_real`의 PCD/occupancy 파일이 있다면 다음 도구로 해당 기준 지도를 가져올 수 있습니다. 높이 지도는 별도로 준비해야 합니다.

```bash
cd ~/legged_robot_ws
python3 tools/import_legacy_rubi_maps.py \
  --source-root "$HOME/ros2_ws/src/navigation_real"
```

도구는 기록된 원본 해시를 검사합니다. 단순히 `alignment.verified`만 변경해서 다른 좌표계의 지도를 연결하지 않습니다. 지도 교체는 [HEIGHT_NAVIGATION.md](HEIGHT_NAVIGATION.md)에 설명되어 있습니다.

## DDS and socket buffers

`navigation_env.sh` 및 `navigation_dds with_profile common`은 Fast DDS의 32 MiB SHM + UDP 프로필을 사용합니다. Localization launch는 큰 PCD를 발행하는 프로세스에 128 MiB 프로필을 별도로 적용합니다. 크기는 participant별 설정이며 시스템 swap이나 `/dev/shm` 전체 크기를 바꾸는 명령이 아닙니다.

프로필의 UDP 송수신 버퍼 요청은 각각 4 MiB입니다. 커널 소켓 버퍼 상한을 올릴 필요가 있으면 다음을 적용합니다.

```bash
sudo sysctl -w net.core.rmem_max=16777216 net.core.wmem_max=16777216
sysctl net.core.rmem_max net.core.wmem_max
```

이 설정은 **장치 전체에 적용되며 재부팅하면 초기화**됩니다. 터미널마다 반복할 필요는 없습니다. 16 MiB는 소켓 버퍼의 상한이며 모든 소켓에 그만큼을 자동 할당하는 것은 아닙니다. 기존 프로세스의 버퍼가 자동으로 커진다고 가정하지 말고 실행 전에 적용합니다.

## Troubleshooting

| 증상 | 확인할 항목 |
|---|---|
| SDK library를 찾지 못함 | `.sdk`를 먼저 빌드했는지, 이동한 workspace를 다시 빌드했는지 |
| `No module named ros2_numpy` | `ros2-numpy==0.0.5` 설치와 ROS 실행 Python의 import 경로 |
| `No module named cupy` | 선택 CuPy 환경 설치 후 `source tools/height_env.sh` |
| 다른 workspace의 package가 실행됨 | 새 터미널에서 legged 환경만 source, `ros2 pkg prefix` 확인 |
| site/asset/hash 오류 | 지도 데이터 복원, site 상대경로·SHA-256·frame 확인 |
| 센서는 보이지만 localization이 안 됨 | CustomMsg/PointCloud2와 `input_format` 일치, IMU, 초기 위치 확인 |
| bag에서 ROS Time=0·costmap 정지 | player의 `--clock`, 일시정지 상태, 모든 노드의 sim time/domain 확인 |

실행 명령은 [README](../README.md#live-livox)와 [재생 안내](HEIGHT_REPLAY.md)를 기준으로 사용합니다.
