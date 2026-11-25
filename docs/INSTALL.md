# 설치 상세와 장비별 설정

일반 설치 명령은 [README의 설치 절차](../README.md#설치)에서 순서대로 실행할 수 있습니다. 이 문서는 확인된 장비 버전, 의존성 목록, `grid_map_msgs`의 소스 빌드 방법을 담고 있습니다. LiDAR, SLAM, localization과 저장된 지도를 쓰는 Nav2는 CPU에서 실행하며, 새 높이 지도를 생성할 장비에는 5절의 CUDA/CuPy 환경도 설치합니다.

## 1. 기준 환경 확인

2026-11-23 개발 장비에서 확인한 값입니다. 모든 항목의 최소 지원 버전을 뜻하지 않습니다.

| 항목 | 확인된 구성 |
|---|---|
| 장비/OS | Jetson Orin NX, aarch64, Ubuntu 22.04.5, kernel 5.15.148-tegra |
| Jetson BSP | `/etc/nv_tegra_release`: **L4T 36.4.3** |
| JetPack 메타패키지 | `nvidia-jetpack 6.2.1+b38` — BSP 번호와 별도로 확인한 값 |
| ROS / Python / GCC | ROS 2 Humble / Python 3.10.12 / GCC 11.4 |
| CUDA Toolkit / NVCC / NVRTC | 12.6.11 / 12.6.68 / 12.6 |
| CuPy | `cupy-cuda12x 13.6.0`, `fastrlock 0.8.4` |
| NumPy / SciPy / Open3D | 1.24.2 / 1.8.0 / 0.18.0 |
| ros2-numpy / PyYAML | 0.0.5 / 5.4.1 |
| Eigen / PCL / yaml-cpp | 3.4.0 / 1.12.1 / 0.7.0 |
| Fast DDS RMW | `rmw_fastrtps_cpp` 6.2.10 |
| grid_map 소스 대안 | commit `d9dbb58fb3e070de000775cefcfd8951515abee8` |

CuPy의 `show_config()`는 **build/linked runtime 12.9, locally installed runtime 및 driver 12.6**을 표시합니다. 이를 시스템 CUDA 12.9 설치 요구로 해석하지 않습니다. 현재 Toolkit/NVRTC는 12.6이며 실제 GPU 커널 실행으로 확인합니다.

새 Jetson은 제조사 BSP/JetPack 이미지에 맞춰 준비합니다. NVIDIA의 [JetPack 6.2 안내](https://developer.nvidia.com/embedded/jetpack-sdk-62)는 L4T 36.4.3·CUDA 12.6 구성을 설명합니다. CuPy 13.6의 Python/CUDA 및 aarch64 wheel 지원 범위는 [공식 설치 문서](https://docs.cupy.dev/en/v13.6.0/install.html)를 참고합니다. 이 workspace는 그중 위 환경에서 확인했습니다. 일반 PC나 다른 JetPack 버전은 별도 검증 대상입니다.

```bash
cat /etc/os-release
cat /etc/nv_tegra_release
cat /usr/local/cuda/version.json
/usr/local/cuda/bin/nvcc --version
python3 --version
```

## 2. ROS와 기본 의존성

Ubuntu 22.04에서 ROS 2 Humble Desktop을 설치하고 다른 ROS 워크스페이스를 source하지 않은 Bash에서 시작합니다. ROS가 처음인 장비는 공식 안내의 저장소 설정과 Desktop 설치까지 진행하세요. [ROS 2 Humble Ubuntu 설치 안내](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

```bash
source /opt/ros/humble/setup.bash
sudo apt update
sudo apt install \
  build-essential cmake git pkg-config liblcm-dev liblcm-bin python3-dev python3-pip python3-venv \
  python3-colcon-common-extensions python3-rosdep \
  python3-numpy python3-scipy python3-matplotlib python3-yaml python3-open3d \
  libeigen3-dev libpcl-dev libyaml-cpp-dev \
  ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-nav2-collision-monitor \
  ros-humble-rviz2 ros-humble-rmw-fastrtps-cpp \
  ros-humble-pcl-ros ros-humble-pcl-conversions \
  ros-humble-tf-transformations ros-humble-sensor-msgs-py

# 소스를 처음 받을 때만 실행합니다. 이미 폴더를 전달받았다면 다음 cd부터 실행합니다.
git clone --branch jetson/real https://github.com/Kimgyoomin/legged_robot_ws.git ~/legged_robot_ws
cd ~/legged_robot_ws
python3 -m pip install --user -r requirements-localization.txt
```

Open3D는 apt로 설치합니다. 개발 장비의 0.18.0과 apt 제공 버전이 다를 수 있으므로 아래 import 검사와 실제 PCD 초기 정합을 확인합니다. 설치 과정에서 시스템 Python이나 CUDA를 일괄 교체하지 않습니다.

`grid_map_msgs`는 전역 높이 지도를 누적하는 패키지의 빌드 의존성입니다.

```bash
sudo apt install ros-humble-grid-map-msgs
```

### grid_map_msgs를 소스로 설치할 때

apt에서 위 패키지를 설치했다면 바로 다음 절로 갑니다. 해당 바이너리가 제공되지 않는 장비는 아래 두 패키지를 먼저 빌드합니다. 이미 같은 소스를 받았다면 `git clone` 줄을 건너뜁니다.

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
  --build-base .deps_build --install-base .deps_install \
  --packages-select grid_map_cmake_helpers grid_map_msgs --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source .deps_install/local_setup.bash
```

### 나머지 패키지 의존성

나머지 manifest 의존성을 설치합니다. rosdep 최초 사용 장비에서만 `sudo rosdep init`을 먼저 실행합니다.

```bash
cd ~/legged_robot_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
```

로컬 grid_map underlay를 사용하면 source한 상태에서 마지막 명령에 `--skip-keys "grid_map_msgs grid_map_cmake_helpers"`를 추가합니다. CuPy와 ros2-numpy는 rosdep만으로 준비되지 않습니다.

## 3. Livox SDK2 빌드

동봉한 `third_party/Livox-SDK2`를 workspace 안의 `.sdk`에 설치합니다.

```bash
cd ~/legged_robot_ws
cmake -S third_party/Livox-SDK2 -B build/sdk \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_SAMPLES=OFF \
  -DCMAKE_INSTALL_PREFIX="$PWD/.sdk"
cmake --build build/sdk -j2
cmake --install build/sdk
```

## 4. ROS 패키지 빌드

다른 workspace를 source하지 않은 새 터미널에서 실행합니다.

```bash
cd ~/legged_robot_ws
source /opt/ros/humble/setup.bash
if [ -f .deps_install/local_setup.bash ]; then
  source .deps_install/local_setup.bash
fi
CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2 \
  colcon build --base-paths src --symlink-install --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
source install/setup.bash

ros2 pkg prefix livox_ros_driver2
ros2 pkg prefix pongbot_lcm
ros2 pkg prefix fast_lio
ros2 pkg prefix fast_lio_localization
ros2 pkg prefix rubi_height_mapping
ros2 pkg prefix legged_navigation
python3 -c 'import numpy, scipy, open3d, ros2_numpy, tf_transformations; print("Localization imports OK")'
```

각 prefix가 `~/legged_robot_ws/install/` 아래여야 합니다. `--base-paths src`로 archive의 과거 중복 패키지는 빌드하지 않습니다. `fast_lio`는 이번 납품부터 활성 패키지에 포함됩니다. 다른 장비나 다른 경로로 옮기면 SDK와 workspace를 다시 빌드합니다.

## 5. CUDA Height Mapping 환경

```bash
cd ~/legged_robot_ws
python3 -m venv --system-site-packages .venv-height
.venv-height/bin/python -m pip install --only-binary=:all: -r requirements-height.txt
touch .venv-height/COLCON_IGNORE
source tools/height_env.sh
ros2 run rubi_height_mapping check_environment
```

`check_environment`의 GPU·커널 검사가 성공해야 합니다. ROS 실행 파일은 `/usr/bin/python3` shebang을 사용할 수 있으므로 venv activate만 하지 말고 **`height_env.sh`를 source**합니다. PyTorch는 mapper 의존성이 아닙니다. `cupy`, `cupy-cuda11x`, `cupy-cuda12x`를 같은 환경에 중복 설치하지 않습니다.

## 6. 센서·지도·통신 준비

- 센서 설정: `src/livox_ros_driver2/config/MID360s_config.json`. 현재 host `192.168.1.5`, LiDAR `192.168.1.145`, roll `180°`. 납품 장비의 NIC·센서 IP·장착에 맞춥니다.
- 기본 driver: **`msg_MID360s_launch.py`**. RViz 없이 CustomMsg `/livox/lidar`(설정상 약 10 Hz), PointCloud2 `/livox/lidar_pointcloud`(매 2번째 스캔, 약 5 Hz), IMU를 발행합니다.
- Localization은 `input_format:=custommsg`. Height mapper는 `/cloud_registered_body`, Nav2 필터는 `/livox/lidar_pointcloud`를 받습니다.
- 지도는 Git과 별도 전달합니다. PCD, RGHM, hybrid용 YAML/PGM을 같은 장소·좌표계 세트로 관리합니다.
- [DDS.md](DDS.md)의 domain/profile을 사용합니다. 로컬 XML 경로는 셸/launch가 설정하므로 수신자가 개인 홈 경로를 하드코딩할 필요가 없습니다.

LiDAR를 켜기 전에 유선 포트의 IPv4 주소와 센서로 향하는 경로를 확인합니다. RUBI 설정에서는 두 번째 명령의 `dev`가 센서에 연결한 인터페이스이고 `src`가 `192.168.1.5`여야 합니다. 새 장비는 해당 포트의 유선 IPv4 설정과 JSON의 `host_ip`를 일치시킵니다.

```bash
ip -br -4 addr
ip route get 192.168.1.145
```

설치가 끝나면 새 터미널마다 공통 환경을 적용하고 A의 LiDAR부터 실행합니다. [터미널 환경과 패키지별 명령](../README.md#터미널-환경) · [실물 운용 상세](OPERATIONS.md) · [rosbag 재생](HEIGHT_REPLAY.md)
