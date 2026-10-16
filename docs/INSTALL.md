# 설치 안내

## 1. 지원 기준과 미확인 환경

| 항목 | 기준 |
|---|---|
| OS / ROS | Ubuntu 22.04 / ROS 2 Humble |
| Python / C++ | 시스템 Python 3.10 / C++17, GenZ는 C++20 |
| GPU 지도 생성 | CUDA에 맞는 CuPy 13.6.0, NumPy 1.x |
| Frozen 지도 주행 | CuPy 불필요; CPU mapper로 대체한다는 뜻은 아님 |
| 대상 장치 | Jetson Orin NX 16 GB / 25 W, JetPack 버전 미확인 |
| 검증한 장치 | x86_64 PC + RTX 5090, CuPy 13.6.0 |

JetPack 6.2.1은 Ubuntu 22.04 기반 Jetson Linux와 CUDA 12.6을 포함하는 공식 구성입니다. 이를 환경 선택의 참고로 사용하되, 현재 Jetson 버전을 확인하지 않고 OS나 CUDA를 덮어쓰지 않습니다. [NVIDIA 릴리스 노트](https://docs.nvidia.com/jetson/jetpack/6.2.1/release-notes/index.html)

```bash
cat /etc/nv_tegra_release
cat /etc/os-release
nvcc --version
python3 --version
dpkg-query -W nvidia-jetpack
```

JetPack 5 / Ubuntu 20.04라면 이 Humble 배포 조합과 동일하지 않습니다. 먼저 OS·ROS 조합을 정해야 합니다.

## 2. ROS 의존성

ROS 2 Humble 설치 후, 새 터미널에서 실행합니다.

```bash
source /opt/ros/humble/setup.bash
sudo apt update
sudo apt install python3-colcon-common-extensions python3-rosdep \
  python3-pip python3-numpy python3-scipy python3-matplotlib python3-yaml \
  python3-open3d libeigen3-dev libpcl-dev libyaml-cpp-dev \
  ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-nav2-collision-monitor \
  ros-humble-grid-map-msgs ros-humble-pcl-ros ros-humble-pcl-conversions \
  ros-humble-tf-transformations ros-humble-sensor-msgs-py
cd ~/legged_robot_ws
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
```

`rosdep`가 처음이라면 시스템 설치 절차에 따라 한 번 초기화합니다. GenZ CMake는 사용 가능한 Eigen/TBB/Sophus 등의 의존성을 찾거나 upstream에서 내려받으므로 최초 빌드에는 네트워크가 필요할 수 있습니다. SDK 소스는 `third_party/Livox-SDK2`에 포함되어 있습니다.

FAST-LIO의 Python 전역 정합은 Open3D를 사용합니다. **Height mapper에서 Open3D·PyTorch를 제거했다는 것과 전체 localization까지 의존성이 없다는 것은 다릅니다.** 새 배포본의 FAST-LIO 전역 정합에서 `ros2_numpy`는 제거하고 ROS의 `sensor_msgs_py`를 사용합니다.

## 3. GPU 지도 생성용 CuPy

CUDA 12.x인 경우의 고정 버전 예시입니다. 시스템 CUDA 버전을 먼저 확인합니다.

```bash
python3 -m pip install --user 'numpy>=1.22,<2' 'cupy-cuda12x==13.6.0'
python3 -c "import cupy as cp; print(cp.__version__); print(cp.arange(8).sum().get())"
```

CUDA 13.x 환경에서는 해당 CUDA와 CuPy 버전에 맞는 패키지를 선택해야 합니다. `cupy`, `cupy-cuda12x`, `cupy-cuda13x`를 동시에 설치하지 않습니다. 아키텍처·Python·CUDA 조합에 맞는 wheel이 없으면 source build가 필요합니다. 설치 방법은 [CuPy 공식 안내](https://docs.cupy.dev/en/v13.6.0/install.html)를 기준으로 합니다. 최신 CuPy 14와 NumPy 2로 자동 업그레이드하는 구성은 이 배포본의 검증 범위가 아닙니다.

Jetson에는 JetPack CUDA를 사용합니다. PC용 드라이버 설치 명령이나 PC의 `.python_deps` 폴더를 복사하지 않습니다. Frozen 지도 주행만 하는 장치에는 이 단계가 필요하지 않습니다.

## 4. 소스 빌드

```bash
cd ~/legged_robot_ws
source /opt/ros/humble/setup.bash
cmake -S third_party/Livox-SDK2 -B build/sdk \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$PWD/.sdk"
cmake --build build/sdk -j2
cmake --install build/sdk
colcon build --symlink-install --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 run rubi_height_mapping check_environment
```

SDK는 이 workspace의 `.sdk`에 설치합니다. 기존 `/usr/local` SDK를 덮어쓰지 않습니다. Livox driver는 이 위치를 명시적으로 링크합니다. 다른 PC로 배포할 때 `src`, `third_party`, `docs`, `tools`, 지도 세트를 복사하고 **그 PC에서 다시 빌드**합니다. PC의 `build`, `install`, `.sdk` 바이너리를 Jetson으로 복사하지 않습니다.

`--executor sequential`과 `-j2`는 빌드 중 메모리 사용을 줄이기 위한 선택이며, 런타임의 BLAS·OMP 스레드를 제한하는 설정이 아닙니다. 새 launch는 `OPENBLAS_NUM_THREADS`, `OMP_NUM_THREADS`, `MKL_NUM_THREADS`를 강제로 설정하지 않습니다.

## 5. MID-360S 설정

`src/legged_bringup/config/sensor.json`:

- 모델 키는 `Mid360s`, device type은 8입니다.
- 기본 예시 sensor IP는 `192.168.1.12`, host IP는 `192.168.1.5`입니다. 실제 라벨·네트워크 설정에 맞춥니다.
- `roll: 180`, `pitch: 0`, `yaw: 0`은 현재 upside-down 장착 조건입니다.
- driver translation은 `[0,0,0]`으로 유지합니다. 물리 장착 위치는 `frames.yaml`과 FAST-LIO extrinsic으로 표현합니다.
- 이미 회전한 `/livox/lidar`·`/livox/imu`에 별도 axis-flip 노드를 다시 연결하지 않습니다.

출력은 `/livox/lidar` = `sensor_msgs/PointCloud2`, `/livox/imu` = `sensor_msgs/Imu`입니다. 파일 이름에 `msg`가 있다는 이유로 형식을 추정하지 않습니다. [고정한 공식 드라이버 문서](https://github.com/Livox-SDK/livox_ros_driver2/blob/21445540f0d100dc86a7e6df312dd70bbdb4afdf/README.md)

## 6. 최초 확인

```bash
ros2 pkg prefix legged_bringup
ros2 pkg prefix fast_lio_localization
ros2 pkg prefix livox_ros_driver2
ros2 topic info /livox/lidar
ros2 topic echo /livox/imu --once
```

패키지 prefix가 `legged_robot_ws/install/...`인지 확인합니다. `/livox/lidar`의 `timestamp` 필드는 FLOAT64 절대 nanosecond이어야 합니다. Header와 IMU가 같은 clock domain인지, 정지 시 가속도 방향이 기대한 축인지, 전방 landmark가 기대한 위치인지 확인합니다. 물리 설치와 새 센서 시리얼은 이 PC 시험에서 확인하지 않았습니다.

## 7. 흔한 오류

| 증상 | 확인 |
|---|---|
| package를 못 찾음 | 새 터미널에서 새 workspace의 `install/setup.bash` source |
| SDK library를 못 찾음 | `.sdk` 설치 완료 여부, workspace 이동 후 재빌드 |
| `Livox cloud requires field timestamp` | XYZ-only cloud나 simulator cloud를 물리 MID-360 parser에 넣지 않았는지 확인 |
| timestamp unit 오류 | driver 절대 ns와 CustomMsg 상대 ns를 혼동하지 않았는지 확인 |
| map frame 불일치 | `map` 이름만 바꾸지 말고 실제 변환·정합 수행 |
| 첫 scan에 odometry bracket 없음 | 시작 시 캐시 이전 scan은 버려질 수 있음. 지속 발생이면 stamp·주기·clock 확인 |
| 지도는 보이지만 로봇이 정지 | `/legged/obstacles`, `/Odometry` 또는 GenZ odom, costmap, Collision Monitor 활성 상태 확인 |
| Nav2가 낮은 terrain을 모두 장애물로 봄 | 2D static map과 obstacle height gate를 사이트의 지면 높이에 맞춰 확인 |
