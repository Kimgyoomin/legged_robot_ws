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

이 Jetson에서는 `sudo apt upgrade`를 실행하지 않습니다. 아래 의존성 설치도 기존 패키지를 먼저 확인한 뒤 필요한 항목만 검토하여 진행하며, 지도 복사·Git 정리에 시스템 업데이트는 필요하지 않습니다.

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
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
cmake -S third_party/Livox-SDK2 -B build/sdk \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_SAMPLES=OFF \
  -DCMAKE_INSTALL_PREFIX="$PWD/.sdk"
cmake --build build/sdk -j2
cmake --install build/sdk
colcon build --symlink-install --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 run rubi_height_mapping check_environment
```

SDK는 이 workspace의 `.sdk`에 설치합니다. 기존 `/usr/local` SDK를 덮어쓰지 않습니다. Livox driver는 이 위치를 명시적으로 링크합니다. 다른 PC로 배포할 때 `src`, `third_party`, `docs`, `tools`, 지도 세트를 복사하고 **그 PC에서 다시 빌드**합니다. PC의 `build`, `install`, `.sdk` 바이너리를 Jetson으로 복사하지 않습니다.

수정된 `src/livox_ros_driver2`와 `third_party/Livox-SDK2`는 submodule이 아닌 일반 소스로 저장합니다. ROS 2용 `src/livox_ros_driver2/package.xml`도 포함하므로 위 `colcon build`를 사용합니다. 드라이버만 먼저 확인하려면 `colcon build --symlink-install --packages-select livox_ros_driver2 --cmake-args -DCMAKE_BUILD_TYPE=Release`로 범위를 제한할 수 있습니다. 드라이버 단독 빌드 성공이 전체 localization/navigation 의존성 충족을 의미하지는 않습니다.

`--executor sequential`과 `-j2`는 빌드 중 메모리 사용을 줄이기 위한 선택이며, 런타임의 BLAS·OMP 스레드를 제한하는 설정이 아닙니다. 새 launch는 `OPENBLAS_NUM_THREADS`, `OMP_NUM_THREADS`, `MKL_NUM_THREADS`를 강제로 설정하지 않습니다.

## 5. MID-360S 설정

Bringup은 `src/legged_bringup/config/sensor.json`, 드라이버 단독 `rviz_MID360s_launch.py`는 `src/livox_ros_driver2/config/MID360s_config.json`을 사용합니다. 두 설정을 함께 확인합니다.

- 모델 키는 `Mid360s`입니다.
- 현재 sensor IP는 `192.168.1.145`, host IP는 `192.168.1.5`입니다. 다른 장치에서는 실제 센서·네트워크 설정에 맞춥니다.
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

## 8. 기존 RUBI 실물 지도와 Git 배포 정책

기존 `navigation_real`의 원본은 보존하고 아래 세 파일을 workspace 안으로 복사합니다. 런타임은 `site.yaml` 기준 상대경로를 사용하므로 이전 workspace나 `/home/rclab` 경로에 의존하지 않습니다.

```text
maps/rubi_1f_legacy/
  site.yaml                              # Git 포함: 경로, SHA-256, 출처, QC 상태
  data/                                  # Git 제외: 별도 전달/백업
    1f_fast_lio_260731_segmented.pcd
    RUBI_1F_occupancy_map.yaml
    RUBI_1F_occupancy_map.pgm
```

처음 clone한 사람에게는 `data/` 세 파일을 별도로 전달해야 합니다. 이미 받은 `data/`를 위 위치에 배치하거나, 기존 파일들이 있는 `navigation_real`에서 다음과 같이 복원합니다. `--source-root`는 각 PC의 실제 경로로 변경합니다.

```bash
cd ~/legged_robot_ws
python3 tools/import_legacy_rubi_maps.py \
  --source-root /home/rclab/ros2_ws/src/navigation_real
```

복원 도구는 manifest의 `provenance.source_assets`에 기록된 상대경로에서 원본을 읽고, 모든 원본의 SHA-256을 먼저 검사합니다. 기존 복사본이 같으면 재사용하고, 다른 내용이면 덮어쓰지 않고 중단합니다. PGM 파일명과 occupancy YAML의 `image` 참조도 그대로 유지합니다. 원본 지도나 `alignment.verified`는 변경하지 않습니다.

**이 지도 묶음은 아직 새 패키지에서 주행 검증을 완료한 사이트가 아닙니다.** `alignment.verified: false`이므로 현재 loader는 실행을 거부합니다. PCD와 occupancy의 좌표 정합을 실제 확인하고 QC 근거를 남긴 뒤에만 승인해야 합니다. Localization은 height 없이 사용할 수 있지만, 새 navigation 경로는 정합된 height asset도 요구하므로 기존 두 지도만으로 주행 준비가 끝나지는 않습니다.

정합 확인·승인 및 필요한 패키지 빌드 후 localization에 전달할 경로는 다음과 같습니다.

```bash
cd ~/legged_robot_ws
source install/setup.bash
ros2 launch legged_bringup localize.launch.py \
  site:="$PWD/maps/rubi_1f_legacy/site.yaml" method:=fastlio use_sim_time:=false
```

Git 포함 대상은 소스·SDK 소스·ROS `package.xml`·센서 JSON·launch/config YAML·지도 manifest·복원 도구입니다. `.gitignore`는 새 실물 `data/`, 새 생성 지도/PCD/height 파일, rosbag 데이터와 `metadata.yaml`, 빌드·설치·캐시·`.sdk` 산출물을 제외합니다. `metadata.yaml`은 rosbag용 제외 규칙이며 일반 YAML/JSON 설정을 통째로 제외하지 않습니다.

**기존에 커밋된 시뮬레이션/검증 지도는 삭제하거나 추적 해제하지 않습니다.** `.gitignore`는 이미 추적 중인 파일의 변경을 숨기지 않습니다. 새 테스트용 PCD/height fixture를 추가하려면 demo 예외 경로를 사용하거나, 필요한 파일만 검토하여 예외 규칙을 추가합니다. `git add .` 후에는 `git diff --cached --stat`으로 포함 파일을 확인하고, 실제 지도 데이터는 Git과 별도로 백업합니다.
