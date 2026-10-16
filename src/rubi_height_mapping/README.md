# RUBI Height Mapping 0.3.1 — 납품 검토용 소스

ROS 2 Humble에서 Livox XYZ + odometry 토픽 또는 localization TF로 GPU 지역 높이·분산을 계산하고,
C++ wrapper로 전역 누적·저장합니다. upstream 높이 알고리즘을 재사용하지만
`elevation_mapping_cupy` 패키지를 설치할 필요는 없습니다.

0.3.1은 한 scan의 이전 지도를 읽기 전용으로 두고, 높이·ray 후보를 모은 뒤
셀마다 결과를 한 번 확정합니다. 같은 scan의 유효 높이 관측은 ray 정리보다
우선합니다. 좌표·높이 상태는 float32, 후보의 합산만 float64이며 새 의존성은 없습니다.
기존 launch로 적용되며 실행 중인 노드에는 재시작 후 반영됩니다.

## 실행

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run rubi_height_mapping check_environment
ros2 launch rubi_height_mapping mapping.launch.py use_sim_time:=true rviz:=true
```

Gazebo/센서, localization 및 정적 센서 TF는 별도 실행합니다. 모든 노드의
ROS_DOMAIN_ID가 같아야 합니다(현재 개발 PC는 45). 실제 센서는
`use_sim_time:=false`를 사용합니다. launch는 CPU 스레드 제한, PYTHONPATH,
CUDA 경로를 설정하지 않습니다. 정상 CUDA/CuPy 설치 환경이 전제입니다.

기본 입력 `/livox/lidar_PointCloud2`에 0.6–3.0 m 거리 필터가 포함되어 별도
거리 필터 패키지나 FastDEM은 필요 없습니다. 이미 필터링한 입력은
`input_scan:=/rubi/lidar/roi_filtered`로 선택합니다. `min_range`, `max_range`,
`tf_wait_timeout`을 launch 인자로 조절합니다. 기본 TF 대기 1.5초는 현재
시뮬레이션용이며 장비에서 다시 조정합니다. `rviz:=false`로 GUI를 끕니다.

기존 mapper/wrapper와 같은 토픽으로 중복 실행하지 마세요. 새 실행은 빈 지도에서
시작하며 Ctrl+C로 mapper·wrapper·RViz를 함께 종료합니다.

## 인터페이스

- 입력: little-endian FLOAT32 XYZ PointCloud2, 원본 stamp/frame 유지.
- 기본 TF 모드: scan 시각의 `map <- cloud.frame_id`, `map <- base_link`.
- 직접 odometry 모드: `odom_topic`으로 nav_msgs/Odometry 수신. 아래 참고.
- 지역 출력: `/rubi/local_elevation/grid_map`, GridMap elevation/variance,
  기본 5 Hz, 8×8 m / 5 cm.
- 전역 출력: `/rubi/global_elevation/cloud`, PointCloud2, 기본 1 Hz,
  50×50 m / 5 cm. 과거 유효 관측 유지, 미관측 영역 보간 없음.
- 전역 Trigger 서비스: `/rubi/global_elevation/{save_map,load_map,clear_map}`.
- 출력 stamp는 마지막으로 제출한 scan 시각이며 모든 셀의 최신 관측을 뜻하지 않습니다.

```bash
ros2 service call /rubi/global_elevation/save_map std_srvs/srv/Trigger '{}'
```

wrapper YAML `output_directory`(기본 `/tmp/rubi_mapping_results`) 아래 새 디렉터리에
`map.rghm`, `surface.pcd`를 저장합니다. 납품 지도는 영구 경로로 변경하세요.
복원은 wrapper YAML `load_path`를 지정해 실행 후 `load_map`을 호출합니다.
좌표계/시뮬레이션 epoch를 혼합하지 마세요. Clear는 전역 wrapper만 지우므로
새 실험은 mapper와 소비자를 함께 재시작합니다. 파라미터는 시작 시 확정됩니다.

## Odometry 토픽으로 자세 선택

`odom_topic`이 비어 있으면 기존 TF 방식입니다. 토픽을 지정하면 해당 odometry의
pose만 사용하며 동적 `/tf`는 구독하지 않습니다. 외부 TF broadcaster를 추가할 필요가 없습니다.
점군 시각의 위치는 선형 보간, 방향은 quaternion SLERP로 보간합니다.
관측 전후의 odometry가 없으면 기존 입력 큐에서 기다리며, 대기 시간을 넘기면 스캔을 버립니다.
최신 자세 대체나 외삽은 하지 않습니다. 기본 `odom_max_gap:=0.2`초보다 긴 간격은
보간하지 않습니다. 입력 캐시는 최대 10초/2,000 자세이며 필요하면 YAML에서
`odom_cache_seconds`를 변경합니다. 이 경로에서도 시뮬레이션 시계가 되감기면 재시작이 필요합니다.

변환 관계는 `T_map_sensor = T_odom_pose × T_child_sensor`입니다.
Odometry의 `header.frame_id`는 `map_frame`과 정확히 같아야 합니다.
`child_frame_id`는 pose가 나타내는 링크입니다(BODY, body, base_link는 서로 다릅니다).
해당 링크에서 센서 및 `base_frame`까지의 **고정 장착 변환만 `/tf_static`에서** 읽습니다.
관절 TF는 필요 없습니다. 별도 로봇 설명 또는 static_transform_publisher로 실제 장착 변환을
제공해야 하며, 이 패키지는 알려지지 않은 센서 위치를 자동 추정하지 않습니다.

현재 RUBI 시뮬레이션에서, 기존 height mapper/wrapper를 종료한 뒤 다음 중 하나를 실행합니다.
Gazebo 및 localization은 그대로 둘 수 있습니다. 각 실행은 빈 지도에서 시작합니다.

```bash
cd /home/kim/rubi_mapping_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

# A: Gazebo GT (world -> BODY)
ros2 launch rubi_height_mapping mapping.launch.py \
  use_sim_time:=true rviz:=true \
  odom_topic:=/ground_truth/odom map_frame:=world base_frame:=BODY

# B: FAST-LIO local odometry (camera_init -> body)
ros2 launch rubi_height_mapping mapping.launch.py \
  use_sim_time:=true rviz:=true \
  odom_topic:=/Odometry map_frame:=camera_init base_frame:=BODY

# C: 전역 localization (map -> body)
ros2 launch rubi_height_mapping mapping.launch.py \
  use_sim_time:=true rviz:=true \
  odom_topic:=/localization map_frame:=map base_frame:=BODY
```

위 세 명령은 **동시에 실행하는 명령이 아니라 비교할 실행 선택지**입니다.
launch의 `map_frame`은 mapper·전역 wrapper·RViz에 함께 적용됩니다.
`base_frame`은 지역 지도가 따라갈 로봇 링크입니다. GT 모드에서 기존 localization을 켜 두어도
그 동적 TF가 GT 지도에 섞이지 않습니다. RViz RobotModel은 기존 TF를 사용하므로
GT 지도와 로봇 모델의 겹침만으로 정합 품질을 판단하지 마세요.

기본 RViz 카메라의 `Target Frame`과 원점 축은 `<Fixed Frame>`을 사용합니다.
직접 odometry로 지도 좌표계를 바꿔도 기존 localization의 `base_link`를 따라가지 않습니다.
이미 열린 RViz에는 설정 파일 변경이 자동 적용되지 않으므로 `Views → Target Frame`을
`<Fixed Frame>`으로 바꾸거나 RViz 설정을 다시 불러옵니다. `Fixed Frame`은 지도와 같아야 합니다.
현재 시뮬레이션 localization의 항등 `world → map` TF는 Gazebo world와 PCD 지도가
정밀 정합되었다는 뜻이 아닙니다. 직접 GT 입력은 이 동적 TF 경로를 사용하지 않습니다.
높이 색상은 기본적으로 자동 범위를 사용하므로, 색깔 자체로 높이 오류를 판단하지 않습니다.
GT world의 바닥/단차 비교 시에는 AxisColor 범위를 예를 들어 −0.02~0.20 m로 고정할 수 있습니다.

이 기능은 자세 입력 선택과 시간 정합을 추가합니다. **Odometry covariance/twist는 아직
높이 분산에 전파하지 않으며**, 높이 backend·노이즈 모델·드리프트 보정은 변경하지 않았습니다.
한 스캔 전체에 하나의 자세를 적용하므로 포인트별 운동 보상(deskew)을 추가하는 기능도 아닙니다.
실제 MID-360의 움직이는 스캔은 별도의 시각/deskew 검증이 필요합니다.
이 입력 기능 검증과 실제 world의 GT/LIO 변형 비교는 별도입니다.

## 설치와 의존성

납품 묶음의 두 패키지를 새 workspace의 `src`에 넣습니다:

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select \
  rubi_global_heightmap_wrapper rubi_height_mapping \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source install/setup.bash
```

RViz 사용 PC에는 `ros-humble-rviz2`가 별도로 필요합니다. 직접 수치 계산 의존성은
NumPy/CuPy입니다. CuPy는 rosdep이 설치하지 않습니다. JetPack/CUDA/Python을
확인한 뒤 맞는 CuPy wheel 또는 소스 빌드를 선택하세요.
[CuPy 13.6 설치 기준](https://docs.cupy.dev/en/v13.6.0/install.html).
한 환경에 `cupy` 및 여러 `cupy-cudaXX` 배포본을 중복 설치하지 않습니다.

PyTorch, Chainer, 영상/semantic 입력, cv_bridge, SciPy, ros2_numpy 및 upstream
전용 메시지는 제외했습니다. 높이 갱신과 연결된 작은 신경망 점수는 같은 계산을
CuPy로 유지하므로 학습 계산을 완전히 삭제한 알고리즘은 아닙니다.
[변경/라이선스 내역](UPSTREAM.md)을 함께 배포하세요.

개발 PC 기준은 Python 3.10 / NumPy 1.26.4 / CuPy 13.6.0입니다. Jetson Orin NX
16 GB / 25 W의 JetPack 버전은 아직 미확인입니다. Orin 설치·성능·실제 센서
정확도 검증이 완료된 납품 승인본은 아닙니다. 전역 wrapper의 계약과 제약은
함께 배포되는 해당 README를 확인하세요.

## 검증 실행

기본 `colcon test`는 CPU 입력 계약과 C++ 전역 누적을 검사합니다. CUDA와 ROS가
설치된 개발 장비에서는 GPU·ROS 연결 검사도 켤 수 있습니다.

```bash
colcon build --packages-select rubi_height_mapping \
  --cmake-args -DBUILD_TESTING=ON -DRUBI_TEST_CUDA=ON
source install/setup.bash
colcon test --packages-select rubi_height_mapping rubi_global_heightmap_wrapper
colcon test-result --verbose
```

연결 검사는 ROS_DOMAIN_ID=183에서 합성 scan, 지연 TF, 센서 뒤집힘, 가려짐,
전역 저장/복원 및 시뮬레이션 시계 되감기를 검사합니다. 해당 도메인에서 다른
실험을 돌리지 마세요. 영상·semantic·PyTorch·원본 backend import를 실제로
차단한 상태로 GPU 계산을 검사합니다. 이 결과는 실제 로봇 보행 정확도나
Jetson 처리량을 보증하지 않습니다.

## Optional representative rays

`ray_mode:=voxel ray_voxel_size:=0.05` selects one original endpoint per 3D
voxel for visibility cleanup only. All height observations remain input to fusion.
Selection runs inside the existing GPU counting pass with reusable buffers.
Default `ray_mode:=all` preserves the original ray path. Ghost removal can be
slower with `voxel`, and sparse/ordinary scans may not become faster. Complex
recorded-cloud repeatability failed before snapshot fusion. Version 0.3.1 passes
the repeated fixed-pose capture test, but all/voxel are still different cleanup
policies: treat voxel as experimental, not a production accuracy improvement. See workspace
`docs/OPERATIONS.md` and `docs/VALIDATION.md` before deployment.
