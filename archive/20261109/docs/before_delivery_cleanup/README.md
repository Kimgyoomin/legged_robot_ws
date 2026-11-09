# legged_robot_ws

RUBI 이족보행 로봇용 **LiDAR → localization → height mapping → 전역 경로 → 주행** ROS 2 workspace입니다. 기존 소스를 복사해 구성했으며, 원본 `ros2_ws_nav`, `rubi_mapping_ws`는 변경하지 않았습니다.

**현재 작업 상태 (2026-11-09):** Jetson에서 실물 기록 기반 height 지도를 생성하고, 버전별 후처리 지도, Nav2 planner-only 검토, 원시 bag 기반 FAST-LIO localization + HeightPlanner + DWB 연동을 연결했습니다. 저장된 지도 경로 검토와 height 기반 실차 주행 검증은 구분합니다. `maps/demo`는 합성 시험 지도입니다.

- [소스 패키지·설정 위치](src/README.md)
- [실제 bag 기반 localization + Height Nav2 영상 실행](docs/HEIGHT_REPLAY.md)
- [현재 height 지도와 Nav2 실행](docs/HEIGHT_NAVIGATION.md)
- [v4 바닥 보간 결과](maps/rubi_1f_height_20261109_floorcomplete_v4/REPORT.md)

| 기능 | 구성 |
|---|---|
| 센서 | MID-360S 공식 Livox driver 1.2.8 기반, 점군·IMU 동일한 180° 회전 |
| Localization | FAST-LIO 또는 GenZ-ICP 중 하나 선택 |
| 지도 생성 | 독립 `rubi_height_mapping` CuPy/CUDA 추정 + C++ 전역 누적 |
| 오프라인 | 평면 내부 잔노이즈 후처리, 변환·재격자화, 지도 세트 해시 저장 |
| 전역 경로 | 기존 RUBI Grid A* + 높이 비용, RUBI는 geometry / generic은 설정에 따른 CoT, 같은 core 사용 |
| 주행 | `nav2`: DWB 지역 회피 / `pursuit`: 기존 Pure Pursuit + 전역 재계획 |
| generic 주행 정지 | Collision Monitor + 센서·odom·costmap·명령 갱신 watchdog |
| RUBI 주행/재생 | 성공한 navigation_real의 DWB 5Hz·BODY 필터·실제 속도 피드백; generic 정지 체인과 별도 |

![전체 구성](docs/architecture.svg)

## 설치

Ubuntu 22.04 / ROS 2 Humble / Python 3.10을 기준으로 합니다. GPU **지도 생성**에는 호환되는 NVIDIA CUDA와 CuPy가 필요합니다. 저장된 지도만 읽어 계획할 때는 CuPy를 import하지 않습니다.

[설치 안내](docs/INSTALL.md)를 따라 의존성을 설치한 후:

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
```

기존 workspace를 함께 source하지 않는 새 터미널을 사용합니다. 임의 `PYTHONPATH`, `.python_deps`, CPU 수치연산 스레드 제한은 필요하지 않습니다.

## 실행

아래는 generic launch다. 이번 RUBI 실물 기준 설정과 bag 영상은 [전용 replay 실행](docs/HEIGHT_REPLAY.md)을 사용한다.

각 터미널에서 `source ~/legged_robot_ws/install/setup.bash`를 실행합니다. 실제 사이트의 `site.yaml`에는 PCD·2D 지도·height map과 정합 QC 기록이 있어야 합니다. 자세한 생성 방법은 [운영 안내](docs/OPERATIONS.md)에 있습니다.

```bash
# 1. 센서: config/sensor.json에서 실제 센서 IP와 PC IP를 먼저 수정
ros2 launch legged_bringup sensor.launch.py

# 2. Localization: 둘 중 하나. RViz의 2D Pose Estimate로 초기 위치 입력
ros2 launch legged_bringup localize.launch.py \
  site:=/absolute/path/site.yaml method:=fastlio
# GenZ 대안: method:=genz

# 3. 지도 생성 세션: FAST-LIO 움직임 보정 점군 + map 기준 odometry
ros2 launch legged_bringup mapping.launch.py use_sim_time:=false

# 4. 저장된 지도로 주행: 둘 중 하나만 실행
ros2 launch legged_bringup navigate.launch.py \
  site:=/absolute/path/site.yaml mode:=nav2
ros2 launch legged_bringup navigate.launch.py \
  site:=/absolute/path/site.yaml mode:=pursuit
```

GenZ로 주행할 때는 navigation 명령에 `odom_topic:=/genz/odometry`를 추가합니다. 시뮬레이션에서는 각 launch에 `use_sim_time:=true`를 추가하고, 이미 존재하는 장착 TF는 `localize.launch.py publish_mount:=false`로 중복 발행하지 않습니다. 실제 센서 launch는 시뮬레이터와 함께 실행하지 않습니다.

Nav2 모드 목표는 RViz의 **Nav2 Goal**, Pure Pursuit 모드 목표는 **2D Goal Pose**(`/goal_pose`)입니다. 로봇 구동기/LCM bridge는 포함해 자동 실행하지 않으며, 최종 `/cmd_vel`을 기존 로봇 인터페이스로 연결합니다.

## 수정할 설정

설정 위치: [`src/legged_bringup/config`](src/legged_bringup/config)

| 파일 | 역할 |
|---|---|
| `sensor.json` / `frames.yaml` | 센서 IP·회전 / 장착 위치 |
| `fastlio.yaml` / `genz.yaml` | 선택한 localization 설정 |
| `mapping.yaml` | Height 입력·clock·frame·거리·RViz·저장 경로 (기본 FAST-LIO 시뮬레이션) |
| `height.yaml` / `global.yaml` | GPU 지역 추정 / 전역 누적 범위 |
| `planner.yaml` | 두 모드 공통 높이 비용·통과 한계·재계획 |
| `nav2.yaml` / `pursuit.yaml` | 주행 모드별 추종 설정 |
| `costmap.yaml` / `obstacle.yaml` / `collision.yaml` | 장애물 지도 / 거리 제외 / 근접 정지 |

## 문서

- [상세 기술 보고서](docs/REPORT.md) · [브라우저용 보고서](docs/report.html)
- [설치와 Jetson 환경](docs/INSTALL.md)
- [지도 취득·정합·동결·배포 운영](docs/OPERATIONS.md)
- [CustomMsg·PointCloud2·timestamp 변경 근거](docs/TIMING.md)
- [검증 결과](docs/VALIDATION.md) · [소스·라이선스 출처](docs/THIRD_PARTY.md)

이 workspace는 ramp/step 지형 분류기를 추가하지 않습니다. Height 비용과 로봇 통과 한계로 경로를 평가합니다. 5 cm 격자의 경계 혼합과 localization 오차는 별도 검증이 필요합니다. 미관측 바닥의 후처리 보간은 원본을 보존한 새 버전에만 저장하고, 추론 마스크와 사용한 근거를 함께 기록합니다.

## 시뮬레이션 height mapping 간단 실행

시뮬레이터와 FAST-LIO localization을 먼저 실행하고 초기 위치를 맞춘 뒤:

```bash
ros2 launch legged_bringup mapping.launch.py
```

[config/mapping.yaml](src/legged_bringup/config/mapping.yaml)에 운영 설정을 모았습니다.
기본 입력은 `/cloud_registered_body` + `/localization`이며 시뮬레이션 시간과
RViz가 켜져 있습니다. 실물에서는 `use_sim_time: false`로 바꿉니다.
