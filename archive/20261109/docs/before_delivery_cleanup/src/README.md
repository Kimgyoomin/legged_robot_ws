# src 패키지 안내

2026-11-09 현재 소스와 설치 상태를 확인한 목록입니다. 폴더명과 ROS 패키지명은 다를 수 있습니다. 기존 launch·include 경로와 수정된 Livox 소스를 유지하기 위해 소스 폴더를 이동하거나 이름을 변경하지 않았습니다.

## 역할과 실행 경로

| 소스 폴더 | ROS 패키지명 | 역할 | 이 Jetson의 install 상태 |
|---|---|---|---|
| `navigation_dds` | `navigation_dds` | 공통/대용량 Fast DDS profile, launch·CLI wrapper | 설치됨 |
| `livox_ros_driver2` | `livox_ros_driver2` | MID-360S 점군/IMU, 좌표 회전·시간 처리 포함한 수정본 | 설치됨 |
| `FAST_LIO_LOCALIZATION2` | `fast_lio_localization` | PCD 기반 localization, 실물 standalone launch | 설치됨 |
| `rubi_height_mapping` | `rubi_height_mapping` | CuPy 지역 높이 추정 | 설치됨 |
| `rubi_global_heightmap_wrapper` | `rubi_global_heightmap_wrapper` | 전역 높이 누적·RGHM 저장 | 설치됨 |
| `rubi_heightmap_postprocess` | `rubi_heightmap_postprocess` | 오프라인 후처리·수동 편집 | 설치됨 |
| `rubi_heightmap_step_wavefront_planner` | `rubi_heightmap_step_wavefront_planner` | 높이 경로 탐색 core·단독 비교 도구 | 설치됨 |
| `legged_navigation` | `legged_navigation` | Nav2 HeightPlanner 플러그인·장애물 점군 처리·Pure Pursuit | 설치됨 |
| `legged_bringup` | `legged_bringup` | 지도 로더·launch·설정·watchdog·RViz 검토 | 설치됨 |
| `FAST_LIO_ROS2` | `fast_lio` | 대안 standalone mapping 소스 | 미설치, 소스 보존 |
| `genz-icp/ros` | `genz_icp` | 대안 odometry/localization 소스 | 미설치, 소스 보존 |

`미설치`는 2026-11-09 현재 workspace의 `install/<패키지>` 확인 결과이며, 미사용·삭제 가능 판정이 아닙니다. `legged_bringup/package.xml`은 FAST-LIO와 GenZ 대안을 실행 의존성으로 선언하고 있어 일괄 삭제하지 않습니다. `genz-icp/cpp/COLCON_IGNORE`도 기존 구조대로 유지합니다.

```text
Livox → FAST-LIO localization → rubi_height_mapping
                                  ↓
                     rubi_global_heightmap_wrapper → maps/raw/
                                  ↓
                      rubi_heightmap_postprocess / tools/
                                  ↓
                버전별 maps/.../site.yaml + map.rghm
                                  ↓
                      legged_bringup frozen_map
                                  ↓
              legged_navigation HeightPlanner → planner core
```

## 현재 수정할 설정 위치

| 목적 | 위치 |
|---|---|
| 센서 IP·Livox 출력 | `livox_ros_driver2/config/`, `launch_ROS2/` |
| 실물 standalone localization | `FAST_LIO_LOCALIZATION2/launch/localization_nav_rubi.launch.py`, `config/mid360s_nav.yaml`(PointCloud2), `config/rubi_custommsg.yaml`(녹화 CustomMsg) |
| Height mapper | `rubi_height_mapping/config/height_mapping.yaml`, `launch/mapping.launch.py` |
| Height mapper와 함께 쓰는 전역 누적 설정 | `rubi_height_mapping/config/global_heightmap.yaml` |
| wrapper 단독 실행 기본값 | `rubi_global_heightmap_wrapper/config/` |
| RUBI Height planner 비용·단차 한계 | `legged_bringup/config/rubi/planner.yaml` |
| RUBI Nav2/필터/속도 피드백 | `legged_bringup/config/rubi/nav2.yaml`, `legged_bringup/legged_bringup/rubi_navigation.py` |
| RUBI 재계획 BT | `legged_bringup/config/rubi/navigate.xml` |
| Generic 높이 planner | `legged_bringup/config/planner.yaml` |
| Generic Nav2 장애물·추종 | `legged_bringup/config/costmap.yaml`, `obstacle.yaml`, `nav2.yaml` |
| 지도 비교·review 전용 한계/비용 | `legged_bringup/launch/rubi_height_navigation.launch.py` 실행 인자 |
| 사용할 PCD·2D·height 지도 | workspace `maps/<지도버전>/site.yaml` |

같은 기능에 설정이 둘 있는 경우 실제로 실행한 launch가 넘긴 YAML이 우선입니다. 예를 들어 `rubi_height_mapping mapping.launch.py`는 mapper 패키지의 전역 누적 설정을 wrapper에 넘깁니다.

## 소스와 데이터 배치 원칙

- `src/`: 패키지 소스·launch·기본 설정·테스트·기존 테스트 fixture.
- `maps/raw/`: 원본 높이 지도. 덮어쓰지 않습니다.
- `maps/<버전>/`: 배포/검토할 지도·manifest·변경 이력. 추론 셀은 `provenance.npz`로 구분합니다.
- `tools/`: workspace 환경 설정과 재현 도구. 지도별 일회성 작업은 해당 지도 `audit/`에 둡니다.
- `validation/`: 검증 자료. `build/`, `install/`, `log/`는 생성물이며 소스가 아닙니다.
- `.deps*`, `.venv-height`, `.sdk`, `third_party/Livox-SDK2`: 로컬 의존성과 SDK. 패키지 정리를 이유로 삭제하지 않습니다.

`__pycache__`, `*.pyc`는 기존 `.gitignore`로 제외되어 있습니다. RUBI 설정은 `config/rubi/`에 묶고, BODY 높이·거리 필터 및 속도 변환은 `legged_navigation`에 통합했습니다. generic 프로필과 기존 지도 fixture는 보존합니다. 변경 출처는 `config/rubi/baseline_sources.json`, 소스 백업은 workspace의 `backups/`에 있습니다.

## 실행 문서

- [설치와 실물 localization](../docs/INSTALL.md)
- [실제 bag → localization → Height Nav2 영상 실행](../docs/HEIGHT_REPLAY.md)
- [Height 지도와 Nav2 검토](../docs/HEIGHT_NAVIGATION.md)
- [일반 운영](../docs/OPERATIONS.md)
- [새 바닥 보간 지도 v4](../archive/20261109/maps/rubi_1f_height_20261109_floorcomplete_v4/REPORT.md)

지도 생성에는 `source ~/legged_robot_ws/tools/height_env.sh`, 저장된 높이 지도 검토와 bag 기반 navigation에는 `source ~/legged_robot_ws/tools/navigation_env.sh`를 사용합니다. 두 환경 스크립트의 역할은 다릅니다. `navigation_real`의 overlay를 같은 터미널에 겹쳐 source하지 않습니다.
