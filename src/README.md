# 활성 패키지와 직접 실행 진입점

`colcon build --base-paths src` 기준 11개 패키지입니다. 실물에서 열 터미널과 패키지별 명령은 루트 README의 [기존 지도로 주행](../README.md#기존-지도로-주행)과 [새 지도 생성](../README.md#새-지도를-만들-때)에 있습니다.

| 폴더 / ROS 패키지 | 역할 / 주 실행 파일 |
|---|---|
| `navigation_dds` | 각 launch가 운용 YAML과 DDS 설정을 읽을 때 사용합니다. 개별 CLI에 같은 설정을 적용하는 예는 `ros2 run navigation_dds with_profile common ros2 topic list`입니다. |
| `livox_ros_driver2` | `msg_MID360s_launch.py`: CustomMsg + PC2 + IMU, RViz 없음 |
| `FAST_LIO_ROS2` / `fast_lio` | `mapping.launch.py`: SLAM, PCD 저장 |
| `FAST_LIO_LOCALIZATION2` / `fast_lio_localization` | `localization_nav_rubi.launch.py`: 저장 PCD 정합·속도·장착 TF |
| `rubi_height_mapping` | `mapping.launch.py`가 위치 추정의 `/cloud_registered_body`로 로컬 높이 격자를 생성하고 전역 누적 노드도 시작합니다. |
| `rubi_global_heightmap_wrapper` | 매핑 중 `/rubi/global_elevation/save_map`으로 RGHM을 저장합니다. Nav2를 시작하면 `frozen_heightmap_node`가 저장 파일을 읽어 `/legged/height`로 발행합니다. [서비스와 파일 형식](rubi_global_heightmap_wrapper/README.md) |
| `rubi_heightmap_postprocess` | 저장된 지도를 오프라인에서 비교·편집합니다. 공통 터미널 환경에서 `ros2 run rubi_heightmap_postprocess compare --input "$LEGGED_WS/maps/rubi_1f/map.rghm" --output "$LEGGED_WS/maps/rubi_1f_review"`를 실행합니다. 출력은 새 폴더여야 합니다. [후처리 옵션](rubi_heightmap_postprocess/README.md) |
| `rubi_heightmap_step_wavefront_planner` | 두 Nav2 모드가 사용하는 StepGrid A*와 지형 평가를 제공합니다. 일반 주행에서는 `legged_navigation`이 이 코드를 사용합니다. [탐색 캐시](../docs/FROZEN_PLANNER.md) |
| `legged_navigation` | `height_hybrid_navigation.launch.py`, `height_only_navigation.launch.py`, Nav2 plugins/DWB 연결 |
| `pongbot_lcm` | `joy_lcm.launch.py` · ROS Twist와 무선 조이스틱을 LCM CMD_VEL로 송신 |
| `legged_bringup` | 과거 호환용 소스. 현재 운용·재생 안내에서는 사용하지 않음 |

[설치](../docs/INSTALL.md) · [실물 운용](../docs/OPERATIONS.md) · [DDS](../docs/DDS.md) · [튜닝](../docs/TUNING.md)

`archive/`는 과거 실험·대안 소스·test fixture입니다. 일부 테스트가 fixture를 참조하므로 임의로 전체 삭제하지 않습니다. 새 기본 실행은 archive의 package를 빌드하지 않습니다. FAST-LIO SLAM의 이식 출처는 [FAST_LIO_ROS2/DELIVERY.md](FAST_LIO_ROS2/DELIVERY.md)에 기록했습니다.
