# 패키지 구조

2026-11-09 현재 `src`에는 9개 ROS 패키지가 있습니다. 기존 센서 알고리즘·드라이버는 이번 캐시 최적화에서 변경하지 않았습니다.

| 폴더 / ROS 패키지 | 역할 |
|---|---|
| `navigation_dds` | SHM/UDP 프로필과 launch/CLI wrapper |
| `livox_ros_driver2` | 수정된 MID360S 드라이버, LiDAR/IMU 장착 회전·시간 처리 |
| `FAST_LIO_LOCALIZATION2` / `fast_lio_localization` | **PCD 기반 localization**, velocity·중력 출력 |
| `rubi_height_mapping` | CuPy 높이 지도 생성 |
| `rubi_global_heightmap_wrapper` | 전역 누적·원본 RGHM 저장 |
| `rubi_heightmap_postprocess` | 오프라인 후처리·수동 편집 |
| `rubi_heightmap_step_wavefront_planner` | 높이 Grid A*, frozen terrain cache, 단위 테스트 |
| `legged_navigation` | Nav2 HeightPlanner adapter, BODY/sector 필터, 속도 변환, 기존 추종 비교 도구 |
| `legged_bringup` | 모듈별 launch, 설정, frozen map, RViz |

`fast_lio` 단독 mapping과 GenZ-ICP 대안은 [archive/20261109/optional_src](../archive/20261109/optional_src)에 원본 그대로 보관했습니다. 기본 빌드 의존성에서만 제외했습니다. 기존 테스트 fixture도 [archive/20261109/maps/demo](../archive/20261109/maps/demo)에 보존하고 테스트 경로를 갱신했습니다.

## 현재 실행 모듈

- `legged_bringup/rubi_localization.launch.py` → `fast_lio_localization/localization_nav_rubi.launch.py`.
- `legged_bringup/rubi_height_navigation.launch.py mode:=replay` → frozen map + costmap + HeightPlanner + DWB. Localization은 별도 실행.
- `mode:=review`: 정적 지도 경로만 계산. `mode:=nav2`: 실물 입력 사용.
- 기존 generic `localize.launch.py`, `navigate.launch.py`는 다른 비교 구성입니다. 대안 소스가 필요한 launch는 archive에서 복원·빌드한 뒤 사용합니다.

## 설정 위치

| 목적 | 파일 |
|---|---|
| Frozen planner / 800ms budget / 11cm 제한 | `legged_bringup/config/rubi/planner.yaml` |
| DWB·footprint·BODY 필터·costmap | `legged_bringup/config/rubi/nav2.yaml` |
| 재계획 BT | `legged_bringup/config/rubi/navigate.xml` |
| 센서 IP/출력 | `livox_ros_driver2/config/`, `launch_ROS2/` |
| Localization 입력 | `FAST_LIO_LOCALIZATION2/config/mid360s_nav.yaml`(PointCloud2), `rubi_custommsg.yaml`(bag CustomMsg) |
| 높이 생성 | `rubi_height_mapping/config/height_mapping.yaml`, `global_heightmap.yaml` |
| 지도 세트 | workspace `maps/rubi_1f_height_20261109_wall_gap_v6/site.yaml` |

[분리 실행](../docs/HEIGHT_REPLAY.md) · [캐시 설계와 성능](../docs/FROZEN_PLANNER.md) · [이동 manifest](../archive/20261109/move_manifest.json)
