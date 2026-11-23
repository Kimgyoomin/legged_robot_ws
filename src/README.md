# 활성 패키지와 직접 실행 진입점

`colcon build --base-paths src` 기준 10개 패키지입니다.

| 폴더 / ROS 패키지 | 역할 / 주 실행 파일 |
|---|---|
| `navigation_dds` | XML profile, `ros2 run navigation_dds with_profile` |
| `livox_ros_driver2` | `msg_MID360s_launch.py`: CustomMsg + PC2 + IMU, RViz 없음 |
| `FAST_LIO_ROS2` / `fast_lio` | `mapping.launch.py`: SLAM, PCD 저장 |
| `FAST_LIO_LOCALIZATION2` / `fast_lio_localization` | `localization_nav_rubi.launch.py`: 저장 PCD 정합·속도·장착 TF |
| `rubi_height_mapping` | `mapping.launch.py`: CUDA 높이 추정 + global 누적 |
| `rubi_global_heightmap_wrapper` | RGHM 저장, `frozen_heightmap_node`: 저장 파일 발행 |
| `rubi_heightmap_postprocess` | 오프라인 height 후처리·수동 편집 |
| `rubi_heightmap_step_wavefront_planner` | 공유 StepGrid A*, height 평가, frozen cache, height-only core |
| `legged_navigation` | `height_hybrid_navigation.launch.py`, `height_only_navigation.launch.py`, Nav2 plugins/DWB 연결 |
| `legged_bringup` | 기존 site/replay/비교 실행 호환용 |

[설치](../docs/INSTALL.md) · [실물 운용](../docs/OPERATIONS.md) · [DDS](../docs/DDS.md) · [튜닝](../docs/TUNING.md)

`archive/`는 과거 실험·대안 소스·test fixture입니다. 일부 테스트가 fixture를 참조하므로 임의로 전체 삭제하지 않습니다. 새 기본 실행은 archive의 package를 빌드하지 않습니다. FAST-LIO SLAM의 이식 출처는 [FAST_LIO_ROS2/DELIVERY.md](FAST_LIO_ROS2/DELIVERY.md)에 기록했습니다.
