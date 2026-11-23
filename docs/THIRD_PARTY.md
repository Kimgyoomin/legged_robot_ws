# 소스와 변경 출처

이 workspace는 서로 다른 upstream과 사용자 소스를 묶은 배포본입니다. **전체를 단일 MIT 라이선스의 신규 구현으로 표기하지 않습니다.** 각 원본 LICENSE와 source notice를 보존했습니다.

| 구성 | 기준 소스 | 배포본 변경 |
|---|---|---|
| Height / global wrapper / postprocess | `rubi_mapping_ws/src/rubi_terrain_mapping` working tree, repository HEAD `58d1e98…` | 기존 높이 알고리즘 유지; 공개 save directory 인자 추가 |
| GPU 추정기의 원 upstream | `iit-DLSLab/elevation_mapping_gpu_ros2` `e08063fe…` | 과거 height-only CuPy 분리·축/pose adapter 수정본 유지 |
| GPU snapshot finalization | `leggedrobotics/elevation_mapping_cupy` `v2.2.0`, `f618dd8…`, MIT | 셀별 확정 kernel 이식; RUBI ray 경로 유지·FP64 후보 합산 |
| Planner / FAST-LIO / GenZ | `navigation_real` working tree, HEAD `34894e17…` | 시간 decoder, Nav2 core adapter, 공개 config/launch, persistence 입력 연결 |
| 실제 Jetson 설정 참조 | `navigation_real` `jetson/260929`, `cb2cdf37…` | 장착 geometry와 message contract 비교; 이 오래된 branch 전체를 그대로 배포하지 않음 |
| Livox driver | 공식 `21445540f0d100dc86a7e6df312dd70bbdb4afdf`, v1.2.8 | 센서별 IMU 회전, 기본 identity 오타 수정, 새 workspace SDK 연결 |
| Livox SDK2 | 공식 `c0796f04c143143899c87a773d9f6b7136453c0b` | samples 기본 비활성; `.sdk` 별도 설치 |
| Pure Pursuit | 기존 `pongbot_navigation` source | core 복사; stale TF 검사; 새 package에서 빌드 |
| Nav2 | 시스템 ROS Humble package | upstream controller/costmap/Collision Monitor 코드는 수정하지 않음 |

전체 commit/path 기록은 [sources.json](sources.json), 수정 전 복사 파일의 SHA-256은 [copied_sources.sha256.json](copied_sources.sha256.json)에 있습니다. Working tree에는 기존 미커밋 수정이 포함되므로 HEAD 하나만으로 source를 완전히 특정하지 않습니다. 최종 배포 파일 hash는 `delivery.sha256.json`에 기록합니다.

## 라이선스 파일

- [GPU mapper MIT notice](../src/rubi_height_mapping/LICENSE)와 [UPSTREAM](../src/rubi_height_mapping/UPSTREAM.md)
- [Global wrapper MIT notice](../src/rubi_global_heightmap_wrapper/LICENSE)
- [Offline postprocess MIT notice](../src/rubi_heightmap_postprocess/LICENSE)
- [Livox driver MIT](../src/livox_ros_driver2/LICENSE.txt), SDK의 [원본 라이선스](../third_party/Livox-SDK2/LICENSE.txt)
- [GenZ 원본 라이선스](../archive/20261109/optional_src/genz-icp/LICENSE)
- [FAST-LIO localization LICENSE](../src/FAST_LIO_LOCALIZATION2/LICENSE), [FAST-LIO mapping LICENSE](../archive/20261109/optional_src/FAST_LIO_ROS2/LICENSE)
- Planner/Pure Pursuit는 원 package.xml의 Apache-2.0 표기와 source 출처를 유지합니다. 신규 adapter package에는 [Apache-2.0 전문](../src/legged_navigation/LICENSE)을 동봉했습니다.

FAST-LIO 두 폴더는 `package.xml`에 BSD라고 적혀 있으나 동봉된 LICENSE는 GPL v2입니다. 이 불일치를 숨기거나 임의로 단일 permissive 라이선스로 바꾸지 않았습니다. 실제 소스·바이너리 배포 범위를 정할 때 해당 구성요소의 원 upstream notice를 기준으로 정리해야 합니다.

## 변경 요약

1. PointCloud2 점별 절대 ns를 읽는 FAST-LIO parser와 regression test 추가.
2. 빈 scan/clock 역행 시 scan·time queue 일관성, 선택적 time-sync 단위 오류 수정.
3. FAST-LIO 전역 pose의 센서 시각 보존 및 전역 정합 전 임시 identity 발행 방지.
4. Python 전역 정합의 `ros2_numpy` 의존성을 ROS `sensor_msgs_py`로 대체.
5. Livox SDK와 driver를 MID-360S 지원 버전으로 고정하고 IMU 회전 port.
6. 동일 Grid A* core의 Nav2 adapter, `static_heightmap` 계약과 standalone core export 정리.
7. 장애물 costmap·재계획·Collision Monitor·freshness watchdog을 두 모드에 연결.
8. CPU frozen server, 지도 정렬·재격자화 및 version bundle 도구 추가.
9. Config는 `legged_bringup/config`에서 짧은 이름으로 제공. 내부 upstream 이름은 비교 가능하도록 보존.

원본 reference profile의 controller 값과 CoT weight가 동봉된 regression contract와 달랐던 부분은 **복사본에서만** reference 값으로 맞췄습니다. 실제 공개 실행은 별도의 `pursuit.yaml`(최대 0.3 m/s)과 `planner.yaml`을 사용합니다.

미디어와 실험 산출물은 runtime 소스로 취급하지 않습니다. 큰 upstream 예시 영상·이미지는 [omitted_media.txt](omitted_media.txt)에 기록하고 생략했습니다. `.git`, 기존 `build/install/log`, 개인 실험 PCD도 복사 배포 대상에서 제외했습니다. 사용한 모델 weights와 필요한 라이선스는 포함했습니다.

## 핵심 patch

- [Livox driver diff](patches/livox_driver.diff)
- [SDK build diff](patches/livox_sdk.diff)
- 새 `imu_rotation.hpp`는 source에 포함되어 있습니다. Diff의 신규 untracked 파일이 자동 포함되지 않는 Git 특성을 고려하여 최종 source 자체와 manifest를 기준으로 배포합니다.
