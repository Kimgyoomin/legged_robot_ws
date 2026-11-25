# 납품 구성 검증 기록 — 2026-11-23

실물 센서를 재시작하거나 로봇을 주행시키지 않고 수행한 검증입니다. 장비의 실제 환경 버전은 [INSTALL.md](INSTALL.md)에 기록했습니다.

| 항목 | 결과 |
|---|---|
| Release 빌드 | `fast_lio`, `rubi_global_heightmap_wrapper`, `rubi_height_mapping`, planner core, `legged_navigation`, `navigation_dds` 성공 |
| Height-only core | 관측 셀 정렬·평지 거리 비용·높이 비용·높은 단차 거부·unknown 차단·우회·warm 취소 통과 |
| 기존 planner | Grid A*, frozen cache, local relief, core smoke 회귀 통과 |
| Local 주행 | sector/height 필터, FAST-LIO 속도 변환, 기존 Pure Pursuit 회귀 통과 |
| PCD 저장 | 빈 지도 실패, 정상 PCD 재읽기, 덮어쓰기 거부/허용 테스트 통과 |
| SLAM native launch | 센서 없는 별도 domain에서 시작, `/map_save`가 빈 지도를 `success=false`로 반환함을 확인 |
| DDS | common/large XML, scoped 상속, 기존/새 직접 launch의 native profile 적용 검증 |
| 2D 지도 도구 | 상대 지면 높이·unknown·좌표축 테스트와 PCD→YAML/PGM CLI 실행 통과 |
| 직접 launch 인자 | driver, SLAM, localization, mapper, hybrid, height-only의 `--show-args` 성공 |
| CUDA 환경 | 현재 장비의 CuPy import 및 `check_environment` GPU 커널 검사 성공 |

관련 C++ gtest 69개와 core/global-grid smoke를 실행했습니다. DDS 및 2D 투영 Python 테스트는 19개 통과했습니다. 기존 DDS 테스트는 RViz 및 replay-localization 선택을 명시하여 해당 launch 조합을 검증하도록 갱신했습니다.

## Nav2 전체 연결

`ROS_DOMAIN_ID=227`, `ROS_LOCALHOST_ONLY=1`, `dds_profile:=default`로 실제 로봇과 분리했습니다. 합성 TF, LiDAR, FAST-LIO 속도, 5 cm 해상도의 120×80 frozen 높이 지도를 사용했습니다. `/cmd_vel`은 `/delivery_test/cmd_vel`로 바꿨습니다.

두 구성 각각 다음을 통과했습니다:

1. planner/controller/behavior/BT lifecycle 활성화.
2. 저장한 RGHM 읽기 및 frozen terrain READY.
3. `ComputePathToPose` 성공, 21개 pose 생성.
4. 실제 local LiDAR 필터 출력 수신.
5. `NavigateToPose` → DWB 양의 전진 명령 → goal 취소 완료.
6. 미관측/지도 밖 goal 거부.
7. 잘못된 높이 파일·해상도 거부.

Height-only는 `/map` 및 map_server 없이 실행됩니다. Hybrid는 map_server와 static/live/inflation global costmap을 포함합니다. 두 integration CTest가 통과했습니다. 소형 합성 맵의 실행 시간은 실물 대형 지도 성능을 뜻하지 않습니다.

## 다시 실행하는 방법

필요한 의존성을 먼저 빌드한 workspace에서:

```bash
cd ~/legged_robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2 \
  colcon build --base-paths src --packages-select legged_navigation \
  --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_TESTING=ON -DLEGGED_TEST_NAV2_INTEGRATION=ON
source install/setup.bash
ctest --test-dir build/legged_navigation --output-on-failure -R delivery_nav2

ROS_LOG_DIR=/tmp/legged_dds_test_logs PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  python3 -m pytest -q src/navigation_dds/test/test_profiles.py tools/test/test_pcd_to_occupancy.py
```

Integration은 테스트용 domain 227을 사용하며 동시에 실행하지 않습니다. 기본 빌드에서는 통합 테스트가 OFF입니다. 센서 driver나 로봇 bridge는 시작하지 않습니다. `/tmp/legged_nav2_*`에 합성 지도와 launch 로그를 남깁니다.

## 현장에서 확인할 범위

이 검증에는 실제 LiDAR로 새 PCD/Height Map을 취득하는 과정, 현장 localization 정합, 실제 로봇의 footprint·속도·정지 동작 및 다중 PC UDP 통신은 포함하지 않았습니다. 실물에서는 [OPERATIONS.md](OPERATIONS.md)의 순서로 해당 장소의 지도와 장착값을 확인합니다. 기존 로봇 `/cmd_vel` bridge는 별도로 실행되어 있어야 합니다.

## 지정 bag으로 직접 패키지 연결 확인 — 2026-11-23

대상은 `~/rosbag_record/[26.11.19]rubi_pointfoot_nav`입니다.

- GitHub 기본 브랜치 `main`과 원격 head `352eb3345e24fbada0b51c11d06d545beb98cf81`을 확인했습니다. 이 commit에는 소스 변경이 있으며, 당시 README/HTML 및 추가 운용 문서는 로컬 미커밋 상태였습니다.
- Bag: 555.684초, CustomMsg LiDAR 5,557개, PC2 2,779개, IMU 40,597개.
- 현재 localization PCD와 bag 첫 `/cloud_pcd`의 XYZ 565,765개를 비교했으며 순서와 값이 모두 일치했습니다.
- 별도 domain 228에서 native `dds_profile:=navigation`, 실물과 같은 CPU affinity, `use_sim_time:=true`, `--rate 1.0`으로 센서 세 토픽만 재생했습니다.
- `fast_lio_localization localization_nav_rubi.launch.py`를 직접 실행하고 초기 pose를 지정했습니다. `/localization` 출력만으로 정합을 판단하지 않고 `/map_to_odom` 갱신을 확인한 뒤 `rubi_height_mapping mapping.launch.py`를 시작했습니다.
- Sensor time 40.124초 동안 odometry/body cloud 각각 399개, 전역 정합 출력 21개, local GridMap 184개를 관측했습니다. 마지막으로 관측한 전역 surface는 5093점이며 save service가 성공했습니다.
- 임시 결과는 `/tmp/legged_direct_replay_20261123/replay_check/`에 있습니다. 이 부분 지도는 운용 지도에 덮어쓰지 않았습니다.
- Driver·Nav2·구동 bridge는 이 실제-bag 연결 검사에서 시작하지 않았습니다. 9분 16초 전체 구간과 실제 주행은 이번 검사 범위가 아닙니다. Nav2 연결 자체는 위 합성 integration 결과를 참고합니다.

## RViz 통합 및 재생 오류 점검 — 2026-11-23

- `fast_lio_localization/rviz/fastlio_localization.rviz`에 Nav2 panel/goal, frozen height, 선택 가능한 live height를 통합했습니다. 같은 창에서 경로와 local/global costmap을 봅니다. 표시 주기는 15 FPS입니다.
- 두 `legged_navigation` launch의 기본값은 `rviz:=false`입니다. 명시적으로 true를 주면 기존 별도 창을 열 수 있습니다. 실물·bag 운용 문서와 HTML은 localization 한 창을 사용하고 mapper/navigation 추가 창을 끄도록 수정했습니다.
- DDS launch graph 검사 및 standalone localization 검사 **29개 통과**. Navigation의 기본 RViz 0개와 true 지정 시 1개를 검사했으며 실제 노드는 시작하지 않았습니다. 세 launch의 `--show-args`, RViz YAML/QoS/goal 구성, 문서 Bash 문법과 whitespace 검사도 통과했습니다. 변경된 GUI를 직접 조작한 주행 검증은 하지 않았습니다.
- 사용자의 최근 로그에서 terrain READY(889.9 ms)와 lifecycle 활성화를 확인했습니다. 이후 localization 정합 실패 및 stale gravity/velocity 입력 폐기가 반복됐습니다. 추가 RViz의 `exit code -11`은 Ctrl+C 종료 과정에서 발생했습니다.
- 원본 bag의 IMU stamp 전체 조사: 100 ms 초과 공백 1,149개, 500 ms 초과 공백 16개, 최대 1.019894초(재생 약 139.64초 지점). 재계산 실패의 의심 원인이며 정확한 기여도는 분리 검증하지 않았습니다. 한 RViz만 실행 중인 재시도에서도 localization fitness 0.0이 반복되어, RViz 수만으로 전체 오류를 설명할 수 없습니다.
- 40초 연결 검사는 IMU 최대 공백보다 앞선 구간입니다. 전체 bag 재계산 성공을 보증하지 않으며, 원본 bag과 운용 Height Map은 수정하지 않았습니다.

## 3차 CoT와 운용 YAML

기록된 hybrid 2D costmap에서 원본 V6와 +Y 0.50m 지도를 같은 시작·goal로 계산했다. 두 CoT 경로의 1,724개 셀이 모두 일치했다. 격리된 ROS domain에서 hybrid와 height-only 각각 ComputePathToPose, DWB 명령, goal 취소를 검증했다. 다른 domain에서 localization 출력과 CUDA mapper의 입출력 토픽을 YAML로 바꿔 실제 발행자·구독자의 연결을 확인했다. [단차 비용](STEP_COST_TUNING.md) · [현재 운용 지도](../maps/README.md)
