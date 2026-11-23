# Frozen height 지도 검토와 실물 입력

## 세 모드

| 모드 | 위치 입력 | 실행 범위 | 명령 출력 |
|---|---|---|---|
| review | 임의 Start/Goal | 정적 지도 + HeightPlanner | 없음 |
| replay | 별도 FAST-LIO Localization + bag | 실시간 costmap + HeightPlanner + DWB | `/legged/replay/cmd_vel` |
| nav2 | 별도 FAST-LIO Localization + 센서 | 실시간 costmap + HeightPlanner + DWB | `/cmd_vel` |

PCD localization은 `fast_lio_localization`입니다. 설치와 실물 Driver → Localization → Navigation 전체 명령은 [README](../README.md#live-livox), bag 실행은 [HEIGHT_REPLAY.md](HEIGHT_REPLAY.md)를 참고하세요.

## 지도만 검토

```bash
cd ~/legged_robot_ws
source tools/navigation_env.sh
export ROS_DOMAIN_ID=87
ros2 launch legged_bringup rubi_height_navigation.launch.py \
  mode:=review \
  review_planning_time_ms:=800 \
  review_height_limit_m:=0.11 \
  review_height_cost:=geometry
```

`Frozen terrain READY` 후 `2D Pose Estimate`로 Start, `2D Goal Pose`로 목적지를 찍습니다. Move Camera/Orbit으로 3D 회전합니다. 기본 최종 지도는 `maps/rubi_1f_height_20261109_wall_gap_v6/site.yaml`입니다. review는 센서 obstacle layer가 없어 실제 replay/nav2 경로와 다를 수 있습니다.

## 실물 입력으로 전환

센서 메시지 형식을 먼저 확인합니다. legged MID360S 기본 `/livox/lidar`가 PointCloud2인 구성은 다음과 같습니다. 별도 터미널에서 공통 환경과 실제 domain을 맞춥니다.

```bash
# Localization: PCD 기반 FAST-LIO Localization
FASTLIO_DIAG_TIMING=1 taskset -c 2,3,4,5 \
  ros2 launch legged_bringup rubi_localization.launch.py \
  input_format:=pointcloud2 dds_profile:=navigation use_sim_time:=false use_rviz:=true

# Localization RViz에서 2D Pose Estimate → 정합 확인 후 Navigation: 다른 터미널
taskset -c 6,7 \
  ros2 launch legged_bringup rubi_height_navigation.launch.py \
  mode:=nav2 dds_profile:=navigation use_sim_time:=false use_fastlio_velocity:=true rviz:=false \
  cloud_topic:=/livox/lidar
```

navigation_real처럼 CustomMsg/PointCloud2 이중 발행 구성은 localization에 `input_format:=custommsg`, navigation에 `cloud_topic:=/livox/lidar_pointcloud`를 사용합니다. 이름만 보고 타입을 바꾸지 않습니다. 실제 장착 TF·지도 정합과 BODY 필터의 높이 하한을 확인해야 합니다. LCM/구동기 노드는 여기서 자동 실행하지 않습니다.

Localization이 기존 navigation_real 형태의 RViz를 먼저 엽니다. Navigation은 같은 창에 frozen global height 및 local/global costmap·경로를 공급하므로 두 번째 RViz가 필요하지 않습니다. 고정 height 셀은 지형 판단에 재사용하며, **동적 장애물은 local/global obstacle layer로 계속 갱신하고 최신 global costmap으로 재계획**합니다. 고정 height의 변경과 동적 장애물 갱신은 별개입니다.

기존 `ros2_ws` Livox driver를 그대로 쓰려면 **driver 터미널만** 기존 환경에서 실행할 수 있습니다. Localization·Height Navigation 터미널은 새 터미널에서 `source ~/legged_robot_ws/tools/navigation_env.sh`를 실행하세요. 같은 domain을 사용하되 기존 FAST-LIO Localization/Navigation과 중복 실행하지 않습니다. `legged_robot_ws`의 기본 MID360S driver는 `/livox/lidar` PointCloud2 구성이므로 기존 CustomMsg driver 명령/형식을 그대로 가정하지 않습니다.

## 지도와 파라미터 교체

```bash
python3 tools/replace_site_height.py \
  --site maps/rubi_1f_height_20261109_wall_gap_v6/site.yaml \
  --height /절대경로/새로운/map.rghm \
  --output /절대경로/새로운/site.yaml
```

새 site를 두 launch의 `site:=...`에 동일하게 넘깁니다. 다른 좌표계는 먼저 정합합니다. 수정한 지도는 새 버전으로 저장하고 원본/추론 마스크를 보존합니다.

- `src/legged_bringup/config/rubi/planner.yaml`: 높이 한도·geometry/CoT·clearance·캐시.
- `src/legged_bringup/config/rubi/nav2.yaml`: DWB·footprint·costmap·BODY 필터·속도 입력.
- `src/legged_bringup/config/rubi/navigate.xml`: 1Hz RateController 재계획 설정.

현재 planner 파라미터는 configure 시 읽는 snapshot입니다. 실행 중 `Height.*` 변경은 거절하며, YAML 수정 후 재시작 또는 lifecycle cleanup/configure로 다시 적용합니다. 높이/격자/파라미터가 바뀌면 캐시를 다시 준비합니다. **매 경로 요청마다 전체 높이 지도를 다시 계산하지 않습니다.**

11cm는 jump/relief 통과 상한이며 29cm 단차를 허용하지 않습니다. geometry 비용은 지형 높이 변화에 근거합니다. unknown·높이 증거 부족·clearance 위반은 비용과 별도로 차단합니다. 전역 중심 clearance0.30m는 DWB의 회전 사각 footprint 검사와 다릅니다.
