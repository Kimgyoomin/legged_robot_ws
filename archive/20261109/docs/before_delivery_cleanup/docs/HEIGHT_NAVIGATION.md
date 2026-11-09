# RUBI height 지도와 Nav2

현재 실행 절차와 성공한 navigation_real 설정의 이식 내역은 [HEIGHT_REPLAY.md](HEIGHT_REPLAY.md)에 모았다.

| 모드 | 시작/자세 입력 | 실행 범위 | 명령 출력 |
|---|---|---|---|
| `review` | RViz에서 임의 Start/Goal | Frozen height + 정적 costmap + HeightPlanner | 없음 |
| `replay` | bag LiDAR/IMU → FAST-LIO + 초기 위치 | Localization + 실시간 두 costmap + HeightPlanner + DWB | `/legged/replay/cmd_vel` |
| `nav2` | 별도 실행한 실시간 localization | 두 costmap + HeightPlanner + DWB | `/cmd_vel` |

기본 지도는 `maps/rubi_1f_height_20261109_wall_gap_v6/site.yaml`이다. 지도 데이터는 이번 코드 연동에서 변경하지 않았다. 동일 site를 사용하더라도 review에는 센서 장애물 레이어가 없고, replay/nav2에는 있으므로 경로가 달라질 수 있다.

## 기존 지도 검토 실행

```bash
cd ~/legged_robot_ws
source tools/navigation_env.sh
export ROS_DOMAIN_ID=186
export ROS_LOCALHOST_ONLY=0
ros2 launch legged_bringup rubi_height_navigation.launch.py \
  mode:=review \
  site:="$PWD/maps/rubi_1f_height_20261109_wall_gap_v6/site.yaml" \
  review_planning_time_ms:=5000 \
  review_height_limit_m:=0.11 \
  review_height_cost:=geometry
```

`2D Pose Estimate` → 시작점, `2D Goal Pose` → 경로 목적지다. Move Camera/Orbit으로 3D 회전한다. 실제 bag 연동 영상은 [replay 절차](HEIGHT_REPLAY.md)를 사용한다.

## 설정의 위치

- RUBI DWB/footprint/costmap/관측 필터: `src/legged_bringup/config/rubi/nav2.yaml`
- RUBI 높이 비용/단차 한도/clearance: `src/legged_bringup/config/rubi/planner.yaml`
- RUBI 1Hz 재계획 BT: `src/legged_bringup/config/rubi/navigate.xml`
- 재생 모드: 모든 `use_sim_time=true`, raw 3-topic 입력, replay 전용 명령 출력.
- `config/nav2.yaml`, `config/planner.yaml` 등 기존 generic 설정은 다른 `navigate.launch.py`에서 사용하는 별도 프로필이다.

## 지도 교체

```bash
python3 tools/replace_site_height.py \
  --site maps/rubi_1f_height_20261109_wall_gap_v6/site.yaml \
  --height /절대경로/새로운/map.rghm \
  --output /절대경로/새로운/site.yaml
```

새 site를 launch 인자로 넘기고 재시작한다. 원본 RGHM, 추론 마스크, 단차와 구조물은 보존한다. 좌표계/격자/원점이 달라지면 별도 정합 검증이 필요하다.

## 비용과 통과 판정

현재 RUBI 기본은 11cm jump/relief 한도와 geometry 비용이다. 인접 셀 높이 차이에 기반한 비용을 사용하며, unknown/높이 제한/clearance는 비용과 별개의 통과 조건이다. 높이 비용이 낮아도 금지 조건을 통과하지 못하면 우회한다. 29cm 단차는 허용하지 않는다.

전역 중심 clearance는 0.30m다. 이전 0.45m는 시작 방의 약 0.8m 출입구 연결을 끊었다. DWB의 0.55m 사각 footprint 검사와 구분한다. 전역 경로가 생겼다는 사실만으로 실제 몸체의 회전이나 단차 보행을 검증한 것은 아니다.

[초기 지도 비교·비용 상세 기록](../archive/20261109/docs/history/HEIGHT_NAVIGATION_20261109_initial.md), [기존 검증 자료](../archive/20261109/validation/height_navigation_20261109), [현재 bag 연동 검증](../archive/20261109/validation/rubi_replay_20261109)을 함께 참고한다.
