# legged_range_trial

RUBI의 방향별 근거리 필터와 Nav2 controller 비교를 위한 별도 실행 패키지다. 기존 `legged_navigation` 실행 파일과 HeightPlanner를 재사용하며, 실험 설정과 속도 어댑터를 분리한다.

**상세 인계 문서: [docs/navigation](../../docs/navigation/README.md)**

## 현재 실행 (2026-10-27)

Gazebo와 FAST-LIO localization이 같은 ROS domain에서 실행 중인 상태에서:

```bash
source /opt/ros/humble/setup.bash
source ~/legged_robot_ws/install/local_setup.bash
export ROS_DOMAIN_ID=45
ros2 launch legged_range_trial navigate.launch.py \
  site:=/home/kim/legged_robot_ws/maps/cost_wall_lower10_v3/site.yaml \
  use_sim_time:=true \
  cloud_topic:=/livox/lidar_PointCloud2 \
  controller:=rpp fastlio_velocity:=true estimate_velocity:=false
```

DWB 비교는 `controller:=dwb`로 바꾼다. 기존 navigation만 종료한 뒤 새 launch를 시작한다. 중복 명령을 막는 실행 잠금이 있다. Nav2 Goal로 목표를 지정한다.

## 설정 안내

| 파일 | 현재 핵심값 |
|---|---|
| [range.yaml](config/range.yaml) | 전방 ±60°/.3m, 나머지 1m, 최대5m; BODY 높이 −.20~1.5m |
| [range15.yaml](config/range15.yaml) | 나머지1.5m; 높이 −.35~1.5m이므로 거리만의 비교는 아님 |
| [costmap.yaml](config/costmap.yaml) | obstacle .3~4m; global inflation .6m/k=.1, local .4m/k=5 |
| [collision.yaml](config/collision.yaml) | Stop 0.55×0.55m; launch가 Nav2 footprint에도 적용 |
| [planner.yaml](config/planner.yaml) | 추가 costmap clearance0, terrain clearance .2m, jump .1m |
| [nav2.yaml](config/nav2.yaml) | DWB x −.2~.7m/s, max_speed_xy .7; failure/progress600초 |
| [rpp.yaml](config/rpp.yaml) | 공식 RPP .7m/s; 곡률 감속 반경1.2m, local 비용 감속 |
| [velocity.yaml](config/velocity.yaml) | pose 회귀 비교용 설정과 공통 출력 `/legged/controller_odom` |
| [navigate.xml](config/navigate.xml) | 1Hz 계획, 실패 시1초 지연 후 최대600회 재시도 |

`controller=dwb`, `fastlio_velocity=false`, `estimate_velocity=false`, `final_guard=false`가 launch 기본값이다. native velocity를 쓰려면 위 명령처럼 명시한다. native/pose 회귀 입력은 동시에 활성화할 수 없다.

`final_guard=false`는 추가 costmap swept 검사를 끄는 선택이다. **Collision Monitor Stop과 센서·odom·TF·명령 freshness 검사는 남는다.** `final_guard=true`로 복원할 수 있다. 기존 custom `mode=pursuit`는 Nav2 RPP가 아니며 final guard를 항상 사용한다.

## 필터만 미리 보기

```bash
ros2 launch legged_range_trial preview.launch.py use_sim_time:=true
```

RViz에서 `/range_trial/obstacles`, `/range_trial/collision_points`를 본다. 이 preview는 속도 명령을 만들지 않고 기존 navigation 입력을 바꾸지 않는다. 1.5m 비교는 `filter:=/absolute/path/range15.yaml`로 선택한다.

## 구현과 검증 범위

- 필터는 driver가 아닌 C++ navigation 전처리다. 센서 원점 3D 거리와 base 축 방향을 사용하며 원시 localization 입력은 유지한다.
- HeightPlanner는 Nav2 costmap과 frozen height를 읽어 Grid A*를 실행한다. YAML의 `tracking_refiner`는 현재 Nav2 plugin에서 사용하지 않는다.
- FAST-LIO 내부 추정 속도는 별도 topic에 발행하고, adapter가 body→base_link 원점·축을 보정한다. `/cmd_vel`을 측정 속도처럼 사용하지 않는다.
- RPP 연결/설정은 준비됐으나 동일 조건의 DWB 대비 완주율 개선은 아직 검증하지 않았다. 전체 gait 외곽·Jetson 실측도 남아 있다.
- 과거 초기 시험은 footprint .70×.65m, clearance .45m, 높이 하한 −.35m였다. 현재와 혼동하지 않도록 [보고서 색인](../../docs/navigation/06-validation.md)에 당시 기록을 모았다.

[운영·점검 명령](../../docs/navigation/05-operation.md) · [필터 상세](../../docs/navigation/02-obstacle-filter.md) · [RPP/DWB](../../docs/navigation/03-controllers.md) · [FAST-LIO 속도](../../docs/navigation/04-fastlio-velocity.md)
