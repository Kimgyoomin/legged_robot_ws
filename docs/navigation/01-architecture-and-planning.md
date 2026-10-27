# 01. 전체 구조와 높이 기반 전역 계획

[문서 처음으로](README.md)

## 데이터 흐름

```mermaid
flowchart TD
    LiDAR[Livox raw cloud / IMU] --> LIO[FAST-LIO + localization]
    LIO --> TF[map → camera_init → body → BODY → base_link]
    LIO --> V[Native velocity → base_link adapter]
    V --> Controller[Nav2 RPP 또는 DWB]
    LiDAR --> Filter[방향별 거리 + BODY 높이 필터]
    Filter --> Obs[/legged/obstacles · 원래 sensor frame]
    Filter --> CP[/legged/collision_points · base_link]
    Map[저장된 occupancy] --> Global[Global costmap]
    Map --> Local[Local costmap]
    Obs --> Global
    Obs --> Local
    Height[저장된 height.rghm → /legged/height] --> Planner[HeightPlanner / Grid A* + CoT]
    Global --> Planner
    Planner --> Path[Global path]
    Path --> Controller
    Local --> Controller
    Controller --> Raw[/legged/cmd_raw]
    Raw --> CM[Collision Monitor]
    CP --> CM
    CM --> Safe[/legged/cmd_safe]
    Safe --> Watch[Freshness watchdog]
    Watch --> Cmd[/cmd_vel → 보행 정책]
```

`final_guard:=true`이면 마지막 watchdog에서 costmap 기반 footprint 이동 충돌 재검사·감속도 수행한다. 기본 trial Nav2는 이 추가 검사만 제외한다. 원시 cloud와 localization 입력은 navigation 필터와 분리되어 있다.

## 코드 위치와 소유 범위

| 역할 | 구현 |
|---|---|
| 실험 조립·설정 선택 | [legged_range_trial/launch/navigate.launch.py](../../src/legged_range_trial/launch/navigate.launch.py) |
| 저장 지도 검증·발행 | [site.py](../../src/legged_bringup/legged_bringup/site.py), [frozen_map.py](../../src/legged_bringup/legged_bringup/frozen_map.py) |
| Nav2 global planner plugin | [height_planner.cpp](../../src/legged_navigation/src/height_planner.cpp) |
| Grid A*·terrain 평가 | [planner core](../../src/rubi_heightmap_step_wavefront_planner/include/rubi_heightmap_step_wavefront_planner/planning) |
| 지역 controller | Nav2 Humble의 공식 RPP 또는 DWB plugin |
| 명령·입력 갱신 감시 | [command_watchdog.py](../../src/legged_bringup/legged_bringup/command_watchdog.py), [freshness_watchdog.py](../../src/legged_range_trial/legged_range_trial/freshness_watchdog.py) |

trial 패키지는 기존 planner 알고리즘을 복제하지 않는다. launch 구성과 YAML, 선택적 속도 어댑터·watchdog을 분리한다. 기존 launch의 사본을 사용하므로 향후 공통 bringup 구조를 바꾸면 trial도 함께 비교해야 한다.

## 지도 세 종류의 역할

| 자산 | 역할 | 현재 선택 |
|---|---|---|
| Localization PCD | 점군 정합으로 map 위치·자세 결정 | `maps/cost_wall_lower10_v3/localization.pcd` |
| 2D occupancy | 정적 벽·기둥 등 static layer | 같은 site의 `occupancy.yaml`/`floor.pgm` |
| Frozen height | 지면·단차 높이 증거와 CoT 계획 | 같은 site의 `height.rghm` |

v3 PCD와 occupancy는 `cost_wall_main_v2`와 바이트 단위로 같다. 따라서 기존 localization 명령의 v2 PCD 경로도 호환된다. height는 사용자 저장본 `cost_wall_lower10_v3_raw/run_1793023986818997341/map.rghm`을 그대로 묶은 v3 자료다. 현재 v3 world에서 옛 `sim_site_v1` height를 쓰면 단차 배치가 맞지 않는다.

`site.yaml`은 자산의 상대 경로·SHA-256·정합 검토를 제공하고 launch가 검사한다. `alignment.verified`는 시뮬레이션 지도 묶음 검토 범위이며 실물 배포 승인이나 현재 localization 정확도 보장이 아니다. landmark 검토 잔차 약 7.6/8.9cm가 남는다. [지도 README와 검토](../../maps/cost_wall_lower10_v3/README.md)

`map`은 이 세 지도를 정합한 공통 좌표계다. OccupancyGrid 메시지는 XY 셀을 표현하지만, TF의 `map`은 3D 좌표계다. map Z=0을 현재 로봇 발밑의 지면으로 가정할 수 없다. height의 identity transform은 이미 map 좌표로 저장했다는 뜻이지 Gazebo world와 map이 같다는 뜻이 아니다.

새로 놓은 큐브는 frozen height에 자동 추가되지 않는다. 실시간 obstacle layer로 costmap에 들어가고 전역 재계획·지역 충돌 검사에 사용된다. 반대로 static occupancy에 이미 찍힌 낮은 단차는 live cloud 필터를 바꿔도 static layer에서 사라지지 않는다.

## HeightPlanner가 실제 하는 일

1. `/legged/height`를 동일 frame의 height snapshot으로 받는다.
2. `createPlan()` 호출 시 Nav2 global costmap의 현재 메모리를 잠금 하에 복사한다.
3. 선택적 `costmap_clearance_m` 처리를 적용하고 StepEvaluator와 terrain hazard를 구성한다.
4. Grid A*가 거리·inflation 비용·CoT 높이 비용과 지형 금지 조건을 평가한다.
5. 성공한 grid node를 Path로 반환하고 인접점 방향으로 yaw를 채운다. 마지막 yaw는 goal의 yaw다.

현재 추가 clearance는 0이다. Grid 계획은 **회전 중인 사각 footprint 전체의 연속 충돌 검사**를 수행하는 SE(2) 계획이 아니다. 중심 경로가 가능해도 급격한 모서리에서 로봇이 감속·회전할 공간이 부족할 수 있다.

현재 [planner.yaml](../../src/legged_range_trial/config/planner.yaml)의 핵심값:

| 항목 | 값 / 해석 |
|---|---|
| map resolution / edge 검사 간격 | 0.05m / 0.025m |
| 탐색 제한 | 500ms, 500,000 expanded states |
| grid diagonal | true |
| 높이 jump 한계 | 0.10m |
| local relief threshold | 0.10m, 관측 증거·분위수 검사 포함 |
| terrain clearance | 활성, 위험 지형에서 0.20m |
| distance / height / inflation weight | 1.0 / 1.0 / 5.0 |
| CoT 높이 표(m) | 0, .05, .07, .09, .11, .13, .15 |
| 대응 등가 비용(m) | 0, .361, .716, 1.083, 1.272, 1.604, 1.634 |

CoT 표에 15cm가 있다고 15cm 통과를 허용하는 것은 아니다. 비용 표와 hard rejection은 별개다. 현재 jump/relief 한계는 10cm이며, 센서의 BODY 높이 필터도 단차 허용 한계와 별개다.

## YAML에 있지만 이 모드에서는 사용되지 않는 항목

`planner.yaml`은 별도 comparison/pursuit 실행과 공유하는 키를 포함한다. Nav2 HeightPlanner는 `configure()`에서 읽는 항목만 사용한다.

- **Nav2 모드 계획 호출 주기는 BT XML의 `RateController hz="1.0"`**가 결정한다. `replanning.check_period_s: 0.1`만 바꿔도 Nav2가 10Hz 재계획하지 않는다.
- `planner_server.expected_planner_frequency`는 목표 빈도 진단 기준이며 자체 계획 타이머가 아니다.
- 현재 HeightPlanner는 `tracking_refiner.*`를 호출하지 않으며 BT도 `SmoothPath`를 실행하지 않는다. YAML을 `enabled: true`로 바꾸는 것만으로 현재 Nav2 경로가 smoothing되지 않는다.
- plugin은 Nav2 costmap 객체를 직접 읽는다. `input_costmap_topic` 변경만으로 plugin의 지도가 바뀌지 않는다.
- `static_heightmap: true`에서는 과거 acquisition timestamp인 frozen height를 live 센서처럼 만료시키지 않는다.

## Footprint, inflation, clearance를 구분하기

| 장치 | 의미 | 현재 |
|---|---|---|
| Footprint | robot occupied shape의 평면 근사 | 길이/폭 0.55m, 네 꼭짓점 ±0.275m |
| Inflation | 장애물 주변의 금지/완만한 비용장을 형성 | global .6m, local .4m |
| Costmap clearance | Height planner가 별도로 obstacle/unknown 주변을 확장하는 추가 mask | 0m |
| Terrain clearance | height에서 판정된 위험 지형 주변 여유 | .2m |
| Collision Monitor Stop | 입력 점이 사각형 안에 일정 수 이상 들어오면 명령 정지 | footprint와 같은 사각형 |

inflation 반경 전체가 통과 불가 영역은 아니다. 이 Grid A*의 costmap 판정에서는 비용 253 이상을 막고 낮은 비용은 경로 비용으로 사용한다. 따라서 통로 폭에서 `2 × (footprint 반폭 + inflation 반경)`을 단순히 빼는 계산은 정확하지 않다.

감쇠 비용의 기본 형태는 inscribed radius 바깥에서 `252 × exp(-k × (d-r_inscribed))`이며 inflation 반경에서 잘린다. `k=cost_scaling_factor`를 낮추면 같은 거리의 비용이 높아지지만 반경 밖으로 영향이 확장되지는 않는다. 현재 global `k=.1`은 높은 비용을 거의 평탄하게 유지하는 시험값이다. 조기 회피를 보장하는 최적값으로 확정하지 않았다. [Humble inflation 구현](https://github.com/ros-navigation/navigation2/blob/humble/nav2_costmap_2d/include/nav2_costmap_2d/inflation_layer.hpp)

0.55m footprint는 기록된 모델 외곽(X 약 −.215~.199, Y 약 ±.246m)을 참고한 축소 시험값이다. 전체 gait의 swept volume과 실물 장착물 여유는 아직 검증되지 않았다. footprint를 더 줄이는 방식으로 추종 지연을 해결했다고 판단하면 안 된다.

## 개선 우선순위

현재는 같은 지도·footprint·native velocity를 유지하고 RPP의 감속·추종을 비교한다. 그래도 grid path의 급격한 꺾임이 원인으로 남으면, 충돌·terrain·비용을 재검사하는 경로 정제 단계 또는 더 이른 전역 우회를 검토한다. 단순 평균 smoothing으로 코너를 잘라 장애물을 통과시키지 않도록 해야 한다. MPPI는 현재 비교 범위에 넣지 않는다.
