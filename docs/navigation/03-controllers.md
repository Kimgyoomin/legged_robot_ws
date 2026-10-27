# 03. RPP·DWB와 코너 감속

[문서 처음으로](README.md)

## 역할 분담

| 구분 | 경로 추종 | 주변 장애물 처리 | 현재 사용 방식 |
|---|---|---|---|
| Nav2 DWB | global path·goal 비용을 평가 | local costmap에서 속도 후보의 예측 궤적을 평가하고 선택 | 기존 비교 기준 |
| Nav2 RPP | lookahead 곡률로 경로를 추종 | 곡률/근접 비용 감속, 앞으로 투영한 footprint 충돌 검사 | 코너 감속 비교 프로필 준비 |
| 기존 custom Pure Pursuit | 자체 경로 추종 | 별도 guard·전역 재계획에 의존 | `mode:=pursuit`; Nav2 RPP와 다른 구현 |

RPP는 local costmap을 사용하지만 장애물 옆을 지나는 새 경로 후보를 검색하는 planner는 아니다. global path가 큐브 안으로 향하면 충돌을 예측해 감속·실패할 수 있으며, **우회 경로는 HeightPlanner의 재계획이 공급해야 한다**. DWB도 제한된 시간·속도 후보 안에서 선택하므로 장거리 후진 후 반대 방향 우회 같은 탈출을 항상 찾지 못한다. [Nav2 Humble RPP 설명](https://github.com/ros-navigation/navigation2/blob/humble/nav2_regulated_pure_pursuit_controller/README.md)

RPP를 비교하는 이유는 global path가 통과 가능한 경우의 추종과 급회전 전 감속을 먼저 확인하기 위해서다. 외부 custom RPP를 새로 만들거나 MPPI를 추가하지 않았다.

## RPP 현재 파라미터

실제 파일: [config/rpp.yaml](../../src/legged_range_trial/config/rpp.yaml). `controller:=rpp`일 때만 FollowPath plugin 설정을 이 파일로 교체한다.

| 항목 | 값 | 의미 |
|---|---:|---|
| `desired_linear_vel` | .7m/s | 기본 목표 전진 속도 |
| `lookahead_dist` | .6m | 고정 lookahead 기본값 |
| velocity scaled lookahead | true | 현재 속도에 따라 거리 선택 |
| `lookahead_time` | 1.2s | 속도 기반 lookahead 시간 |
| min/max lookahead | .4 / .8m | 선택 거리 제한 |
| curvature regulation | true | 곡률에 따른 감속 |
| `regulated_linear_scaling_min_radius` | 1.2m | 이 반경보다 작은 회전에서 곡률 감속 |
| `regulated_linear_scaling_min_speed` | .05m/s | 규제 감속의 최소 속도 설정 |
| proximity regulation | true | local 비용에 따른 감속 |
| `cost_scaling_dist` / gain | .4m / 1.0 | 장애물 근접 감속 거리·계수 |
| `inflation_cost_scaling_factor` | 5.0 | **local** inflation의 계수와 일치시켜 거리 역산 |
| collision detection | true | 전방 투영 충돌 검사 |
| time-to-collision limit | 1.0s | carrot까지의 제한과 함께 사용 |
| rotate-to-heading | true | 경로 방향/목표 방향 정렬 |
| heading angle threshold | .7rad (약40°) | 경로 방향 정렬 시작 기준 |
| rotate speed / angular accel | .4rad/s / 1.0rad/s² | 정렬 회전 목표·가속 제한 |
| approach distance / min speed | .6m / .05m/s | 목표 접근 감속 |
| `allow_reversing` | false | 현재 RPP 후진 추종 비활성 |

곡률만 보면 작은 회전 반경 R에서 대략 `v=0.7×R/1.2`로 줄어든다. R=.6m이면 약 .35m/s, R=.3m이면 약 .175m/s다. 실제 값은 proximity/접근/충돌/회전 모드와 최소 속도 조건도 함께 적용하므로 이 식 그대로 항상 출력되지는 않는다.

RPP도 보행체의 실제 제동거리·관성·명령 응답을 자동 식별하지 않는다. `.7m/s` 명령 상한이 실제 속도의 강제 상한은 아니다. 급격한 감속 명령 뒤에도 몸체가 전진하는지 측정해야 한다.

## DWB 현재 비교 기준

파일: [config/nav2.yaml](../../src/legged_range_trial/config/nav2.yaml).

| 항목 | 값 |
|---|---|
| min/max_vel_x | −.2 / .7m/s |
| min/max_speed_xy | 0 / .7m/s |
| min/max_vel_y | 0 / 0m/s |
| max_vel_theta | .8rad/s |
| acc_lim_x / decel_lim_x | .5 / −.5m/s² |
| acc_lim_y / decel_lim_y | .5 / −.5m/s² |
| acc_lim_theta / decel_lim_theta | 1.0 / −1.0rad/s² |
| vx/vy/vtheta samples | 12 / 1 / 20 |
| sim_time | 2.0s |
| linear/angular granularity | .05m / .025rad |
| ObstacleFootprint scale | .02 |
| PathAlign / PathDist | 32 / 32 |
| GoalAlign / GoalDist | 24 / 24 |
| RotateToGoal scale / slowing factor | 32 / 5 |

`min_vel_x`만 음수로 바꾸면 후진 후보가 허용될 뿐이다. Path/Goal 비용, Oscillation 제약, 현재 속도·가속 한계, collision invalidation에 의해 후진이 선택되지 않을 수 있다. `min_speed_xy`는 방향이 아닌 **속력의 크기**이므로 후진을 위해 음수로 맞추는 값이 아니다. 0이면 저속/정지 후보를 허용하는 데 적합하고, `max_speed_xy`는 x/y 합성 속력을 제한한다.

`acc_lim_y`를 0에서 .5로 변경한 것은 횡방향 명령을 추가한 것이 아니다. 보행 중 측정되는 vy가 0이 아닐 때 StandardTrajectoryGenerator가 예측 중 이를 0으로 줄일 수 있게 한 시험 모델이다. y 명령 후보는 여전히 0이다. 원래 acc/decel_y가 모두 0이면 예측 내내 횡속도가 남을 수 있었다. 이 .5는 실측 식별값이 아니며, offline 예측 비교를 전체 navigation 개선으로 해석하지 않는다. [Humble trajectory generator](https://github.com/ros-navigation/navigation2/blob/humble/nav2_dwb_controller/dwb_plugins/src/standard_traj_generator.cpp)

## 코너에서 확인한 현상

[corner_braking 보고서](reports/fastlio_velocity_20261027/corner_braking.html)의 6.4초 sim 기록에서는 NAV2 입력으로 주행하면서 전진 약 .7m/s와 yaw 약 .295rad/s가 함께 나왔다. 이 조합의 단순 회전반경은 약 2.37m라 급한 코너를 좁게 돌기 어렵다.

보수적인 cell/footprint 교차 검사에서 겹침이 나타난 뒤에도, 한 시점에는 전진 명령 .618m/s에 수신 추정 속도 .876m/s가 관찰됐다. 명령이 0이 된 뒤에도 수신 속도 .620m/s인 표본이 있었다. 다만 속도 age 약 .1초, TF age 약 .2초이므로 같은 순간의 접촉·제동 응답으로 단정하지 않는다. DWB 내부 critic을 직접 계측한 결과도 아니다.

이 근거는 **미리 감속하는 명령 선택이 필요하다**는 가설을 지지한다. RPP profile은 그 가설을 시험하기 위한 구성이다. 정지 후 cloud timeout은 Gazebo clock이 멈춘 뒤의 현상이라, 코너 진입 최초 원인과 구분한다.

## 정지 장치와 `aborted`는 한 가지가 아니다

| 관찰 | 발생 계층 | 대응 |
|---|---|---|
| `grid start invalid`, `grid path not found` | HeightPlanner/global path | 시작/목표 cost·height·map 정합 확인 |
| `No valid trajectories` | DWB local rollout | footprint, local cost, 측정 twist, 후보 제약 확인 |
| raw 명령은 있는데 safe가 0 | Collision Monitor | 근접점이 Stop 안에 있는지 확인 |
| safe는 있는데 최종 cmd가 0 | watchdog | 상태 토픽, stamp/TF/costmap/odom freshness 확인 |
| `Failed to make progress` | progress checker | 일정 거리 이동 여부 확인 |
| RViz feedback aborted | 최종 action 상태 | 어떤 action 실패가 BT에 전달됐는지 로그 순서 확인 |

trial의 `failure_tolerance=600`은 마지막 유효한 controller 명령 이후 controller exception을 기다리는 ROS 시간이다. 이 기간에는 실패 시 0을 발행하고 다시 시도한다. 진행 검사도 `.2m / 600s`다. BT는 action 실패 후 wall-clock 1초 Delay를 두고 최대 600회 재시도한다. controller 실행 시간까지 더해지므로 **전체 goal의 정확한 10분 deadline이 아니다**. planner 실패는 controller tolerance와 다른 경로로 발생한다. [BT XML](../../src/legged_range_trial/config/navigate.xml), [Humble controller server](https://github.com/ros-navigation/navigation2/blob/humble/nav2_controller/src/controller_server.cpp)

Collision Monitor Stop은 탈출 회전 명령을 만드는 recovery가 아니다. 사각형 안에 점이 남으면 회전 명령도 0이 될 수 있다. 현재 BT에는 자동 BackUp/Spin 회복 동작이 없으며, plugin 라이브러리가 목록에 있다는 사실만으로 실행되지 않는다. 장시간 대기는 goal을 유지하는 정책이지 stuck를 해결하는 알고리즘이 아니다.

## 비교 튜닝 방법

같은 시작·목표·큐브 배치·native velocity로 DWB/RPP를 비교한다. 먼저 local/global inflation과 footprint를 고정한다. RPP에서 코너 반경, lookahead, 회전 진입 조건을 한 묶음씩 바꾸고 경로오차·최소 여유·cmd/실제 속도·회전 지연을 기록한다. 실패가 전역 경로 자체인지 지역 추종인지 구분한 뒤 해당 계층만 조정한다. [실행·기록 방법](05-operation.md)
