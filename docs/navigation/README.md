# RUBI Navigation 개발·운영 인계 문서

**기준: 2026-10-27, ROS 2 Humble, `legged_range_trial`의 현재 소스 설정.**
전방 ±60° 비대칭 점군 필터, Nav2 RPP/DWB, 높이 기반 전역 계획, FAST-LIO 속도 피드백을 한곳에 정리했다. 이 문서는 저장소의 구성 기준이며, 이미 실행 중인 노드가 같은 값을 쓰는지는 런타임 파라미터로 확인해야 한다.

## 읽는 순서

| 문서 | 내용 |
|---|---|
| [01. 전체 구조와 전역 계획](01-architecture-and-planning.md) | 데이터 흐름, 지도 선택, Height A*, costmap/clearance/footprint의 차이 |
| [02. 비대칭 장애물 필터](02-obstacle-filter.md) | ±60°/0.3m/1.0m, 센서 원점과 BODY 높이, clearing, Jetson 비용 |
| [03. RPP와 DWB](03-controllers.md) | 현재 계수, 코너 감속, 회피 역할 분담, 실패와 재시도 |
| [04. FAST-LIO 속도 피드백](04-fastlio-velocity.md) | native state → body → base_link → Nav2, GT 비교, 지연 처리 |
| [05. 실행·점검·재현](05-operation.md) | 빌드, 세 터미널 실행, 설정 확인, 멈춤 진단, A/B 실험 |
| [06. 검증 보고서 색인](06-validation.md) | 과거 보고서·그림·수치 근거와 아직 검증하지 못한 부분 |
| [07. 브랜치와 인계 범위](07-handoff.md) | 포함 코드·지도, 외부 시뮬레이터 의존성, 다음 실험 |

## 현재 기준값

| 항목 | 현재 값 / 의미 |
|---|---|
| 실험 launch | `ros2 launch legged_range_trial navigate.launch.py` |
| launch 기본 controller | `dwb`; RPP는 **`controller:=rpp`를 명시** |
| 권장 비교 입력 | `fastlio_velocity:=true estimate_velocity:=false`를 명시; 둘 다 기본값은 false |
| 전방 필터 | 센서 원점 기준 3D 거리 ≥0.3m, base_link +X 방향 ±60° |
| 나머지 방향 | 센서 원점 기준 ≥1.0m; 최대 거리 5.0m |
| 장애물 높이 | base_link 기준 −0.20~+1.50m; 지면 기준 단차 높이가 아님 |
| costmap 관측 | `/legged/obstacles`; obstacle 최소 0.3m, 최대 4.0m |
| footprint | 0.55×0.55m 사각형, launch에서 padding 0으로 통일 |
| global inflation | 반경 0.60m / 감쇠 계수 0.1 |
| local inflation | 반경 0.40m / 감쇠 계수 5.0 |
| planner 추가 costmap clearance | 0.0m; costmap의 금지 셀 판정은 유지 |
| terrain 조건 | 높이 점프 한계 0.10m, terrain clearance 0.20m |
| 속도 | DWB max_vel_x/max_speed_xy 0.7m/s, RPP desired_linear_vel 0.7m/s |
| 실행 주기 | BT 전역 계획 1Hz, controller 10Hz, costmap 갱신 5Hz/발행 2Hz |
| 최종 guard | Nav2 trial 기본 OFF; Collision Monitor Stop·freshness watchdog은 ON |
| 장시간 대기 | failure_tolerance 600초, progress allowance 600초, BT 600회 재시도 |
| 지도 | `maps/cost_wall_lower10_v3/site.yaml` + 대응 v3 lower10 world |

설정의 실제 출처는 [trial config](../../src/legged_range_trial/config)와 [launch](../../src/legged_range_trial/launch/navigate.launch.py)다. 특히 costmap YAML의 `robot_radius`는 fallback이며, launch가 `collision.yaml`의 Stop 사각형으로 덮어쓴다.

## 현재 판단

FAST-LIO native 속도 출력과 Nav2 연결은 구현되어 있고, 자동 주행 기록에서 GT 대비 `vx` RMSE 약 0.024m/s, yaw rate RMSE 약 0.034rad/s를 확인했다. 그러나 좋은 속도 추정만으로 좁은 구간 통과가 해결되지는 않았다. 코너에서 전진 속도를 충분히 낮추지 않은 DWB 명령과 뒤늦은 정지가 관찰되어, 공식 RPP의 곡률·장애물 근접 감속을 비교할 설정을 준비했다.

**이 저장본은 RPP가 DWB보다 우수하다는 완료 보고서가 아니다.** RPP의 동일 경로 반복 주행, 전체 보행 외곽, 실물 제동 응답, Jetson Orin NX 성능은 추가 측정 대상이다. 실험 중 변경된 값이 많은 만큼, 과거 HTML의 값은 당시 조건으로 읽고 현재 값은 위 표와 소스를 따른다.
