# legged_navigation

직접 실행용 **Height+2D hybrid**와 **Height-only** Nav2 구성을 분리합니다.

| 역할 | Hybrid | Height-only |
|---|---|---|
| plugin C++ | `src/height_planner.cpp` | `src/height_only_planner.cpp` |
| plugin ID | `legged_navigation/HeightPlanner` | `legged_navigation/HeightOnlyPlanner` |
| launch | `launch/height_hybrid_navigation.launch.py` | `launch/height_only_navigation.launch.py` |
| global 설정 | `config/height_hybrid.yaml` | `config/height_only.yaml` |
| BT | `config/navigate_hybrid.xml` | `config/navigate_only.xml` |

공통 local DWB·LiDAR costmap·BODY 필터는 `config/nav2_common.yaml`, launch 조립은 `legged_navigation/launch_support.py`를 사용합니다. 별도 알고리즘 복제 없이 공통 `StepGridAStarPlanner`, height 평가, frozen cache를 이용합니다.

Height-only core의 내부 grid는 Height Map의 관측 여부만 0/255로 표현합니다. Nav2 2D costmap 값을 전달받는 API가 없으며 외부 obstacle/inflation 비용을 사용하지 않습니다. Nav2 planner server가 요구하는 빈 global costmap은 TF/lifecycle 유지만 담당합니다. 경로는 거리+높이 비용, 높이 기반 위험 clearance, 미관측 차단을 유지합니다.

실물은 navigation RViz를 끄고, 녹화 localization을 쓰는 복도 재생은 navigation RViz를 켭니다. Localization의 `fastlio_localization.rviz` 한 창에서 Nav2 Goal·경로·costmap·height를 봅니다. 별도 창이 필요하면 `rviz:=true`를 지정합니다.

실물은 `/cmd_vel`, 재생은 `/legged/replay/cmd_vel`로 출력하며 실물에서는 `pongbot_lcm joy_lcm.launch.py operation:=live`를 별도로 실행합니다. 둘을 동시에 실행하지 않습니다. [운용 순서](../../docs/OPERATIONS.md), [튜닝](../../docs/TUNING.md)를 참고하세요.

두 planner의 기본 상승 비용은 실측 3차 CoT이며 높이 가중치는 5입니다. 실물과 bag 실행에는 `operation:=live` 또는 `operation:=replay`를 지정합니다. [단차 비용과 설정](../../docs/STEP_COST_TUNING.md)
