# Height navigation — 패키지 직접 실행

Driver 또는 bag → `fast_lio_localization` → `legged_navigation` 순서로 실행합니다. 새 Height Map은 중간에 `rubi_height_mapping`으로 생성·저장하고 mapper를 종료한 뒤 사용합니다.

- 실물 전체 명령: [OPERATIONS.md](OPERATIONS.md)
- `[26.11.19]rubi_pointfoot_nav` 재생 명령: [HEIGHT_REPLAY.md](HEIGHT_REPLAY.md)
- 파라미터 파일: [TUNING.md](TUNING.md)

[운용 YAML](../src/navigation_dds/config/operation.yaml)에서 지도·토픽·frame을 지정한 뒤 각 직접 launch에 `operation:=live` 또는 `operation:=replay`를 전달합니다.

## RViz 한 창 사용

**RViz는 localization에서 띄운 한 창을 계속 사용합니다.** 기본 구성 `src/FAST_LIO_LOCALIZATION2/rviz/fastlio_localization.rviz`에 Nav2 패널·**Nav2 Goal**, `/plan`, local/global costmap, frozen `/legged/height`를 포함합니다. 실물은 localization의 RViz를 사용하고, 녹화 localization을 쓰는 복도 재생은 navigation의 RViz를 엽니다. 별도 창이 필요할 때만 navigation에 `rviz:=true`를 지정합니다.

Height mapping 중에는 Displays에서 **Live global height (mapping only)**를 켜고, 저장 후 navigation으로 전환하면 끕니다. Height-only에서는 **Occupancy map (hybrid)**와 **Global costmap (hybrid)**를 끕니다. 이 체크는 시각화에만 영향을 줍니다.

이미 열린 localization RViz는 **File → Open Config**로 `~/legged_robot_ws/src/FAST_LIO_LOCALIZATION2/rviz/fastlio_localization.rviz`를 불러오면 됩니다. Localization 프로세스를 재시작할 필요는 없습니다.

## Global planner 선택

| 구분 | 직접 launch | 설정 파일 |
|---|---|---|
| Height + 2D Occupancy + 실시간 global 장애물 | `legged_navigation height_hybrid_navigation.launch.py` | `src/legged_navigation/config/height_hybrid.yaml` |
| Height-only global planner | `legged_navigation height_only_navigation.launch.py` | `src/legged_navigation/config/height_only.yaml` |
| 두 구성 공통 local costmap·DWB | 위 launch에서 함께 실행 | `src/legged_navigation/config/nav2_common.yaml` |

두 구성 모두 운용 YAML의 `common.navigation.height_map`을 읽습니다. Hybrid는 `common.hybrid.map_yaml`도 읽습니다. 다른 장소의 PCD·height·2D 지도를 섞지 않습니다. 기존 site manifest의 assets 경로를 확인할 수 있지만 직접 launch는 `site` 인자를 사용하지 않습니다.

실물은 `use_sim_time:=false`, bag은 모든 노드에 `use_sim_time:=true`를 사용합니다. 재생에서는 `cmd_vel_topic:=/legged/replay/cmd_vel`로 새 출력을 분리합니다.

Height 입력이 바뀌면 terrain cache를 다시 준비합니다. 매 goal마다 전체 frozen height를 다시 처리하지 않습니다. `terrain READY` 후 goal을 지정합니다. `Height.*` 또는 `HeightOnly.*` 설정은 YAML 수정 후 재시작 또는 lifecycle cleanup/configure로 반영합니다.

높이 변화 제한 0.11 m, Height 위험 지형 clearance 0.20 m, Hybrid의 2D 장애물 추가 clearance 0.30 m는 서로 다른 조건입니다. Height-only는 마지막 0.30 m와 2D inflation 비용을 사용하지 않습니다. Local DWB footprint 검사는 공통으로 유지합니다.
