# Lower10 v3 frozen map

현재 v3 시뮬레이션에서 저장한 height map을 navigation용으로 묶었습니다.
해상도는 5cm, 좌표계는 `map`, 사용 가능한 셀은 53,018개입니다.

- 대응 world: `/home/kim/cot_experiment/navigation_cost_comparison/worlds_v3_lower10/rubi_cost_comparison_lower10.world`
- 원본: `/home/kim/legged_robot_ws/maps/cost_wall_lower10_v3_raw/run_1793023986818997341/map.rghm`
- height SHA-256: `3cb78ea0148646bb6c579c7c7d7061e424a3612d55c3958c32ab77a892304876`
- PCD, occupancy YAML, 이미지: `cost_wall_main_v2`와 바이트 단위로 동일합니다.

`height.rghm`은 원본 그대로입니다. 기존 map 좌표를 유지하며, 높이 수정·평탄화·새로운
좌표 변환을 하지 않았습니다. `T_map_from_height`의 항등행렬은 height 데이터가
이미 map 좌표라는 뜻이며, Gazebo `world`와 `map`이 같다는 뜻이 아닙니다.

## 실행

같은 ROS domain에서 Gazebo와 FAST-LIO localization이 실행 중일 때:

```bash
source /home/kim/legged_robot_ws/install/local_setup.bash
ros2 launch legged_range_trial navigate.launch.py \
  site:=/home/kim/legged_robot_ws/maps/cost_wall_lower10_v3/site.yaml \
  use_sim_time:=true \
  cloud_topic:=/livox/lidar_PointCloud2
```

이 명령이 frozen height publisher와 map server를 실행하므로, height mapper나 별도
frozen publisher를 함께 실행하지 않습니다. 현재 FAST-LIO가 사용하는
`cost_wall_main_v2/localization.pcd`는 동일한 PCD라 계속 사용할 수 있습니다.
지도 파일만 추가했으므로 재빌드는 필요 없습니다.

실행 후 선택된 파일 확인:

```bash
ros2 param get /frozen_map file
ros2 param get /map_server yaml_filename
```

두 경로 모두 `maps/cost_wall_lower10_v3/` 아래를 가리켜야 합니다.
`site` 선택은 명령에서 명시해야 하며, simulator의 world를 자동으로 읽지 않습니다.

## 확인 범위

2026-10-26의 `review/` 기록과 원본 hash를 대조하고 2026-10-27에 패키징했습니다.
현재 실행 중인 Gazebo의 초기 world 파일도 그 검토 기록과 동일합니다.

- 하단 D 단차: world 기준 10cm, 주변 바닥 대비 지도 중앙값 약 9.88cm.
- 검토한 내부 바닥 ROI 관측률 100%; 기존 오프라인 실험 planner 질의 8/8 성공.
- 정적 물체를 이용한 근사 정합의 별도 검증 잔차는 약 7.6cm와 8.9cm입니다.
- 기존 오프라인 검증은 이전에 기록한 costmap과 실험 planner 설정을 사용했습니다.
  현재 `legged_range_trial` 설정의 DWB 주행이나 좁은 통로 통과 검증은 아닙니다.
- 바닥 높이의 넓은 왜곡은 남아 있습니다. 실제 장비 배포 검증은 하지 않았습니다.

`alignment.verified`의 범위는 이 시뮬레이션 지도 묶음의 출처·기하 검토입니다.
원본 검토 JSON의 `new_site_packaged: false`는 2026-10-26 당시 상태이며,
현재 패키징 결과는 workspace의 `validation/site_lower10_v3_20261027/`에 있습니다.

사용자가 나중에 추가한 큐브는 frozen 지형에 자동 저장되지 않습니다. 새 장애물은
실시간 점군과 costmap이 처리합니다. 지도 교체만으로 odometry 속도 입력이나
planner clearance, DWB 실패의 모든 원인이 해결된 것으로 판단하지 않습니다.
