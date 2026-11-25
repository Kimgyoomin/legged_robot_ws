# Grid A* 단차 비용과 로봇별 설정

Hybrid와 Height-only의 기본 상승 비용은 240회 실험으로 보정한 3차 CoT다. 사용한 브랜치 버전은 `6d78b40610c560f35e6db934ad3cd0936799a888`이다. [보정 데이터](https://github.com/Kimgyoomin/navigation_real/blob/6d78b40610c560f35e6db934ad3cd0936799a888/rubi_heightmap_step_wavefront_planner/config/cot_fit_240trials_20261019.json) · [참조 launch](https://github.com/Kimgyoomin/navigation_real/blob/6d78b40610c560f35e6db934ad3cd0936799a888/rubi_heightmap_step_wavefront_planner/launch/grid_cost_experiment.launch.py)

## 상승과 하강

상승 이벤트 높이 H는 인접 셀의 높이차 절댓값과 주변 local relief 중 큰 값이다. H가 노이즈 바닥 0.01m를 넘고 실제 이동이 상승 방향일 때, 미터 단위 H를 센티미터로 바꾼 h = 100 × H를 식에 넣는다.

```text
f(h) = −0.00027219790684918907 h³ + 0.004037674812204141 h² + 0.008922036892203038 h
평지 환산 거리 P(H) = f(h) / 0.2397005087210085 × 1 m
한 상승 edge의 높이 비용 = height_cost_weight × P(H) × clamp(실제 상승량 / H, 0, 1)
```

| 한 번에 올라서는 이상적 단차 | 평지 환산 거리 | 기본 가중치 5 적용 |
|---|---:|---:|
| 6cm | 0.5845m | 2.9223m |
| 10cm | 0.9211m | 4.6055m |
| 29cm | 11cm 범위 초과 | 통과 금지 |

하강은 참조 구현의 기하 비용을 사용한다. 높이차 d가 노이즈 바닥 n을 넘으면 `weight × Hmax × ((d−n)/(Hmax−n))^p`이며 기본 Hmax=0.11m, n=0.01m, p=2다. Vanilla에서는 상승에도 이 식을 사용한다. 표는 상승 한 번의 값이다. 실제 경로 비용은 모든 edge의 거리·높이와 hybrid의 2D inflation을 합하므로, 단차가 좁고 평지 우회가 짧으면 6cm도 우회할 수 있다.

## 바꿀 파일

- [hybrid 설정](../src/legged_navigation/config/height_hybrid.yaml)과 [height-only 설정](../src/legged_navigation/config/height_only.yaml)의 `evaluation.height_cost_weight`로 모든 높이의 비용 배율을 조절한다. 비교 시 거리 가중치, 지도와 다른 제한을 고정한다.
- [공유 보정값](../src/legged_navigation/config/cot_cubic.yaml)은 cm 내림차순 계수, 평지 기준 CoT, 실험 거리와 상한을 담는다. 다른 로봇의 실측 보정 파일은 planner YAML의 `height_cost_calibration_file`에서 절대경로로 지정한다. 상대경로는 설치된 `legged_navigation/config` 기준이다.
- `evaluation.grid_cot_height_cost_enabled: false`는 vanilla 비교용이다. `evaluation.max_crossable_height_jump_m`, `evaluation.local_relief_threshold_m`와 `evaluation.grid_terrain_clearance_distance_m`은 비용 배율과 독립된 통과 조건이다. 적용한 3차식은 11cm까지 보정됐다.
- [비용 코드](../src/rubi_heightmap_step_wavefront_planner/src/step_evaluator.cpp)의 `gridCotEquivalentPenalty()`가 다항식을 계산하고, `evaluateGridTransition()`이 방향·상승 비율·가중치를 반영한다. 두 Nav2 planner는 같은 Grid A* 평가를 사용한다.

## 실행 중 확인

```bash
ros2 param get /planner_server Height.evaluation.grid_cot_height_cost_enabled
ros2 param get /planner_server Height.evaluation.grid_cot_polynomial_coefficients_cm
ros2 param get /planner_server Height.evaluation.height_cost_weight
```

Height-only에서는 `Height`를 `HeightOnly`로 바꾼다. YAML을 바꾼 후 navigation을 재시작한다. 확정 지도의 단차 배치와 결과는 [지도 목록](../maps/README.md)에서 확인합니다.
