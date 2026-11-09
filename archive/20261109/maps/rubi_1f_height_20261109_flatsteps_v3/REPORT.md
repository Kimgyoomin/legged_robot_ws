# 다섯 단차 윗면 평탄화 v3

사용자가 확인한 단차 높이(10cm 두 개, 6cm 한 개, 29cm 두 개)를 기준으로, v2 높이 지도의 관측된 윗면 249셀을 보정했다. 단차 주변 바닥의 국소 평면에 지정 높이를 더해 평탄화했다. 모든 단차에 동일한 절대 map Z를 대입한 것은 아니다.

기존 원본·v1·v2 지도와 PCD/2D occupancy, planner 설정은 유지했다. 새 `site.yaml`을 지정해야 v3가 로드된다. 실행 중인 RViz/노드는 자동으로 교체되지 않는다.

## 범위와 결과

| 단차 ID | 확인된 위치 x,y (m) | v2 윗면 중앙 높이 | v3 상대 윗면 높이 | 수정 셀 |
|---|---|---:|---:|---:|
| S5 | 19.83, 43.16 | 10.69cm | 10.00cm | 75 |
| S3 | 29.60, 47.25 | 9.17cm | 10.00cm | 70 |
| S4 | 32.48, 43.82 | 5.75cm | 6.00cm | 20 |
| S1 | 38.82, 44.37 | 28.88cm | 29.00cm | 59 |
| S2 | 43.45, 43.24 | 29.33cm | 29.00cm | 25 |

전 값은 이전에 확인한 중앙 윗면 영역의 국소 바닥 대비 중앙값이다. 후 값은 **사용자가 알려준 치수로 부여한 보정값**이며 새 측정으로 얻은 정밀도나 실측 오차가 0이라는 뜻이 아니다. 총 보정 면적은 0.6225m²다.

![윗면 및 높이 단면 전후](step_tops_before_after.png)

왼쪽은 v2, 가운데는 v3, 오른쪽은 중심 단면이다. 자홍색은 수정한 관측 영역이다. 바닥 평면은 중심 반경 0.5~1.0m 고리에서 강건하게 추정했고 RMS는 5.1~9.2mm였다. 목표 윗면은 그 평면에 평행하다.

## 적용 방식과 보존 항목

1. 이전에 확인한 중앙 윗면 ROI를 시작 영역으로 사용했다.
2. ROI에서 15cm 이내의 관측 중 지정 높이의 75% 이상이고, 지정 높이보다 `max(4cm, 지정 높이×0.2)` 이상 높지 않은 셀을 후보로 삼았다. 시작 영역에 8방향으로 연결된 상단 관측만 추가했다.
3. 선택한 셀의 Z를 `주변 바닥 평면(x,y) + 지정 단차 높이`로 교체했다.
4. 미관측 셀은 채우지 않았고, 선택 영역 밖의 바닥·낮은 측면·구조물은 그대로 뒀다. 실제 폭이 알려져 있지 않으므로 외곽을 직사각형으로 확장하지 않았다. 상단 경계의 일부 혼합 셀은 선택에 포함될 수 있다.

최대 상승 보정은 약 6.73cm, 최대 하강 보정은 약 3.46cm였다. 중앙 윗면뿐 아니라 이어진 상단 관측의 낮은 혼합 셀도 보정했기 때문이다. 물체 폭의 정확성은 이 작업으로 검증되지 않는다.

파일 재검사로 관측/미관측 마스크, index, frame, 원점, 해상도, 크기, 각 셀 timestamp와 variance가 유지됨을 확인했다. 선택 영역 밖 Z도 동일하다. 원래 variance는 보정면의 사후 불확실성을 의미하지 않는다.

`provenance.npz`에는 기존 바닥 추론·수정 기록과 함께 `v3_top_flattened`, `v3_step_labels`, `v3_before`를 보존했다. RGHM만 단독으로 읽는 planner에는 이러한 보정 출처 구분이 전달되지 않으므로 인수인계 시 provenance와 이 문서를 함께 보존한다.

## 11cm 기하 높이 비용에서 경로 검증

동일 costmap과 동일 planner 설정(인접 높이 한도 0.11m, local relief 한도 0.11m, CoT=false)으로 v2와 v3를 비교했다. 지형 여유 거리 0.2m와 costmap 여유 거리 0.45m는 바꾸지 않았다.

| 단차 | v2 중심 직선의 거부 edge 수 | v3 | v2 경로 길이 | v3 경로 길이 |
|---|---:|---:|---:|---:|
| S5 10cm | 13 | 0 | 2.166m | 2.000m 직선 |
| S3 10cm | 8 | 0 | 1.541m | 1.500m 직선 |
| S4 6cm | 0 | 0 | 2.000m | 2.000m 직선 |
| S1 29cm | 24 | 24 | 2.456m | 2.456m 우회 |
| S2 29cm | 23 | 23 | 2.414m | 2.414m 우회 |

![동일 설정에서 보정 전후 경로](routes_before_after.png)

설치된 Nav2 HeightPlanner로도 격리 domain 187에서 다섯 ComputePathToPose 요청이 모두 성공했다. 6/10cm 세 구간은 Y 변화가 없는 직선(2.0m, 1.5m, 2.0m)이었고, 29cm 두 구간은 약 0.6m/0.55m 옆으로 우회했다. 실제 action 입력과 core probe의 셀 중심이 일부 수 cm 달라 29cm 우회 길이는 각각 약 2.497m/2.456m였다.

시험은 controller·LCM 없이 실행했고 주행 명령 토픽이 없음을 검사했다. 경로 생성 성공은 실제 로봇의 단차 보행 성공이나 전체 경로의 실시간 성능을 의미하지 않는다. 출입구의 occupancy clearance 문제와 남은 사람 흔적은 이번 윗면 보정 대상이 아니다.

## 실행

기존 review launch를 종료한 뒤:

```bash
cd ~/legged_robot_ws
source tools/navigation_env.sh
export ROS_DOMAIN_ID=186
export ROS_LOCALHOST_ONLY=1

ros2 launch legged_bringup rubi_height_navigation.launch.py \
  mode:=review \
  site:="$PWD/maps/rubi_1f_height_20261109_flatsteps_v3/site.yaml" \
  review_planning_time_ms:=5000 \
  review_height_limit_m:=0.11 \
  review_height_cost:=geometry
```

사용한 지도, 보정 수치, 비교 자료는 `map.rghm`, `metrics.json`, `planner_comparison.json`, `probe_11_geometry.json`, `runtime/`에 있다. `audit/`에는 보정 및 검증 스크립트를 보존했다. source v2 지도 해시는 `c42f246860b2a22c0446954e3256566c04e68e283b6234a6454a81a0fc3ac912`이다.
