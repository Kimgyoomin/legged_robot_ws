# 추가 바닥 보간 v4 — 2026-11-09

기준: `rubi_1f_height_20261109_flatsteps_v3`. 10 cm 두 개, 6 cm 한 개, 29 cm 두 개의 단차 윗면을 정리한 v3를 보존하고, **미관측 셀에만** 주변 바닥 높이를 추가했습니다. 기존 PCD와 2D occupancy 파일은 수정하지 않았습니다.

## 결과

| 항목 | v3 | v4 |
|---|---:|---:|
| 유효 height 셀 | 263,391 | 264,627 |
| 누적 추론 바닥 셀 | 5,295 | 6,531 |
| 이번에 추가한 바닥 | — | 1,236 셀 / 3.09 m² |
| 기존 셀 높이·분산·시각 변경 | — | 0 |
| 단차 보호 영역 변경 | — | 0 |
| occupied/unknown occupancy 영역 추가 | — | 0 |

원래 데이터 레코드를 바이트 단위로 유지하고 새 셀만 추가했습니다. 5 cm 해상도·map frame·원점·격자 크기도 유지했습니다. 결과 SHA-256: `8a6fb20d85292cbfa3a51de7c345fc2fe5829dba4b87f926fec2e1af292fde30`.

## 판단 원리

1. 기존 2D 지도가 free이며 occupied/unknown 경계에서 **35 cm 이상** 떨어진 위치만 후보로 선정합니다. free 표시만으로 평평한 바닥이라고 가정하지 않습니다.
2. 현재 지도 범위 안의 구멍과 작은 경계 결손만 고려합니다. 임의의 미탐사 방 전체를 확장하지 않습니다.
3. 이전 PCD의 바닥 높이는 이미 검증한 공간별 높이 편차를 보정해 사용합니다. 원래 관측한 현재 바닥과 PCD 바닥을 각각 반경 2 m에서 평면으로 적합합니다. 이전 버전에서 보간한 높이와 사람이 지운 흔적은 현재 바닥 적합의 근거로 사용하지 않습니다.
4. 각 적합 RMS ≤18 mm, 기울기 ≤0.04, 두 표면 높이 차이 ≤35 mm, 가까운 PCD 높이와의 차이 ≤35 mm를 만족해야 합니다.
5. 현재 바닥 근거는 최대 1.5 m, PCD 근거는 최대 0.75 m 이내여야 합니다. 현재 바닥이 한쪽에만 있으면 PCD가 대상점을 둘러싸고, 실제 관측 바닥이 0.65 m 이내인 경우만 허용합니다.
6. 단차 보호 마스크는 유지합니다. 관측된 바닥보다 높거나 낮은 구조와 그 주변 20 cm도 보존합니다. 실제 장애물인지 판단되지 않은 높이 관측은 평탄화하지 않습니다.

후보 2,787개 중 1,236개만 통과했습니다. 나머지 1,551개는 현재 바닥 적합 601, PCD 적합 362, 높이 불일치 226, 둘러싼 근거 부족 362 때문에 남겼습니다. 보호 조건에서 미리 제외된 셀은 이 후보 수에 포함하지 않습니다.

이번 추가분 중 16셀은 현재 측정으로 양쪽을 지지하고, 1220셀은 PCD가 둘러싸고 현재 바닥이 가까운 경계 보간입니다. 따라서 대부분은 남아 있던 지도 가장자리 결손 보완이며, 새로운 넓은 복도를 만든 결과로 해석하면 안 됩니다.

## 비판적 검증

현재 측정 바닥의 25/50/75 cm 반경을 각각 가린 공간 검증 90건 중 84건이 같은 기준을 통과했습니다. 통과 지점의 절대 높이 오차는 중앙값 **6.0 mm**, 95백분위 **20.5 mm**, 최대 **24.4 mm**였습니다. 이는 측정 바닥을 가린 검증이며 실제 미관측 영역의 정확도 보증은 아닙니다.

새 높이는 **추론값**입니다. RGHM v1 자체는 관측과 추론을 구분하지 않아 `provenance.npz`를 반드시 함께 보존합니다. 신규 분산 0.0009 m²는 파일 인코딩용 값이며 검증된 불확실성이 아닙니다. 신규 시각도 원본 snapshot 시각을 사용한 인코딩 시각입니다.

기존 occupancy의 고립된 검은 점, 구조물 경계, PCD와 높이가 맞지 않는 부분은 그대로 남았습니다. 이곳은 실제 장애물인지 과거 사람 흔적인지 추가 확인 없이 자동으로 바닥으로 바꾸지 않았습니다. 즉, 모든 빈 셀이 채워진 지도는 아닙니다. Nav2가 점유 지도로 막고 있는 곳은 height만 채워도 통과 가능해지지 않습니다.

## 전후 비교

세 번째 패널의 청록색이 이번에 추가한 셀입니다. 회색/검은색 배경은 기존 occupancy입니다.

![전체 전후](full_before_after.png)
![출발 영역](start_before_after.png)
![복도/기둥 보존](hall_before_after.png)
![상단 영역](upper_before_after.png)

## Planner 검사

11 cm 허용·geometry 높이 비용, 기존 clearance 설정에서 다섯 단차의 직선 경로 검사 결과는 v3와 같습니다. 10/10/6 cm는 직선 통과 가능하고, 29/29 cm는 직선 경계가 각각 24/23개 거부됩니다. 자세한 수치는 `planner_comparison.json`과 `probe_11_geometry.json`에 있습니다. 실제 Nav2 planner-only 실행 결과는 `runtime_verified/review_results.json`에 저장합니다.

이번 지도 작업으로 `planner.yaml`, localization, velocity feedback, controller, LCM은 변경하지 않았습니다. 실차 주행 검증은 별도입니다.

## 열기

```bash
cd ~/legged_robot_ws
source tools/navigation_env.sh
export ROS_DOMAIN_ID=186
export ROS_LOCALHOST_ONLY=1
ros2 launch legged_bringup rubi_height_navigation.launch.py \
  mode:=review \
  site:="$PWD/maps/rubi_1f_height_20261109_floorcomplete_v4/site.yaml" \
  review_planning_time_ms:=5000 \
  review_height_limit_m:=0.11 \
  review_height_cost:=geometry
```

기존 같은 domain의 review 창을 종료한 뒤 실행합니다. 2D Pose Estimate가 시작점, 2D Goal Pose가 경로 계산입니다. 센서·bag·주행 controller·LCM은 필요하지 않습니다. 실제 `mode:=nav2`에는 review 한계 인자가 적용되지 않습니다.

## 재현

`audit/fill_remaining_floor.py`는 v3 디렉터리, `reference_masks.npz`, **새 출력 디렉터리**를 인자로 받습니다. 기존 출력 덮어쓰기를 거부합니다. 입력 참조 파일과 검증 도구 위치는 `audit/inputs.json`에 기록했습니다. SHA-256을 확인해 같은 입력인지 검증한 뒤 재현합니다.

## 실제 Nav2 검증 결과

별도 domain 187에서 controller·LCM 없이 planner만 실행했습니다. GetParameters 서비스로 11 cm와 geometry 비용 적용을 확인했습니다.

| 경로 | 결과 | 길이 [m] | Y방향 범위 [m] |
|---|---|---:|---:|
| hall_route | 성공 | 43.136 | 4.450 |
| S5_10cm | 성공 | 2.000 | 0.000 |
| S3_10cm | 성공 | 1.500 | 0.000 |
| S4_6cm | 성공 | 2.000 | 0.000 |
| S1_29cm | 성공 | 2.497 | 0.600 |
| S2_29cm | 성공 | 2.456 | 0.550 |

10/10/6 cm는 직선 경로, 29/29 cm는 우회 경로였습니다. 길이 약 40 m의 복도 구간도 경로 계산에 성공했습니다. 이는 5초 검토용 탐색 예산에서의 결과이며, 기본 500 ms 주행 예산의 처리시간이나 실차 보행 성공을 의미하지 않습니다. 첫 검증은 ROS CLI 파라미터 조회 단계에서 종료되었고, 직접 서비스 조회로 수정한 검증 도구로 재실행했습니다. 첫 launch 로그는 `runtime/`, 완료된 결과는 `runtime_verified/`입니다.
