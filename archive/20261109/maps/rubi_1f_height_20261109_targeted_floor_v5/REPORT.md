# V5: 사용자 지정 네 구역의 바닥 보완 (2026-11-09)

V4를 보존하고, 사용자가 RViz에서 지정한 세 구역의 빈 바닥과 한 구역의 사람 흔적을 수정했다. 원본 관측 바닥, 기존 2D occupancy, 높이 편차를 보정한 과거 PCD를 비교했다. 재녹화·재정합이나 실차 주행은 수행하지 않았다.

## 처리 결과

해상도는 5cm이며, 한 셀 면적은 0.0025m²이다.

| 구역 / 클릭 좌표 | 처리 | 새로 채운 셀 | 기존 높이를 수정한 셀 |
|---|---|---:|---:|
| A: (13.406, 43.5112) | 지정 범위의 작은 바닥 구멍 | 83 | 0 |
| B: (5.92883, 4.87223) | 기둥 북서쪽 빈 바닥 | 174 | 0 |
| C: (5.11579, -0.04495) | 출입구의 사람·이동 물체 흔적 평탄화 | 0 | 289 |
| D: (31.0846, 51.578) | 기둥 주변 빈 바닥 | 218 | 0 |
| 합계 | 빈 바닥 1.1875m², 평탄화 0.7225m² | 475 | 289 |

- 유효 높이 셀: **264,627 → 265,102**.
- A의 지정 범위 안 미관측 셀은 **83 → 0**이다.
- B/D에는 기둥 내부·경계 여유·자료가 부족한 부분을 남겼다. 모든 흰/검은 영역을 바닥으로 바꾼 것이 아니다.
- C 클릭점 높이: **−0.33022 → −0.46193m**. 주변 바닥보다 약 13.2cm 높았던 흔적을 낮췄다. C 전체의 최대 수정량은 33.6cm이다.
- 다섯 실제 단차의 윗면 **249셀**, 단차 보호 영역, 기존 occupancy의 occupied/unknown 영역은 변경하지 않았다.
- 원래 셀 중 수정 대상이 아닌 **264,338개 레코드는 문자열 그대로 보존**했다. 수정 셀도 높이 외 분산·시각·사용 가능 여부는 보존했다.

## 범위와 추정 방법

정확한 폴리곤/범위와 마스크는 `metrics.json` 및 `provenance.npz`에 기록했다.

1. 현재 원본 관측 중 과거 PCD 바닥과 높이가 일치하는 점으로 주변 바닥을 추정한다. 이전 버전에서 채워 넣은 값이나 평탄화한 값을 학습 지지점으로 사용하지 않는다.
2. A/B/D는 로컬 평면을 robust least squares로 구하고 과거 PCD의 로컬 바닥 평면과 비교한다. 사방 지지가 부족하거나 높이가 맞지 않는 셀은 남긴다.
3. 사방에 현재 바닥이 있고 둘레 높이 범위가 4cm 이내인 16셀 이하의 작은 구멍은 별도로 검사했다. 이 방법으로 추가된 29셀도 별도 마스크로 남겼다.
4. C는 사용자가 평탄화를 지정한 폴리곤 안에서 주변의 깨끗한 바닥 426점을 사용했다. 바닥보다 3cm 이상 솟은 부분과 한 셀 폭 가장자리를 평면으로 바꿨다. 과거 PCD 한 점의 잡음 대신 주변 PCD 평면과 비교한다.
5. 채움은 기존 2D 지도상 free인 곳에서만 수행했다. 이번 지정 구역의 데이터 편집 여유는 10cm이다. 이는 **로봇 충돌 여유를 줄인 값이 아니다**. Nav2의 45cm costmap clearance와 20cm terrain clearance는 그대로다.

C의 추정 평면:

```text
z = -0.461999866
    -0.000126312 * (x - 5.11579)
    +0.003479769 * (y + 0.0449459)
```

## 수치 검사

- 수정 영역 평면 적합 RMS: 중앙값 **7.35mm**, 95백분위 **11.63mm**.
- 과거 PCD 로컬 평면과의 높이 차이: 중앙값 **3.09mm**, 95백분위 **8.62mm**, 최대 **27.39mm**.
- 새 높이와 맞닿은 기존 셀의 높이 차이: 중앙값 **2.74mm**, 95백분위 **15.84mm**, 최대 **37.13mm**. 최대값은 기둥 주변에 보존한 측정 높이와의 경계에서 발생한다.
- 20/40cm 반경의 실제 바닥 관측을 가리고 예측한 공간 홀드아웃 160회 중 153회가 조건을 만족했다. 절대오차 중앙값 **2.98mm**, 95백분위 **9.81mm**, 최대 **19.97mm**.
- 홀드아웃은 주변 관측에 대한 보간 일관성 검사다. 채운 구멍의 실제 바닥을 새로 측량한 정확도가 아니다. 나머지 7회는 지지/높이 조건을 만족하지 않아 제외했으며 원시 결과도 저장했다.

## Nav2 전후 비교

분리된 localhost domain 187에서 같은 planner-only 구성을 각각 실행했다. geometry cost, 높이 한계 0.11m를 ROS parameter service로 확인했고, controller/LCM 없이 `ComputePathToPose`만 호출했다.

| 시험 | V4 | V5 |
|---|---|---|
| A 클릭점 → (15.0,43.5) | 성공 | 성공 |
| C 클릭점 → (5.0,0.8) | `grid start invalid` | **성공, 17 poses / 0.862m** |
| D 기둥 옆 (30.3,51.1) → (30.3,52.5) | 성공 | 성공 |
| 복도 장거리 경로 | 성공 | 성공, 43.14m |
| 10cm 두 곳 / 6cm 한 곳 | 직선 통과 | 직선 통과 유지 |
| 29cm 두 곳 | 우회 | 우회 유지 |
| B 클릭점을 시작점으로 사용 | 거절 | 거절 |
| D 클릭점을 시작점으로 사용 | 거절 | 거절 |

**B/D의 거절은 바닥을 더 지운다고 해결할 대상이 아니다.** B 클릭점의 기존 지도 장애물까지 거리는 약 20cm로, 현재 planner clearance 45cm보다 작다. D 클릭점은 기존 지도의 기둥 occupied 영역에 걸친다. 해당 주변을 볼 때는 사용할 수 있지만, planner 시작점은 기둥에서 떨어진 바닥에 찍어야 한다. 두 위치의 기둥을 제거하거나 clearance를 낮추지는 않았다.

원시 action 결과와 launch 로그는 `runtime_v4/`, `runtime_v5/`에, 요약은 `planner_comparison.json`에 있다. 이것은 지도 및 global planner 검증이며 실차 주행 성공을 뜻하지 않는다.

## 비교 그림

- [A: 작은 구멍 채움](A_hall_before_after.png)
- [B: 출입구 기둥 주변](B_entrance_fill_before_after.png)
- [C: 출입구 사람 흔적 평탄화](C_entrance_flat_before_after.png)
- [D: 상부 기둥 주변](D_upper_before_after.png)

## 실행

기존 review 창을 종료한 다음 새 지도를 지정한다. 지도 데이터만 추가했으므로 다시 빌드할 필요는 없다.

```bash
cd ~/legged_robot_ws
source tools/navigation_env.sh
export ROS_DOMAIN_ID=186
export ROS_LOCALHOST_ONLY=1

ros2 launch legged_bringup rubi_height_navigation.launch.py \
  mode:=review \
  site:="$PWD/maps/rubi_1f_height_20261109_targeted_floor_v5/site.yaml" \
  review_planning_time_ms:=5000 \
  review_height_limit_m:=0.11 \
  review_height_cost:=geometry
```

`2D Pose Estimate`는 검토용 시작점, `2D Goal Pose`는 global path 계산이다. 이 모드에는 실차 controller/LCM이 없다. 기존 실시간 navigation 기본 설정은 바꾸지 않았다.

## 출처와 재현

- 기반 V4: `../rubi_1f_height_20261109_floorcomplete_v4/map.rghm`
- V4 SHA256: `8a6fb20d85292cbfa3a51de7c345fc2fe5829dba4b87f926fec2e1af292fde30`
- V5 SHA256: `92aadca91c74a48ab22e28482097a65f9b6c459071f87e681ca88eea0b5046ea`
- 기존 PCD/2D 지도는 `site.yaml`의 경로/해시로 고정했다. 원본은 수정하지 않았다.
- `audit/repair_regions.py`로 새 출력 디렉터리에 재생성할 수 있다. 검사 코드는 `audit/validate_review.py`이다.
- 보간·수정 마스크: `v5_filled`, `v5_repaired`, `v5_region_labels`, `v5_small_enclosed_hole_fill`, 이전 높이 `v5_before`.
- 보간 셀은 새 센서 관측이 아니다. RGHM v1은 inferred 표지가 없으므로 `provenance.npz`를 지도와 함께 보존해야 한다. 새 셀의 분산 0.0009와 snapshot 시각은 직렬화용 값이며, 보간 불확실성이나 실제 관측 시각을 뜻하지 않는다.
