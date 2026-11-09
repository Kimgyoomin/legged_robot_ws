# V6: 벽 아래 빈 바닥 보완 — 2026-11-09

사용자가 지정한 `(45.2512, 46.7622)` 주변의 빈 바닥을 V5 위에 추가했다. V5와 원본 지도는 보존한다. 기존 단차와 벽, 모든 기존 높이 레코드는 그대로다.

## 결과

- 수정 범위: `x=[44.35,46.25], y=[46.25,47.05]m` 안의 미관측 바닥.
- 새 바닥 **117셀 / 0.2925㎡**, 유효 셀 **265,102 → 265,219**.
- 범위 내 기존 2D 지도상 free인 미관측 셀 **117 → 0**.
- 기존 높이 변경 **0셀**. 다섯 단차를 포함해 기존 레코드 265,102개는 문자열 그대로 보존했다.
- 같은 범위에서 2D 지도상 벽에 해당하는 미관측 88셀은 유지했다. 사진의 빈 부분 전체를 일괄적으로 바닥으로 만들지 않았다.
- 클릭 셀: 미관측 → `map z=-0.427105m`. 이는 지도 좌표의 높이이며 바닥에서 42.7cm 높이라는 의미가 아니다.

## 방법과 정확도 한계

현재 지도에서 실제 관측한 깨끗한 바닥을 지지점으로 사용하고, 높이 편차를 보정한 과거 PCD 바닥의 독립적인 국소 평면과 대조했다. 이전에 채웠거나 평탄화한 셀은 지지점에서 제외했다. 2D occupancy가 occupied/unknown인 곳과 실제 단차 보호 영역도 제외했다.

구멍 위쪽이 벽이므로 사방에서 관측을 확보할 수 없는 셀이 있다. 그런 셀은 양옆과 아래쪽에 지지점이 있고, 가장 가까운 지지점이 40cm 이내이며, 현재 평면 RMS ≤12mm, 경사 ≤0.05, PCD 평면과 차이 ≤25mm인 경우에만 추가했다. 벽 바로 아래 마지막 free 셀까지 채웠지만 로봇/플래너의 충돌 여유 설정은 바꾸지 않았다.

| 수치 | 중앙값 | 95백분위 | 최대 |
|---|---:|---:|---:|
| 현재 바닥 평면 적합 RMS | 8.90mm | 10.61mm | 11.29mm |
| 과거 PCD 국소 평면과 차이 | 8.77mm | 16.63mm | 18.57mm |
| 새 셀과 인접한 기존 셀 높이 차이 | 5.32mm | 18.41mm | 23.82mm |
| 실제 관측을 가린 홀드아웃 절대오차 | 4.59mm | 14.98mm | 18.01mm |

홀드아웃은 15/30cm 반경 관측을 가린 60회 모두 조건을 만족했다. 이는 주변 바닥의 보간 일관성을 확인한 값이며 원래 비어 있던 곳의 실측 정확도는 아니다.

## 경로 확인

localhost 전용 domain 187에서 geometry cost, 높이 허용 0.11m의 planner-only review를 V5/V6에 동일하게 실행했다. controller와 LCM은 실행하지 않았다.

- `(44.25,46.2) → (46.25,46.2)` 복도 경로: 수정 전후 성공, 44 poses.
- 인접 29cm 단차를 가로지르는 요청: 수정 전후 우회 경로 성공, 41 poses.
- 클릭점 `(45.2512,46.7622)`은 2D 지도상 벽까지 약 5cm여서 수정 전후 시작점 거절이 정상이다. 바닥이 채워졌다고 로봇이 벽에 붙어 설 수 있게 되는 것은 아니다.
- 추가 요청 `(45.25,46.3) → (44.25,46.3)`은 수정 전후 `grid goal invalid`였다. 목표점의 유효성은 이번 바닥 채움과 별도로 확인해야 한다. 위의 `(44.25,46.2) → (46.25,46.2)`는 성공한 비교 경로다.

원시 결과는 `runtime_v5/`, `runtime_v6/`, 요약은 `planner_comparison.json`에 있다. 실차 검증은 수행하지 않았다.

## 전후 그림

[벽 아래 바닥 채움 전후](wall_gap_before_after.png)

[현재 높이·2D 지도·PCD 대조](reference_inspection.png)

## 실행

기존 review를 종료한 뒤 아래로 연다. 지도만 추가했으므로 빌드는 필요 없다.

```bash
cd ~/legged_robot_ws
source tools/navigation_env.sh
export ROS_DOMAIN_ID=186
export ROS_LOCALHOST_ONLY=1

ros2 launch legged_bringup rubi_height_navigation.launch.py \
  mode:=review \
  site:="$PWD/maps/rubi_1f_height_20261109_wall_gap_v6/site.yaml" \
  review_planning_time_ms:=5000 \
  review_height_limit_m:=0.11 \
  review_height_cost:=geometry
```

## 보존과 재현

- 기반 V5 SHA256: `92aadca91c74a48ab22e28482097a65f9b6c459071f87e681ca88eea0b5046ea`
- 새 V6 SHA256: `c5c491b9dc00e50659a6ce812296f001d5ae4f6af8ab126f4cff2561828ee35c`
- `audit/repair_wall_gap.py NEW_OUTPUT_DIRECTORY`로 재생성할 수 있다.
- `provenance.npz`에 V5까지의 원본/보간/평탄화 마스크와 `v6_filled`, `v6_before`, `v6_fit_rms`, `v6_reference_agreement`를 함께 보존했다.
- 새 높이는 센서 관측이 아니라 추정이다. RGHM v1에 inference 표지가 없으므로 지도와 provenance를 함께 보존해야 한다. 새 레코드의 분산 0.0009와 snapshot 시각은 직렬화용 값이며 검증된 불확실성·새 관측 시각을 뜻하지 않는다.
