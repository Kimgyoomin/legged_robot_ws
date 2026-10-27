# 02. 전방 ±60° 비대칭 장애물 필터

[문서 처음으로](README.md)

## 위치: Livox driver가 아닌 navigation 전처리

현재 구현은 [obstacle_cloud.cpp](../../src/legged_navigation/src/obstacle_cloud.cpp)의 C++ 노드다. [SectorFilter](../../src/legged_navigation/include/legged_navigation/sector_filter.hpp)가 방향별 최소 거리를 판정한다. `legged_range_trial`은 이 실행 파일에 별도 YAML을 넣는다.

```text
/livox/lidar_PointCloud2 (navigation의 원시 입력)
  └─ finite / 형식 확인 → 고정 TF → 방향별 거리 → BODY 높이
      ├─ /legged/obstacles: 원본 sensor frame·xyz·timestamp·추가 fields 보존
      └─ /legged/collision_points: 선택한 점의 xyz를 base_link로 변환

/livox/lidar (CustomMsg) + /livox/imu → FAST-LIO (별도 입력)
```

navigation 필터는 driver 출력이나 FAST-LIO 입력을 바꾸지 않는다. FAST-LIO의 `preprocess.blind: 0.5`와 obstacle filter의 0.3m는 서로 다른 처리 경로의 값이다. driver/시뮬레이터가 아예 발행하지 않은 가까운 점은 후단 필터로 복원할 수 없다.

## 좌표와 거리의 정확한 정의

센서 좌표의 점을 `p_s=(x,y,z)`, 고정 장착 변환을 `T_base_sensor=(R,t)`라 하면:

- **거리:** `r²=x²+y²+z²`. 센서 원점부터의 **3D 거리**다.
- **전방 각도:** `q=R p_s`의 XY 방향이 base_link +X에서 ±60° 안인가를 판정한다. 각도 계산에는 translation `t`를 더하지 않으므로 부채꼴 꼭짓점은 센서 원점이다.
- **높이:** `(R p_s+t).z`. 이때는 translation까지 포함한 base_link 기준 높이다.
- 동적 `map→body` TF를 높이 필터에 넣지 않는다. `/tf_static`의 고정 장착 chain만 받는다.

```text
                         base_link +X 방향
                               ↑
                     0.3m부터 허용하는 전방
                         \   ±60°   /
                          \        /
                           ● 센서 원점
                 나머지 방위는 1.0m부터 허용
                   모든 방향에서 최대 5.0m
```

이 도식의 0.3m는 로봇 앞면에서 장애물까지의 간격이 아니다. 센서 장착 위치와 몸체 크기를 더해서 실제 여유를 해석해야 한다.

## 현재 두 프로필

| 파라미터 | 기본 [range.yaml](../../src/legged_range_trial/config/range.yaml) | 비교 [range15.yaml](../../src/legged_range_trial/config/range15.yaml) |
|---|---:|---:|
| `front_half_angle_deg` | 60° | 60° |
| `front_min_range` | .3m | .3m |
| `min_range` (나머지 방향) | 1.0m | 1.5m |
| `max_range` | 5.0m | 5.0m |
| `base_frame` | base_link | base_link |
| `min_body_height` | −.20m | −.35m |
| `max_body_height` | +1.50m | +1.50m |

**현재 두 YAML은 거리만의 A/B 비교가 아니다.** range15에는 이전 높이 하한 −.35m가 남아 있다. 거리 효과만 비교하려면 별도 파일에서 높이 조건을 같게 맞춰야 한다.

조건은 inclusive다. 전방에서 `r≥.3`, 측후방에서 `r≥1.0`인 점을 허용하고 `r>5.0`은 버린다. `atan2` 대신 회전한 XY 벡터에 대해 `qx≥0` 및 `qx²≥cos²(60°)(qx²+qy²)`를 검사한다. 수직 ray처럼 XY 방향이 없는 점은 전방 특례를 받지 못한다.

## BODY 높이와 단차 높이는 다르다

`min_body_height=-.20`은 **몸체 원점보다 20cm 아래까지** 장애물 점으로 쓴다는 뜻이다. 20cm 또는 15cm 단차까지 통과 가능하다는 의미가 아니다.

예를 들어 지면이 몸체 원점 아래 40cm에 있으면 15cm 단차 윗면은 단순한 수평 자세 가정에서 BODY Z≈−25cm다. 이 점은 −20cm 하한에 의해 제외된다. 실제 몸체 높이·pitch/roll·보행·단차 진입에 따라 값이 달라지므로 이 숫자를 고정된 단차 높이 분류기로 볼 수 없다.

기록된 9cm 단차 표면은 BODY Z 약 −.35~−.24m였고, 하한을 −.20m로 올린 offline 비교에서 관련 809점이 제외되었다. 큐브는 94,413점 중 61,474점이 남았다. 이 결과는 해당 기록의 구분 성능이며, 모든 보행 자세에서 15cm 단차를 배제한다는 검증은 아니다. [높이 필터 결과](reports/height_band_20261027/offline_check.json)

map Z drift가 이 노드의 BODY 높이 판정을 직접 바꾸지는 않는다. 다만 localization 오차는 이후 점군의 map 투영·costmap과 로봇 위치의 상대 정합을 망가뜨릴 수 있다. 바닥이 흔들려 보인다고 모두 발 자가 반사라고 판단하지 말고 원시점·BODY Z·map 투영을 나누어 확인한다.

## Costmap 설정과의 연결

[costmap.yaml](../../src/legged_range_trial/config/costmap.yaml)의 local/global 모두:

| 항목 | 값 |
|---|---|
| observation topic | `/legged/obstacles` |
| marking / clearing | true / true |
| obstacle_min_range / max | .3m / 4.0m |
| raytrace_min_range | 0.0m |
| raytrace_max_range | local 4.0m / global 5.0m |
| min/max obstacle height | layer와 source 양쪽 −10000~+10000m |
| observation_persistence | 0.0s |
| expected_update_rate | 0.4s (Hz가 아닌 허용 간격) |

costmap의 최소 거리도 .3m로 낮춰야 필터가 살린 전방 점을 다시 버리지 않는다. map 기준 절대 Z 범위는 넓게 두어 BODY 높이로 선별한 점이 site의 Z 원점 때문에 다시 탈락하지 않게 했다. 높이 선별 책임이 전처리 노드에 있다는 설계다.

원래 센서 좌표를 보존하는 이유는 ray clearing의 **센서 원점**을 유지하기 위해서다. 단순히 cloud를 base_link로 바꾸어 costmap에 넣으면 관측 원점 의미도 달라질 수 있다.

`observation_persistence: 0`은 obstacle cell의 자동 만료 시간이 0이라는 뜻이 아니다. 기존 표시가 없어지려면 clearing 관측이 지나가거나 layer가 갱신·초기화되어야 한다. 필터가 많은 점을 버리면 marking뿐 아니라 clearing ray도 줄어 이전 흔적이 오래 남을 수 있다.

## Collision Monitor 입력과 freshness

두 출력은 같은 선택 점을 쓴다. Collision Monitor의 높이 범위는 −.35~+1.5m이지만 기본 range에서 이미 −.20m 미만을 버렸으므로 이를 복구하지 못한다.

`/legged/obstacles`는 원래 stamp로 발행한다. `/legged/collision_points`는 입력 나이가 기본 .4s를 넘거나 .1s 이상 미래이면 발행하지 않고 상태를 `stale_collision_cloud`로 바꾼다. 따라서 이 필터가 costmap 출력까지 모든 stale 점을 발행 전에 차단한다고 설명하면 부정확하다. 최종 watchdog은 별도로 입력·TF·명령의 freshness를 확인한다.

유효한 finite 원시점은 있지만 모두 필터로 제외된 scan은 빈 출력으로 전달한다. finite 원시점 자체가 없는 메시지는 `no_finite_returns`이며 정상 빈 공간 관측으로 보지 않는다. 상태 토픽은 `/legged/collision_input_state`다.

## 연산량과 관측 한계

- C++ **O(N)** 1회 순회, 제곱 거리·회전 내적·범위 비교다. 점별 sqrt/atan2·mesh collision·학습 모델을 쓰지 않는다.
- 출력 메시지 복사와 base_link 변환은 남은 점 수에 비례한다. 센서 발행률·point_step·DDS 복사 비용까지 포함해서 측정해야 한다.
- ±60° 밖 1~1.5m를 버리면 발 반사뿐 아니라 **실제 측후방 근거리 장애물도 잃는다**. 특히 후진·제자리 회전은 전방 감지 개선만으로 보장되지 않는다.
- 2026-10-20 의심점 424개는 1.88~3.99m여서 1.5m 필터에서도 남았다. 거리 조건만으로 모든 오검출을 해결하지 못한다.
- Jetson Orin NX 실측은 아직 없다. 직선/회전/단차 조건에서 callback 처리시간 p50/p95/p99, CPU, 입력-출력 나이, 출력점 수를 기록해야 한다. 숫자 임계값을 바꾸는 작업 자체는 새로운 알고리즘 단계를 추가하지 않는다.

원시점 분석·자가 반사·섹터 비교 근거는 [보고서 색인](06-validation.md)을 참고한다.
