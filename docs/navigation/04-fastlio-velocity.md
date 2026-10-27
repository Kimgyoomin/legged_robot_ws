# 04. FAST-LIO native velocity → Nav2

[문서 처음으로](README.md)

## 무엇을 넣는가

Nav2에 제공하는 것은 FAST-LIO가 IMU와 LiDAR로 추정한 **몸체의 실제 속도**다. `/cmd_vel`은 보행 정책에 보내는 요구 속도이므로 피드백을 대신할 수 없다. 현재는 위치를 수치미분하는 방식도 기본 사용하지 않는다.

```text
FAST-LIO state.vel / gyro / gyro bias
  → /fastlio/odometry_velocity (nav_msgs/Odometry, child_frame=body)
  → fastlio_controller_odom (장착 offset·축 변환)
  → /legged/controller_odom (child_frame=base_link)
  → controller_server + bt_navigator + watchdog

/Odometry: 기존 pose 출력 유지, 기존 twist는 변경하지 않음
TF: 기존 localization 체계 유지; velocity adapter는 TF를 발행하지 않음
GT: 검증 때만 사용, controller 입력에는 넣지 않음
```

## FAST-LIO 쪽 변경

패키지 디렉터리는 `src/FAST_LIO_LOCALIZATION2`, ROS package 이름은 `fast_lio_localization`이다.

| 파일 | 역할 |
|---|---|
| [include/velocity_output.hpp](../../src/FAST_LIO_LOCALIZATION2/include/velocity_output.hpp) | state velocity·gyro를 body twist와 covariance로 변환 |
| [src/IMU_Processing.hpp](../../src/FAST_LIO_LOCALIZATION2/src/IMU_Processing.hpp) | 마지막 propagation에 쓴 gyro 평균값 보관 |
| [src/laserMapping.cpp](../../src/FAST_LIO_LOCALIZATION2/src/laserMapping.cpp) | 별도 velocity Odometry 발행, bounded scan 처리 |
| [test/test_velocity_output.cpp](../../src/FAST_LIO_LOCALIZATION2/test/test_velocity_output.cpp) | 축 회전·bias·covariance 검증 |

`state.vel`은 `camera_init` 축에서 IMU/body 원점의 선속도다. `R`을 body→camera_init 회전이라 하면:

```text
v_body = Rᵀ v_world
ω_body = gyro_last_propagation − gyro_bias_posterior
```

새 Odometry는 pose·header를 복사하고 twist를 채운다. timestamp는 scan의 `lidar_end_time`을 유지한다. covariance는 회전 오차·속도·gyro bias에 대한 6×23 Jacobian과 gyro noise 근사로 계산한다. 이는 보정된 실측 오차 보증이 아닌 모델 불확실성이다. 기존 pose covariance도 publish 전에 채우도록 순서를 정리했다.

upstream에서 내부 velocity state가 존재해도 ROS Odometry의 twist를 자동으로 채우는 것은 아니다. 사용하는 fork의 `publish_odometry()`에서 명시적으로 대입해야 한다. 관련 배경은 [FAST_LIO issue #185](https://github.com/hku-mars/FAST_LIO/issues/185), [upstream laserMapping](https://github.com/hku-mars/FAST_LIO/blob/main/src/laserMapping.cpp)를 참고하되, 이 브랜치 동작은 로컬 구현을 기준으로 한다.

## body와 base_link 원점 차이

현재 simulation launch는 다음 고정 연결을 만든다.

```text
body → BODY: translation [-0.112, +0.02329, +0.07278] m, rotation identity
BODY → base_link: identity
```

원점이 다르므로 회전할 때 `ω×r` 선속도 보정을 해야 한다. source frame에서 본 base 원점 offset을 r, base→source 회전을 Q라 하면:

```text
v_base = Qᵀ (v_body + ω_body × r)
ω_base = Qᵀ ω_body
Σ_base = J Σ_body Jᵀ
```

단순히 `child_frame_id` 문자열만 바꾸거나 x/y를 회전하는 것으로 끝내면 원점 속도가 틀린다. [twist_frame.py](../../src/legged_range_trial/legged_range_trial/twist_frame.py)가 6×6 rigid-body 변환을 수행하고 [fastlio_controller_odom.py](../../src/legged_range_trial/legged_range_trial/fastlio_controller_odom.py)가 TF·메시지를 처리한다.

어댑터는 원래 stamp를 유지하고 pose도 base 원점으로 옮긴다. pose covariance는 대각 1e6으로 설정해 controller용 연결임을 명시한다. 이 출력을 다시 localization 측정으로 넣지 않는다. 기본 max_age=.5초보다 오래되거나 .05초 이상 미래인 입력, 없는 TF, 비유한 값을 거부한다. 위치 수치미분·시간 smoothing·새 TF 발행은 하지 않는다.

## 선택 스위치

| `fastlio_velocity` | `estimate_velocity` | Nav2 odometry 입력 |
|---|---|---|
| false | false | `odom_topic` (기본 `/Odometry`) |
| true | false | native state adapter → `/legged/controller_odom` |
| false | true | pose 회귀 미분 adapter → `/legged/controller_odom` |
| true | true | launch 오류: 두 입력을 동시에 사용하지 않음 |

현재 비교 명령에는 `fastlio_velocity:=true estimate_velocity:=false`를 명시한다. launch 기본값은 둘 다 false이므로 스위치를 생략하면 새 입력이 연결되지 않는다. `mode:=nav2`에서만 적용한다.

이전 pose 회귀 방식은 0.4초 window를 사용하여 잡음을 줄이지만 약 0.2초 수준의 중심 지연이 생길 수 있다. localization 보정 점프를 속도로 해석할 가능성도 있다. 비교용 코드로 남겨두었고 현재 native 방식에 추가로 겹쳐 적용하지 않는다.

## 입력 지연과 batch 처리

기존 50ms timer가 낮은 주기의 `/clock`과 결합했을 때 scan을 한 번에 하나만 소비하면 시작/재생 시 생긴 backlog가 남을 수 있었다. `config/mid360.yaml`은 다음을 설정한다.

```yaml
processing:
  max_scans_per_tick: 4
  max_batch_time_ms: 20.0
```

같은 scan을 FIFO로 처리하며 stamp를 현재 시각으로 덮거나 큐를 임의 삭제하지 않는다. 추가 scan을 처리하기 전에 wall 처리시간을 검사한다. 이미 시작한 한 scan을 중단하지 않으므로 **20ms hard deadline은 아니다**. 코드 fallback은 1 scan이며 다른 launch/config에 이 값이 자동 전파되지는 않는다.

## GT 비교 결과와 해석

시간 stamp를 맞추고 GT world 속도를 BODY/base_link 축으로 바꾼 뒤 비교했다. Gazebo P3D의 header/child 이름만 보고 twist 축을 추정하지 않고 구현과 대조했다. GT pose 미분은 offline 교차 점검용이며 controller에 넣지 않았다.

| 기록 | 유효 sim 시간 / 표본 | vx RMSE | yaw rate RMSE | 입력 age 중앙값 / p95 |
|---|---|---:|---:|---:|
| 저속 전진·후진·좌우 yaw 시험 | 51.002s / 511 | .03679m/s | .03167rad/s | .100 / .102s |
| 실제 자동 DWB 주행 | 36.502s / 366 | .02357m/s | .03438rad/s | .100 / .102s |

저속 시험 명령은 전진 .25m/s, 후진 −.15m/s, yaw ±.4rad/s였다. 첫 시험은 별도 FAST-LIO probe이며, 두 번째는 production stream의 실제 연결 확인이다. body→base 독립 계산과 어댑터 출력도 대조했다. [native velocity 보고서](reports/fastlio_velocity_20261027/report.html), [DWB 연결 보고서](reports/fastlio_velocity_20261027/dwb_live_report.html)

이 수치는 해당 PC 시뮬레이션 구간에서 controller 피드백으로 사용할 근거다. 모든 축의 동일 정확도, 실물 slip·충격·센서 드롭, 최대 속도, 긴 운행의 안정성 또는 Jetson 성능을 입증하지 않는다. 좋은 velocity RMSE와 경로 추종 성공률은 따로 측정한다.

## Jetson 비용

native state를 재사용하므로 새 속도 estimator를 실행하지 않는다. C++의 소형 행렬 연산과 메시지 1개, Python adapter의 TF lookup·6×6 연산·메시지 복사가 추가된다. 점군에 대한 두 번째 대형 최적화는 없다. 그러나 Open3D 전역 정합과 FAST-LIO 자체 CPU 사용량은 별개이며 Orin NX에서 실측해야 한다. adapter를 C++로 옮길지는 실제 callback 시간·age·CPU 측정 후 결정한다.
