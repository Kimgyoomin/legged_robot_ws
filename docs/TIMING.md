# Livox 입력 형식과 시간 처리

## 이전 작업에서 확인한 사실

Codex의 **FAST-LIO localization 진단** 기록(2026-07-23)에는 driver 기본 launch가 PointCloud2인데 FAST-LIO type 1이 CustomMsg를 구독하는 형식 불일치와, Nav2 장애물 입력의 PointCloud2 요구가 기록되어 있습니다. 그때 시뮬레이터 adapter는 CustomMsg를 발행하고 있어 하드웨어 driver와 실행 경로도 달랐습니다.

Git의 `1262a060aa5b23bf882fe3e0780da4e4e3d8f3f0` 커밋 제목은 `changed for TF / timestamp and other configs`이며, 실제 변경 대상은 **GenZ-ICP의 센서 timestamp 사용과 TF 정책**입니다. 현재 확인한 이 커밋을 FAST-LIO CustomMsg 문제 수정으로 설명할 수는 없습니다. 사용자가 기억하는 과거 해결 사건을 완전히 특정한 것은 아닙니다.

## 두 형식의 실제 차이

| 항목 | CustomMsg | 이 배포본 Livox PointCloud2 |
|---|---|---|
| ROS 타입 | `livox_ros_driver2/msg/CustomMsg` | `sensor_msgs/msg/PointCloud2` |
| 점 데이터 | x/y/z, reflectivity, line, tag, offset_time | x/y/z, intensity, line, tag, timestamp |
| 기준시간 | `header.stamp`, `timebase` | `header.stamp` |
| 점별 시간 | `offset_time`: 기준시간에서 상대 ns | `timestamp`: 절대 ns, FLOAT64 |
| FAST-LIO 내부 시간 | offset ns / 10⁶ → ms | (absolute ns − header ns) / 10⁶ → ms |
| Nav2 ObstacleLayer | 직접 입력 불가 | 직접 입력 가능 |

Nav2의 local/global costmap이 서로 CustomMsg/PointCloud2를 각각 요구하는 것은 아닙니다. 두 costmap의 ObstacleLayer가 PointCloud2를 받고, StaticLayer가 `/map`의 OccupancyGrid를 받는 구조입니다.

## 이번에 수정한 부분

복사한 기존 FAST-LIO `mid360_handler`는 PointCloud2의 실제 timestamp를 읽지 않고, 방위각과 `omega_l=3.61`로 상대시간을 만들어 사용했습니다. MID-360의 비반복 스캔에 이 회전식 LiDAR 가정을 적용하지 않도록 수정했습니다.

새 decoder는:

1. `x/y/z/intensity/tag/line/timestamp`의 형식과 데이터 경계를 검사합니다.
2. row padding을 포함한 PointCloud2 레이아웃을 읽습니다.
3. 절대 ns에서 header ns를 빼고 FAST-LIO 내부 단위인 ms로 변환합니다.
4. FLOAT64 epoch ns의 작은 반올림 오차만 허용합니다. 단위를 숫자 크기로 추측하지 않습니다.
5. 비정상 시간·누락 field·0.5초를 넘는 scan offset은 해당 scan을 거부합니다. 공개 launch는 10 Hz입니다.
6. 시간순으로 정렬한 후 tag/line·거리·downsampling을 적용합니다.

`preprocess.timestamp_unit` YAML 하나를 바꾸는 수정이 아닙니다. 기존 fork에서는 해당 parameter 선언/읽기가 주석 상태였습니다. 새로운 MID-360 경로의 시간 계약은 코드와 테스트로 고정했습니다. 이 decoder는 `FAST_LIO_LOCALIZATION2`와 `FAST_LIO_ROS2` 양쪽에 적용했습니다.

함께 수정한 사항은 빈 scan 또는 시간 역행 때 pointcloud queue와 time queue가 함께 제거되도록 한 부분, 선택적 time-sync 경로에서 seconds를 nanoseconds 생성자로 잘못 전달하지 않도록 한 부분입니다. `time_sync_en`은 기본 false이며 물리 LiDAR/IMU의 시간 동기화를 대신하는 기능으로 사용하지 않습니다.

## Timestamp를 보존하는 전체 흐름

```text
Livox packet clock
  ├─ PointCloud2: scan 시작 header + 각 점의 절대 ns
  └─ Imu: 해당 packet stamp
          ↓
FAST-LIO: IMU 구간과 점별 상대시간으로 deskew
          ↓
/cloud_registered_body: scan 끝 시각의 IMU-body 기준 점군
/Odometry: 같은 scan 끝 시각의 camera_init → body
          ↓
global transform fusion
/localization: 같은 sensor stamp의 map → body
          ↓
height mapper: scan stamp에서 pose 보간 + 고정 장착 변환
          ↓
local GridMap → global wrapper → 원래 snapshot stamp로 저장
```

`transform_fusion.py`는 복사해 둔 PC 수정본의 sensor stamp·deep-copy 정책을 유지합니다. 전역 정합이 아직 준비되지 않았을 때 임시 identity를 유효한 `/localization`으로 발행하지 않도록 추가 변경했습니다. Frozen map을 다시 발행한다고 취득시간을 현재로 바꾸지 않습니다.

## 180° 장착 처리

현재 조건의 회전은 `R = Rx(π) = diag(1,-1,-1)`입니다.

```text
point_corrected = R point_native
gyro_corrected  = R gyro_native
accel_corrected = R accel_native
```

새 driver patch는 해당 센서 handle의 설정을 잠금 하에 읽어 IMU를 회전합니다. 첫 pointcloud가 와야 초기화되는 전역 회전행렬에 의존하지 않습니다. 설정이 아직 없는 IMU packet은 발행하지 않습니다. IMU에는 translation을 적용하지 않습니다. SDK가 주는 raw gyro/acceleration을 처리하며, 존재하지 않는 IMU orientation을 만들어내지 않습니다.

LiDAR origin과 IMU origin은 여전히 다릅니다. 현재 장착 모델에서 FAST-LIO `extrinsic_T=[-0.011,+0.02329,-0.04412]`, `extrinsic_R=I`와 `frames.yaml`의 translation을 함께 사용합니다. 새 bracket이나 센서 기준점이 달라지면 실제 치수를 다시 확인해야 합니다.

## 검증 범위

이번 테스트는 절대 ns와 CustomMsg 상대 ns의 일치, packed field/row padding, 단위 오류 거부, IMU의 180° 회전과 벡터 크기 보존을 검사합니다. 실제 MID-360S의 packet clock/PTP·송수신 지연·IMU 교정은 실물 확인 대상입니다. CustomMsg가 본질적으로 잘못된 형식이라는 결론은 내리지 않습니다.

근거: [공식 driver 고정 소스](https://github.com/Livox-SDK/livox_ros_driver2/tree/21445540f0d100dc86a7e6df312dd70bbdb4afdf), [새 시간 decoder](../src/FAST_LIO_LOCALIZATION2/include/livox_time.hpp), [실제 입력 handler](../src/FAST_LIO_LOCALIZATION2/src/preprocess.cpp), [회전 patch](patches/livox_driver.diff).
