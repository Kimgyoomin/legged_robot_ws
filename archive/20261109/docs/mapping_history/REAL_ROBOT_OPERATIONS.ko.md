# RUBI 실제 로봇 통합·지도 배포안

2026-10-14 · 코드 감사 및 운용 설계 · **실기 검증 전**

목표는 `SLAM 지도 제작 → 높이 지도 제작·QC → 높이 기반 전역 경로 계획`이다.
경사로/단차 분류, semantic, 영상 인식, foothold plane segmentation은 납품 필수 기능에 넣지 않는다.
잔노이즈는 실제 경로의 통행 판정과 비용을 불안정하게 만들지 않는 범위에서 허용한다.
높이 변화·미관측 영역·통행 한계는 계속 보존한다. 지형 이름을 분류할 필요는 없다.

## 1. 이번에 확인한 코드와 범위

- 실제 장비 기준 소스: `navigation_real`, `jetson/260929`, commit `cb2cdf3738036b4aa62e0bf3befe3b3e5a5bc4d2`.
- 높이 planner: 별도 `feature/CoT-based-cost`의 `rubi_heightmap_step_wavefront_planner`. Jetson 브랜치에는 없다.
- mapper: 현재 개발 PC의 `rubi_height_mapping` 0.3.0과 `rubi_global_heightmap_wrapper`. 아직 커밋하지 않은 수정도 포함한다.
- Jetson: Orin NX 16 GB / 25 W. JetPack·실제 설치 SDK·펌웨어·현재 실행 바이너리는 미확인이다.
- 소스는 읽기 전용 사본으로 비교했다. 실행 중인 로봇, navigation checkout, driver는 바꾸지 않았다.

Git에 저장된 소스와 Jetson에 설치된 바이너리가 같다는 보장은 없다. 배포 시 commit뿐 아니라
실제 설치 경로, 라이브러리 버전, 설정 파일 해시를 함께 기록한다.

## 2. MID-360S 교체 판단

**사용 가능한 센서다. 다만 기존 설치를 무수정으로 재사용할 수 있다는 뜻은 아니다.**

| 항목 | 확인 결과 | 적용 방향 |
|---|---|---|
| 외형·시야 | 두 모델 모두 65×65×60 mm, 265 g, 수평 360°, 수직 −7°~52° | 기존 설치를 이어갈 수 있는 기본 조건은 같다. 브래킷과 배선 실물 확인은 필요 |
| 내부 IMU 위치 | 두 매뉴얼 모두 LiDAR 좌표에서 (11.0, 23.29, −44.12) mm | 기존 보정의 근거는 유지된다. 회전한 출력 좌표계 기준으로 외적 파라미터의 부호 확인 |
| ROS driver | Jetson 소스는 1.2.4 기반 + RUBI 수정. 공식 MID-360S 지원은 1.2.5부터 | MID-360S 지원 버전으로 갱신하고 RUBI IMU 회전 수정을 이식 |
| SDK2 | 저장된 submodule `f5d9375…`는 1.3.1, MID-360S 타입 포함 | SDK도 무조건 구형이라고 볼 수 없다. 실제 설치본과 driver 조합을 확인·고정 |
| 설정 | 공식 `MID360s_config.json`, 모델 키 `Mid360s` | 기존 `MID360_config.json`의 이름만 바꾸지 말고 새 모델 설정에서 IP·회전을 적용 |
| 출력 형식 | 기존 `msg_MID360_launch.py`는 이름과 달리 `xfer_format=0`(PointCloud2). 공식 `msg_MID360s_launch.py`는 1(CustomMsg) | launch 이름이 아니라 실제 메시지 타입으로 연결. 서로 바꾸면 FAST-LIO 입력 설정도 같이 변경 |

참고: [MID-360 사양](https://www.livoxtech.com/mid-360/specs),
[MID-360S 사양](https://www.livoxtech.com/mid-360s/specs),
[MID-360 매뉴얼](https://terra-1-g.djicdn.com/851d20f7b9f64838a34cd02351370894/Livox/Livox_Mid-360_User_Manual_EN.pdf),
[MID-360S 매뉴얼](https://terra-1-g.djicdn.com/65c028cd298f4669a7f0e40e50ba1131/Mid-360S/UM/20260601/Livox_Mid-360s_User_Manual_en.pdf),
[driver 변경 이력](https://github.com/Livox-SDK/livox_ros_driver2/blob/21445540f0d100dc86a7e6df312dd70bbdb4afdf/CHANGELOG.md),
[SDK 변경 이력](https://github.com/Livox-SDK/Livox-SDK2/blob/c0796f04c143143899c87a773d9f6b7136453c0b/CHANGELOG.md).

### 거꾸로 장착한 센서

Jetson 소스에는 JSON `roll=180`과 IMU gyro/acceleration 회전 패치가 있다.
점군은 `LidarPubHandler::ProcessCartesianHighPoint()` 등의 driver 코드에서 같은 회전 행렬을 적용한다.
센서에 설정값을 보내는 호출만 보고 센서 내부에서 점을 회전한다고 해석하면 안 된다.
MID-360S 매뉴얼은 extrinsic을 센서가 저장만 한다고 설명한다.

기존 방식을 이어가되 **점군·IMU에 같은 회전을 각각 한 번만 적용**하는 구성을 권장한다.
회전한 frame은 축 방향만 바뀐 것이며 base_link와 원점이 같아지는 것은 아니다.
센서/IMU 원점 사이의 병진, 로봇 기준 장착 위치는 별도로 유지한다.
driver에서 회전한 뒤 `livox_axis_flip`이나 mount TF로 다시 180° 회전시키지 않는다.

현재 공식 driver의 IMU 수신부는 gyro/acc를 그대로 복사한다. 따라서 공식 driver로
교체하면서 기존 IMU 회전 패치를 빠뜨리면 점군과 IMU의 축이 달라질 수 있다.
단일 센서 설정에 맞춰 회전 행렬을 첫 IMU 수신 전 준비하고, 값·시각을 검사한 뒤 사용할 것.
기존 `livox_axis_flip`은 PointCloud2 전용이고 covariance/orientation 변환까지 완성된
범용 대체품은 아니므로 그대로 중복 실행하지 않는다.

참고: MID-360S 매뉴얼의 거꾸로 장착 조건은 장착면과 지면 사이 0.5 m 이상이다.
이 조건은 mapper의 `min_range=0.6`과 다른 의미다.

## 3. 기억한 timestamp 수정의 실제 위치

| 코드 | 현재 동작 | 통합 전에 할 일 |
|---|---|---|
| GenZ `1262a06` | `use_sensor_stamp=true`, PointCloud2 header 시각으로 odom/localization 발행 | 이 정책 유지. map→odom TF는 기본 1초 미래 시각으로 발행하므로 센서 pose 시각과 구별 |
| FAST-LIO localization `preprocess.cpp:459` | `lidar_type=4`는 MID360 handler. 점별 timestamp 필드 없이 방위각과 고정 각속도로 시간을 추정 | 비반복 스캔의 실제 점별 시각 사용으로 보완. YAML의 “any other input” 주석과 실제 코드가 다름 |
| FAST-LIO `timestamp_unit` | localization 쪽 선언·읽기가 주석 처리됨 | YAML 값만 조정해서 위 문제를 해결할 수 없음 |
| FAST-LIO `transform_fusion.py:91` | 과거 odom pose를 `/localization`에 현재 시각으로 재발행 | 원본 pose 시각 유지. 중복 pose 재발행 정책 정리 |
| 개발 PC의 같은 파일 | 원본 odom stamp 보존·header deep copy·중복 억제 수정이 작업 트리에 있음 | Jetson 브랜치에는 아직 없음. 검증한 수정만 별도로 이식 |
| Livox driver의 비동기화 fallback | PTP/GPS가 아니면 패킷 수신 시 host clock 사용 | 실제 time_type 확인. 센서 공통 시계를 ROS 시계와 일관되게 연결하고 LiDAR/IMU 수신 지터를 측정 |
| `FAST_LIO_ROS2` mapping | 별도 코드에도 MID360 방위각 시간 추정이 남음. 외적 병진은 localization 설정과 y/z 부호가 다름 | 지도 제작용과 localization용 설정을 같은 장착 규약으로 통일 |

공식 매뉴얼의 MID-360S는 비반복 스캔이다. 점별 시각이 있는데 방위각으로 대신 추정하는
현재 경로는 실기 전 보완 대상이다. 이것만으로 이전 실험의 왜곡 원인을 전부 입증한 것은 아니다.
`time_sync_en=true`를 켜거나 모든 메시지에 `now()`를 붙여 우회하지 않는다.
한 센서의 LiDAR/IMU는 공통 장치 시각을 유지하고, ROS 시계와의 연결은 하나의 규약으로 처리한다.
PTP는 가능한 방법 중 하나이며, 설치했다고 실제 동기화가 성립했다고 간주하지 않는다.

권장 구현 순서는 기존 PointCloud2 경로를 유지하며 driver의 `timestamp`를 읽고,
`(point_time − scan_start_time)`을 FAST-LIO가 기대하는 ms 단위로 변환하는 것이다.
이 브랜치 driver의 PointCloud2 필드는 절대 ns 값이다. 비유한 값·음수 상대 시각·범위를 벗어난
scan duration은 거부하고 새 driver에서도 단위를 재검증한다. CustomMsg 경로도 대안이지만
변환기와 GenZ/mapper 입력을 함께 정리해야 하므로 단순히 launch만 교체하지 않는다.

### PointCloud2를 선택한 Nav2 호환성 배경

Jetson 브랜치의 `nav2_rubi_dwb.yaml`에서는 local/global costmap 모두
ObstacleLayer가 `/livox/lidar`의 PointCloud2를 받고, StaticLayer는 `/map`의
OccupancyGrid를 받는다. 사용자가 기억한 PointCloud2 선택은 이 구성과 일치한다.
local/global별로 CustomMsg와 PointCloud2를 나눠 받는 설정은 아니다.
Nav2 Humble 기본 ObstacleLayer는 CustomMsg를 직접 받지 않는다.

따라서 MID-360S 통합에서도 **PointCloud2 인터페이스를 유지**한다. 기존 Livox PointCloud2에는
XYZ/intensity/tag/line 외에 점별 `timestamp`가 이미 있다. FAST-LIO에서 이 필드를 읽어
motion deskew에 사용하는 부분을 보완하면 Nav2와 GenZ 입력 형식을 유지할 수 있다.
중간 필터·변환기가 XYZ만 재생성해 timestamp를 삭제하지 않도록 확인한다.
드라이버의 ROS header stamp와 점별 timestamp는 구분해서 검증한다.

CustomMsg를 선택하는 대안도 가능하지만 FAST-LIO용 CustomMsg와 Nav2/GenZ/mapper용
PointCloud2를 별도 토픽으로 제공해야 한다. 기존 토픽을 CustomMsg로 교체하거나 같은 토픽에
두 타입을 섞는 방식은 사용하지 않는다. 현재 구성에서는 이런 변환기를 추가할 이유가 없다.

근거: `nav2_rubi_dwb.yaml` 185–186행 / 262–263행,
[Nav2 Humble ObstacleLayer](https://github.com/ros-navigation/navigation2/blob/humble/nav2_costmap_2d/plugins/obstacle_layer.cpp),
[StaticLayer](https://github.com/ros-navigation/navigation2/blob/humble/nav2_costmap_2d/plugins/static_layer.cpp),
[Livox PointCloud2 필드](https://github.com/Livox-SDK/livox_ros_driver2/blob/21445540f0d100dc86a7e6df312dd70bbdb4afdf/README.md).

## 4. 납품 구성

| 구성 요소 | 현재 코드 | 배치·역할 |
|---|---|---|
| 센서 수신 | MID-360S 지원 `livox_ros_driver2` + SDK2 + RUBI 회전 수정 | Jetson, PointCloud2와 IMU, 실제 점별 시각 유지 |
| Localization | `fast_lio_localization` **또는** `genz_icp` | Jetson. 기본 운용에서는 하나만 TF 소유. 비교 시 namespace/TF 분리 |
| 높이 지역지도 | `rubi_height_mapping` | Jetson, CUDA/CuPy. 작은 지역 지도와 거리 필터 |
| 제작 중 전역 누적 | `rubi_global_heightmap_wrapper` | 조사 주행·bag 재생 시 사용. 주행마다 새 전역지도 제작 불필요 |
| 오프라인 후처리 | `rubi_heightmap_postprocess` | 작업 PC, NumPy/SciPy. 보수적 노이즈 감소와 QC |
| 정적 지도 공급 | 기존 Nav2 map_server + **추가할 frozen height map server** | Jetson, 저장 지도 로드. height server는 CPU만으로 가능 |
| 전역 경로 | `rubi_heightmap_step_wavefront_planner`의 검증한 알고리즘 | 높이+2D costmap 입력. 비교용 launch를 Nav2 planner처럼 즉시 쓸 수는 없음 |
| 추종·로봇 통신 | 기존 Nav2 controller / `pongbot_lcm` | 유지. 새 전역 경로의 연결 규약만 확인 |
| 통합 실행 | **추가할 `rubi_navigation_bringup`** | sensor/localization provider/site bundle을 한 곳에서 지정 |

`rubi_mapping_ws`는 workspace 이름이다. 이 폴더 전체를 옮기는 대신 위 ROS 패키지를
별도 소스 저장소로 유지하고 같은 배포 workspace에서 빌드한다. `build/install/log`, Gazebo,
FastDEM, 영상·semantic, 원본 `elevation_mapping_cupy`를 런타임에 포함할 필요가 없다.
현재 높이 패키지는 원본 패키지의 런타임 의존성 없이 동작하지만 CuPy/CUDA는 필요하다.
Jetson의 JetPack·CUDA·Python 조합 확인 전 특정 wheel 설치 명령을 확정하지 않는다.

### 현재 navigation에서 주의할 연결

`nav2_rubi_dwb.yaml`의 전역 planner는 `pongbot_global_planner/AstarPlanner`이며 높이 입력이 없다.
같은 파일의 controller odom 기본값은 `/genz/odometry`다. FAST-LIO를 선택하면 `/Odometry`로
맞춰야 한다. 일부 이전 launch에는 회전 전 FRD frame을 가정한 TF와 개인 PC 절대경로가 남아 있다.
실기 통합 launch는 하나를 기준으로 만들고, 예전 launch들을 함께 실행하지 않는다.

현재 높이 comparison 설정은 `/fastdem/mapping/cloud_global`, frame `map`,
`max_heightmap_age_s=2.5`, `max_input_stamp_skew_s=2.0`을 사용한다.
RUBI 출력을 연결하려면 토픽도 바꾸고, 정적 지도와 실시간 지도 시각의 의미도 구분해야 한다.
comparison launch는 비교·시각화용이며 Nav2 action/controller 통합 검증을 대신하지 않는다.

2D Occupancy에서 이미 occupied로 막은 셀은 height cost를 추가해도 자동으로 통과 가능해지지 않는다.
PCD에서 2D 지도를 만드는 높이 범위와 로봇 충돌 기준도 확인하여, 보행 가능한 낮은 지형을
높이 planner가 평가할 수 있도록 한다. 벽·기둥의 충돌 정보와 unknown 처리는 유지한다.
이는 지형 종류 분류가 아니라 두 지도 입력의 통행 판정이 충돌하지 않도록 맞추는 작업이다.

## 5. 두 가지 운용 모드

**지도 제작:** 센서 기록 → SLAM/trajectory QC → 동일 map 좌표에서 height fusion →
후처리 후보 비교 → PCD·2D·height 정합 QC → 버전별 bundle 저장.
실시간 생성은 현장 확인용으로 사용하고, 품질이 필요하면 같은 bag을 작업 PC에서 재처리한다.
후처리만으로 pose drift를 제거할 수 없으므로 넓은 휨은 보정된 궤적으로 재생성한다.

**일상 주행:** MID-360S → 선택한 localization → 저장 지도 로드 → global planning →
기존 controller. 동시에 작은 실시간 지역 height map을 유지한다.
frozen 전역 높이를 주행마다 덮어쓰지 않으며, 새 장애물·바뀐 지형은 지역 관측으로 경로를
재평가한다. 전역 지도의 재제작은 별도 버전으로 배포한다.

정적 지도 공급과 global planning만으로는 CuPy가 필요하지 않다. 실시간 지역 height mapping을
함께 켜는 구성이므로 Jetson에는 여전히 CUDA/CuPy가 필요하다. RViz·오프라인 처리·상시 전역
재누적을 빼고 측정하면 된다. Orin 25 W 성능을 desktop 결과로 보장할 수는 없다.

## 6. PCD·Occupancy·Height 좌표 정합

### 기준

이미 정합한 PCD와 Occupancy 쌍을 기준으로 삼아 **PCD의 중력 방향을 확인한 `map` 좌표계**에
height map을 만든다. 지도 세 종류의 원점 숫자가 모두 같을 필요는 없다.
Occupancy YAML의 origin은 이미지 격자의 배치이며 PCD의 원점과 별개다.
각 셀을 공통 map의 실제 XY로 변환했을 때 위치가 맞아야 한다. height의 Z 기준도 기록한다.
PCD 자체가 기울어져 있다면 이름만 `map`으로 쓰지 말고 gravity 기준 변환을 정의하고,
기존 2D 지도와 함께 일관되게 적용한다.

### 가장 간단한 제작 절차

1. 기존에 정합한 **같은 현장의** PCD+PGM/YAML을 선택하고 해시를 고정한다.
2. 그 PCD에 localization한 3D pose와 센서 장착 변환으로, scan 시각의 점을 `map`으로 변환한다.
3. 이 좌표에서 5 cm height map을 누적한다. 처음부터 같은 map에서 생성했으므로 별도 ICP 없이
   정합되는 구조다. 그래도 위치·높이 오차는 QC로 확인한다.
4. 최적화한 SLAM 궤적으로 재생성할 경우 PCD 생성에 쓴 좌표 변환을 궤적에도 동일하게 적용한다.
   online 누적 결과가 loop closure 후 자동으로 재배치되는 기능은 현재 wrapper에 없다.
5. 원본 보존 → 보수적 후처리 → 통과한 후보만 frozen bundle로 확정한다.

### 이미 다른 좌표에서 만든 height map

같은 장소에서 얻은 원본 3D 점군과 기준 PCD를 이용해 `T_map_from_height`를 추정한다.
평평한 바닥만으로 XY/yaw가 충분히 결정된다고 가정하지 않는다. 모서리·기둥·벽 등 공유 구조를
여러 위치에서 사용하고, fitting에 쓰지 않은 위치로 검증한다.

`p_map = T_map_from_height × p_height`를 적용한 뒤 목표 격자로 다시 만든다.
평행 이동·yaw·z offset과 회전 방향을 명시하며, 기울기까지 있으면 3D 변환 후 높이 격자화한다.
단순히 frame_id나 YAML origin을 바꾸는 것은 회전·높이 보정을 대신하지 못한다.
가장 좋은 방법은 변환한 원본 점군을 다시 융합하는 것이다. 높이 raster만 재표본화한다면
단차 양쪽을 평균내지 않고 충돌·미관측 셀을 명시적으로 처리한다. 단일 rigid transform으로
지역마다 다른 pose drift/휨을 고칠 수는 없다.

이번 대화에서 저장한 Gazebo `world` 지도는 실제 건물 PCD와 다른 장소다.
이 둘의 정합행렬은 계산하지 않았다. 현재 저장된 실기 PCD/Occupancy 쌍의 실측 정합 정확도도
이번에는 다시 검증하지 않았다. 실제 survey 데이터가 생기면 위 절차로 QC한다.

### 배포 단위

```text
site_bundle/<site>/<revision>/
  manifest.yaml
  localization.pcd
  occupancy.yaml
  occupancy.pgm
  height_raw.rghm
  height_frozen.rghm
  height_surface.pcd
  processing_metrics.json
  alignment_qc.json
```

`manifest.yaml`에 site/revision, 각 파일 SHA-256, map frame, 중력·Z 기준, resolution·origin,
좌표변환 방향·행렬, 생성 시각, source bag/trajectory, 후처리 파라미터·버전,
QC 결과를 기록한다. 미확인 변환에는 identity를 임의로 넣지 않는다.
이름이 같은 `map`이라도 다른 버전의 PCD와 height를 혼용하지 않는다.
[작성용 명세](site_bundle.template.yaml)는 실행 설정이 아니며 미확인 항목을 null로 남겼다.

## 7. frozen height map server에 필요한 최소 기능

기존 wrapper의 save/load와 후처리 preview는 원본 관측 시각을 보존한다.
따라서 예전 지도를 불러와 현재 comparison planner에 연결하면 2.5초 age/2초 skew 조건에 걸린다.
토픽 이름 변경이나 무조건 `now()`로 바꾸는 것만으로 마무리하지 않는다.

추가할 server는 CPU로 검증한 bundle을 읽고, 선택한 최종 height만 고정 map frame에 발행한다.
지도 ID·해시·생성 시각·QC 상태를 함께 제공한다. 배포 소비자는 명시적인 static map 모드에서
관측 시각 age/skew 대신 bundle 일치와 로드 상태를 확인한다. late subscriber용 QoS와 재시작 후
재발행도 검증한다. 실시간 지역지도에는 기존 age/timeout 검사를 유지한다.
이는 제안된 계약이며 아직 구현된 launch/파라미터 이름으로 안내하지 않는다.

## 8. 실행과 검증 순서

1. MID-360S 지원 driver와 기존 회전 처리를 통합한다. 정지/회전 bag으로 점군·IMU 축, 시계,
   점별 시각 범위, 실제 frame을 확인한다. 최소거리 0.6 m는 시작값이며 보행 중 바닥 손실도 측정한다.
2. FAST-LIO 점별 시각/pose stamp를 보완하고 GenZ를 대안 provider로 유지한다.
   지도용 global pose는 3D를 유지한다. controller용 2D pose를 height fusion에 넣지 않는다.
3. 같은 bag에서 mapper를 실행해 평지 잔차·실측 높이·관측 누락을 확인한다.
   mapper 자체는 점별 deskew를 하지 않는다. 보행 중 데이터에는 검증한 deskew 입력을 사용한다.
   FAST-LIO의 `/cloud_registered_body`는 그 후보지만 시간 처리 보완 후 확인해야 한다.
   이 토픽은 `body` 좌표이므로 원점이 다른 거리 필터와 frame 설정도 재검증한다.
4. site bundle과 frozen server/static planner 입력을 연결한다.
5. 현재 높이 planner를 비교 화면에서 먼저 검증하고, 선택한 한 경로를 Nav2 controller에 연결한다.
6. Jetson 25 W에서 localization+지역지도+planning을 함께 실행해 CPU/GPU/RAM, 온도,
   처리 지연 P95/max, drop·queue 증가, 재계획 응답을 측정한다.

QC는 바닥의 국소 평면 잔차와 넓은 범위 Z 변형을 따로 기록한다. 벽/기둥 XY 정합,
알려진 높이의 오차, unknown 비율, 반복 주행 경로의 통과/차단 일관성을 포함한다.
5 cm cell이라고 높이 정확도 허용치를 자동으로 5 cm로 정하지 않는다.
허용치는 planner의 실제 문턱과 여유로 정하고 `alignment_qc.json`에 수치와 평가 위치를 저장한다.
GT로 확인한 수치는 시뮬레이션 증거이며 실기 합격 수치로 재사용하지 않는다.

현재 mapper는 별도 설치 후 아래와 같은 연결 형태를 지원한다. **실기 전 검증용 예시**이며
driver/frame/time 보완을 완료하기 전 정상 품질을 보장하는 일괄 실행 명령은 아니다.

```bash
# corrected PointCloud2와 map 기준 TF가 이미 준비된 경우
ros2 launch rubi_height_mapping mapping.launch.py \
  use_sim_time:=false rviz:=false \
  input_scan:=/livox/lidar map_frame:=map base_frame:=base_link \
  min_range:=0.6 max_range:=3.0
```

직접 odometry를 사용할 때에는 `header.frame_id=map`인 global pose를 지정한다.
GenZ의 후보는 `/genz/localization_odom`; FAST-LIO의 후보는 **stamp 수정 후** `/localization`이다.
`/Odometry`는 `camera_init` 기준이므로 `map_frame:=map`에 그대로 넣으면 안 된다.
frame 변환과 pose 시각을 먼저 검증하고, 장착 변환은 `/tf_static`으로 제공한다.

## 9. 이번 결과와 다음 구현 범위

이번에는 branch/공식 자료 감사, 배포 구성 명세, 좌표 정합·QC 절차와 기존 설치 문서의
입력 설명을 정리했다. MID-360S 포팅, frozen server, planner 실행 연동, 실기 정합은 아직 구현·검증
완료가 아니다. 다음 구현 단위는 **센서·시간 계약 정리 → frozen map 공급 → planner 연결**이다.
높이 알고리즘을 새로 만드는 작업이나 경사로/단차 분류 연구를 추가하지 않는다.
