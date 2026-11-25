# navigation_dds — 실행과 설정 프로토콜

`navigation_dds`는 메시지 내용을 바꾸는 노드가 아니라 **Fast DDS transport XML을 해당 프로세스에 적용하는 launch/CLI 도구**입니다. 같은 장치의 SHM(shared memory)와 장치 사이 UDP를 함께 활성화합니다. Endpoint의 reliable/best-effort QoS, 메시지 timestamp, ROS domain은 별도 설정입니다.

## 1. 그대로 사용하는 명령

모든 터미널에서:

```bash
source ~/legged_robot_ws/install/setup.bash
export ROS_DOMAIN_ID=13
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0
```

| 실행 대상 | launch 인자 | 적용 |
|---|---|---|
| `livox_ros_driver2 msg_MID360s_launch.py` | `dds_profile:=livox` | Livox 전용 XML, 32 MiB SHM |
| `fast_lio mapping.launch.py` | `dds_profile:=navigation` | common, 32 MiB |
| `fast_lio_localization localization_nav_rubi.launch.py` | `dds_profile:=navigation` | 일반 프로세스 32 MiB, 큰 PCD 발행 프로세스 128 MiB |
| `rubi_height_mapping mapping.launch.py` | `dds_profile:=navigation` | common, 32 MiB |
| `legged_navigation height_hybrid_navigation.launch.py` | `dds_profile:=navigation` | common, 32 MiB |
| `legged_navigation height_only_navigation.launch.py` | `dds_profile:=navigation` | common, 32 MiB |

launch가 DDS 인자를 지원하지 않거나 단일 명령/서비스에 적용할 때:

```bash
ros2 run navigation_dds with_profile common \
  taskset -c 2,3,4,5 \
  ros2 launch fast_lio mapping.launch.py dds_profile:=default rviz:=false

ros2 run navigation_dds with_profile common \
  ros2 service call /rubi/global_elevation/save_map std_srvs/srv/Trigger '{}'
```

CLI 선택지는 **`common` / `large`**, 일반 launch 선택지는 **`navigation` / `default`**, Livox는 **`livox` / `default`**입니다. `dds_profile:=common`은 launch의 유효한 값이 아닙니다.

`default`는 **부모 셸에서 상속한 설정을 유지**한다는 뜻입니다. XML을 해제하거나 ROS 기본 설정으로 초기화하지 않습니다. launch의 명시적 profile이 외부 wrapper 설정보다 안쪽에서 적용되면 그 값이 우선합니다. 보통 위 표의 native 인자만 사용하면 됩니다.

## 2. 수정할 파일

| 파일 | 용도 |
|---|---|
| `src/navigation_dds/config/fastdds_common.xml` | 일반 노드 SHM 32 MiB + UDP |
| `src/navigation_dds/config/fastdds_large.xml` | 큰 PCD sender SHM 128 MiB + UDP |
| `src/navigation_dds/navigation_dds/profiles.py` | 프로필 이름→XML, 환경 검사, scoped launch group |
| `src/navigation_dds/scripts/with_profile` | 단일 명령을 지정 profile로 실행 |
| `src/livox_ros_driver2/config/fastdds_livox_shm.xml` | Livox 전용 32 MiB XML |
| `src/livox_ros_driver2/launch_ROS2/msg_MID360s_launch.py` | Livox profile 선택 및 driver 설정 |
| `src/FAST_LIO_LOCALIZATION2/launch/localization.launch.py` | large profile을 사용하는 PCD 발행 그룹 |
| `tools/navigation_env.sh` | 편의용 셸 환경(common XML). 직접 실행 가이드에서는 native profile 인자를 사용 |

XML의 `segment_size`는 **participant당 SHM 크기(byte)**입니다. common/Livox는 `33554432`, large는 `134217728`. `sendBufferSize`/`receiveBufferSize`는 UDP 소켓 버퍼 요청값이며 각각 `4194304`입니다. `useBuiltinTransports=false`와 SHM/UDP transport 참조는 묶어서 봅니다. 값을 변경하면 `profiles.py`의 표시값도 맞추고 **관련 프로세스를 재시작**합니다. 이미 실행 중인 DDS participant에는 소급 적용되지 않습니다.

두 환경변수 `FASTRTPS_DEFAULT_PROFILES_FILE`, `FASTDDS_DEFAULT_PROFILES_FILE`을 같은 XML로 설정합니다. `--symlink-install`에서는 대체로 소스 XML 수정이 보이지만 실제 선택 파일은 다음으로 확인합니다:

```bash
ros2 pkg prefix navigation_dds
ls -l "$(ros2 pkg prefix navigation_dds)/share/navigation_dds/config/"
```

소스 위치를 바꾼 뒤에는 재빌드합니다. 설치용 복사본만 고치면 다음 빌드에서 사라질 수 있습니다.

## 3. Domain·QoS·frame·토픽 약속

| 토픽 | 타입 / 소비자 |
|---|---|
| `/livox/lidar` | CustomMsg → FAST-LIO SLAM/localization |
| `/livox/lidar_pointcloud` | PointCloud2 → `costmap_sector_filter` |
| `/livox/imu` | Imu → FAST-LIO |
| `/cloud_registered_body` | PointCloud2, `body` frame → GPU height mapper |
| `/rubi/global_elevation/cloud` | live 누적 높이 surface → mapping 시각화 |
| `/legged/height` | 저장 RGHM에서 발행한 frozen PointCloud2, `map`, reliable + transient-local → 두 planner |
| `/livox/lidar_costmap` | BODY 높이/sector 필터 결과 → local costmap, hybrid global costmap |
| `/fastlio/odometry_velocity` → `/fastlio/odometry_base` | FAST-LIO 원점 속도 → base_link 원점 속도, DWB 입력 |
| `/map` | OccupancyGrid, hybrid만 사용 |
| `/plan`, `/cmd_vel` | Nav2 경로, 로봇 bridge가 받는 속도 명령 |

Domain은 모든 참여 프로세스가 일치해야 합니다. 다른 domain이면 발견되지 않습니다. 이 프로필은 `ROS_LOCALHOST_ONLY=1`을 거부합니다. Humble Fast DDS의 별도 SHM 생성 동작과 충돌하기 때문입니다. 다른 RMW나 localhost 전용 테스트는 `dds_profile:=default`와 그 환경에 맞는 설정을 사용합니다.

통신 상태 확인:

```bash
ros2 topic info /livox/lidar -v
ros2 topic info /livox/lidar_pointcloud -v
ros2 topic info /legged/height -v
printenv ROS_DOMAIN_ID RMW_IMPLEMENTATION ROS_LOCALHOST_ONLY
printenv FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
```

마지막 명령은 **현재 셸**의 환경입니다. launch 내부에만 적용한 값은 셸에 나타나지 않을 수 있으므로 launch의 DDS profile 로그도 확인합니다. Profile 크기를 키워도 잘못된 타입·QoS·frame·센서 timestamp는 해결되지 않습니다.

## 4. UDP 버퍼와 점검

필요한 장비에서 실행 전에 커널 상한을 조정할 수 있습니다:

```bash
sysctl net.core.rmem_max net.core.wmem_max
sudo sysctl -w net.core.rmem_max=16777216 net.core.wmem_max=16777216
```

장치 전체에 적용되며 재부팅하면 초기화됩니다. 16 MiB는 상한이고 XML의 4 MiB는 각 소켓의 요청입니다. 이는 `/dev/shm` 전체 용량이나 swap을 변경하는 설정이 아닙니다. 무조건 버퍼를 늘리기 전에 `ros2 topic info -v`, CPU 부하, launch 로그, `/dev/shm` 용량을 확인합니다.

`ros2` CLI가 이전 domain의 daemon 정보를 보여주면 동일 환경에서 `ros2 daemon stop`, `ros2 daemon start` 후 다시 확인합니다. 통신이 끊긴 상태에서 장애물 데이터가 없는 것을 주행 가능한 공간으로 해석하지 않습니다.
