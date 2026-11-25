# RUBI ROS → LCM 브리지

`ros2 launch pongbot_lcm joy_lcm.launch.py operation:=live`를 사용합니다. Nav2의 `/cmd_vel` (geometry_msgs/Twist)과 무선 조이스틱의 `/joy_wireless`를 받아 `robotlcm::joy_cmd`를 LCM 채널 `CMD_VEL`로 50 Hz 전송합니다. ROS 입력과 LCM 주소는 [운용 YAML](../navigation_dds/config/operation.yaml)의 `live.lcm`, 원본의 축 매핑·주기·버튼 처리는 [브리지 설정](config/joy_lcm.yaml)과 `src/joy_lcm_node.cpp`에서 확인합니다. 기존 `dds_profile:=navigation` 실행도 지원합니다.

송신 코드는 navigation_real 버전과 같습니다. 속도 필드는 마지막 Twist 값을 반복합니다. `deadman_timeout_s`는 joystick 수신 시간을 검사하며, Twist 갱신이 멈춰도 속도 필드는 마지막 값을 유지합니다. 조이스틱 축과 버튼은 별도 필드입니다. Navigation Mode 버튼은 무선 조이스틱 `buttons[1]`에서 LCM `buttons[3]`으로 전달되며, 보행 모드는 수신 제어기가 처리합니다.

로봇 LAN을 연결하고 workspace 루트에서 `./run_lcm_start.sh`를 한 번 실행합니다. 기본 인터페이스는 `enP7p1s0`, multicast 경로는 `239.255.76.67/32`입니다. 다른 NIC나 그룹을 사용하면 스크립트의 `LCM_IF`, `LCM_GROUP`과 YAML의 `lcm_url`을 맞춥니다. 실물 실행 순서는 [운용 안내](../../docs/OPERATIONS.md)에 있습니다.
