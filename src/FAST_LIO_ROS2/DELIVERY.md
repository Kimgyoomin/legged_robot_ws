# 납품용 FAST-LIO SLAM

기준: `~/ros2_ws/src/navigation_real/FAST_LIO_ROS2`, repository commit
`e11ddddd370f2ca1587947160832628209ad86cd`의 작업 트리(2026-11-23).
실제 이 장비에는 `~/robot_ws`가 없으므로 운용 중인 `ros2_ws` 소스를 기준으로 가져왔습니다.
추정 알고리즘·센서 extrinsic·CustomMsg 입력·속도 출력을 유지합니다.
납품 변경: C++17 옵션 정리, DDS launch 인자, PCD 출력 경로 인자 및 저장 실패 검출.
`doc/`의 논문/대형 GIF, 생성 로그·PCD는 복사하지 않았습니다. 원본 LICENSE와 소스 저작권은 유지합니다.

직접 실행/저장: workspace `docs/OPERATIONS.md` 참조.
`/map_save`는 1 Hz `/Laser_map` 누적 포인트를 저장합니다. 아직 포인트가 없으면 실패합니다.
SLAM과 localization은 동일 토픽·TF를 발행하므로 동시에 실행하지 않습니다.
