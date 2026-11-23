# 운영 도구

- `navigation_env.sh`: ROS/DDS·CPU 기반 frozen navigation 환경.
- `height_env.sh`: CuPy 높이 지도 생성 환경.
- `replay_height_bag.py`: CustomMsg·IMU·PointCloud2 3개만 재생. `/clock` 생성, 기본 일시정지, 실제 로봇 domain과 분리.
- `build_height_from_bag.py`: 높이 생성용 bag 구성 도구.
- `align_height.py`, `package_site.py`, `replace_site_height.py`: 정합·지도 세트 구성·height 교체.
- `import_legacy_rubi_maps.py`: legacy PCD/2D 자료 가져오기.

성능 측정·과거 smoke/validation 도구는 [archive/20261109/tools](../archive/20261109/tools)에 옮겼습니다. 현재 실행법은 [HEIGHT_REPLAY.md](../docs/HEIGHT_REPLAY.md)를 우선합니다.
