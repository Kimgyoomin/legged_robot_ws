# 운용 도구

기본 절차는 [OPERATIONS.md](../docs/OPERATIONS.md)를 따릅니다.

| 파일 | 용도 |
|---|---|
| `render_delivery_report.py` | README·설치·운용·DDS·튜닝·검증 Markdown을 `docs/report.html`로 생성. 문서 재생성 시에만 `python3-markdown` 필요 |
| `height_env.sh` | GPU mapper용 CuPy/CUDA/PYTHONPATH 환경. `source`로 실행 |
| `navigation_env.sh` | ROS/domain/common DDS를 설정하는 편의 셸 환경 |
| `pcd_to_occupancy.py` | 정합된 PCD와 Height Map에서 hybrid용 2D YAML/PGM 초안 생성 |
| `package_site.py` | 기존 bringup용 정합된 PCD/2D/height 세트와 해시 패키징 |
| `replace_site_height.py` | 기존 site의 height 교체·manifest 갱신 |
| `import_legacy_rubi_maps.py` | navigation_real의 기록된 legacy 지도 세트 가져오기 |
| `align_height.py` | 이미 아는 좌표 변환으로 height 정렬 |
| `build_height_from_bag.py`, `replay_height_bag.py` | 기존 rosbag 기반 height 생성·검증 |

2D 투영 도구는 좌표 정합이나 inflation을 수행하지 않습니다. 입력 PCD와 RGHM이 같은 장소·좌표계여야 하며, 결과 `projection.json`에 입력 해시와 임계값을 남깁니다. 새 출력 폴더를 사용하고 RViz에서 지도 겹침을 확인합니다.
