# 2026-11-09 납품 정리 보관함

삭제한 데이터는 없습니다. `move_manifest.json`에 원래 경로, 보관 경로, 이동 전후 SHA256을 기록했습니다. 지도 수치는 보존했고 YAML/JSON의 상대 참조 및 일부 문서·개발 도구 경로만 갱신했습니다.

- `maps/`: 중간 후처리 지도, 이전 원본, 합성·시뮬레이션 fixture. 현재 최종/원본은 workspace `maps/`에 있습니다.
- `optional_src/`: FAST-LIO 단독 mapping과 GenZ-ICP 대안. 기본 빌드에서 제외합니다. 필요하면 원하는 폴더만 `src/`로 복원해 빌드합니다.
- `validation/`, `tools/`: 과거 검증·성능 분석·개발 스크립트. `tools/` 일부는 ROS 노드를 실행하므로 용도를 확인하고 사용합니다.
- `backups/`: 변경 전 소스 백업.
- `docs/`: 과거 작업 기록.

현재 빌드: `colcon build --base-paths src ...`. `COLCON_IGNORE`가 보관함 중복 패키지 검색을 막습니다. 원래 경로 복원이 필요하면 manifest를 기준으로 복사하고, 지금의 파일을 덮어쓰기 전에 비교하십시오. Git commit/push는 자동으로 실행하지 않았습니다.
