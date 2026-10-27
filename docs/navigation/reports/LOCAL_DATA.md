# 로컬 원본과 보고서 배포본

이 디렉터리는 2026-10-27 navigation 인계용 보고서 묶음이다. HTML의 본문 수치는
원본 그대로이며, 현재 설정 안내 배너와 이동 가능한 상대 링크만 추가했다.
원본은 workspace의 `validation/<실험명>/`에 있다.

점군/bag/대형 dump, 전체 before/ 디렉터리 등은 이 묶음에 포함하지 않는다.
미포함 데이터 링크는 이 문서로 연결했다. 도식과 작은 수치 결과는 함께 포함한다.
그림만으로 원시 데이터 전체를 재분석할 수는 없다. 원본 보유 장비의 validation
디렉터리를 별도 데이터 저장소로 복사해야 전체 재현이 가능하다.

포함·미포함 파일의 출처·크기·SHA-256은 [manifest.json](manifest.json)을 참고한다.
JSON 안의 `/home/kim/...` 경로와 PID는 실험 당시 provenance이며 실행 인수가 아니다.
배포본 생성기는 [export_navigation_reports.py](../../../tools/export_navigation_reports.py)다.
생성기는 ROS 노드를 실행하거나 주행 명령을 발행하지 않는다.

## 링크된 원본 중 이번 묶음에 포함하지 않은 자료

- `validation/narrow_20261020/before` — directory or missing local capture
- `validation/self_returns_20261020/before_compact` — directory or missing local capture
