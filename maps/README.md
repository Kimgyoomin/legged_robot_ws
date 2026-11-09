# 현재 지도 세트

- **원본**: `raw/run_1794037249460292244/map.rghm` — 관측258,096셀.
  SHA256 `1ae7382f5ffcb09e6f418738dcfedaa7815a5198973929ab235987251f853cf8`.
- **최종**: `rubi_1f_height_20261109_wall_gap_v6/map.rghm` — 265,219셀, 해상도0.05m, frame map.
  SHA256 `c5c491b9dc00e50659a6ce812296f001d5ae4f6af8ab126f4cff2561828ee35c`.
- **기준 PCD/2D**: `rubi_1f_legacy/data/`. Localization과 static costmap에 필요합니다.

이번 최적화/정리에서 위 데이터는 수정하지 않았습니다. 각 `site.yaml`의 데이터 해시와 참조 경로를 확인했습니다. 최종 지도에는 기존 후처리의 보간·단차 수동 제약이 들어 있고 `provenance.npz`, `metrics.json`, 보고서를 함께 유지합니다. 측량이나 실차 통과 검증과 동일하지 않습니다.

중간 V1~V5·이전 세션·합성 자료는 [보관함](../archive/20261109/maps)으로 이동했습니다. 최종 site의 provenance/evidence 상대경로도 갱신했습니다. 추적 근거를 유지하려면 보관함의 관련 reports/evidence를 함께 전달합니다. `.pcd`, `.rghm` 등 실제 대용량 지도는 `.gitignore`에 의해 기본 제외되므로 Git clone만으로 실물 데이터가 전달되지 않습니다.

새 지도는 새 site를 만들고 launch의 `site:=...`로 선택합니다. 최종 지도의 경로를 바꾸면 [교체 도구](../tools/replace_site_height.py)를 사용하고, 원본을 덮어쓰지 않습니다.
