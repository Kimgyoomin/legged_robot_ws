# RUBI 1층 운용 지도

실물과 rosbag의 hybrid 및 height-only navigation은 `rubi_1f/map.rghm`을 읽습니다. 5cm 격자, `map` frame, 265,219셀입니다. 홀 입구 6cm 판의 기준점은 **(9.075, 40.925)m**이고 윗면 외접 크기는 X 25cm × Y 45cm입니다. 복도 (8.43, 20.05)m 주변은 보정된 바닥을 사용합니다.

| 용도 | 현장 폴더 안의 경로 |
|---|---|
| Height Map | `map.rghm` |
| FAST-LIO localization PCD | `data/1f_fast_lio_260731_segmented.pcd` |
| 2D Occupancy | `data/RUBI_1F_occupancy_map.yaml` 및 같은 폴더의 PGM |
| 파일 해시와 좌표계 | [site.yaml](rubi_1f/site.yaml) |

장비에 배포할 때는 **`maps/rubi_1f/` 폴더 전체**를 같은 위치에 복사합니다. 실측 지도는 Git 소스와 별도 전달합니다. 장소 폴더는 [운용 YAML](../src/navigation_dds/config/operation_rubi.yaml)의 `site`에서, 파일명은 `common.navigation.height_map`, `common.localization.pcd_map`, `common.hybrid.map_yaml`에서 변경합니다. `${SITE}`는 YAML의 `site`를 가리킵니다.

이전 원본·후처리 지도와 10cm 대조군은 [실험 보관함](../archive/20261124/maps)에 있습니다. 원래 goal에서 11월 2일 bag의 global planner 중심선은 확정 지도의 6cm 윗면을 CoT 7셀, vanilla 6셀 지났습니다. 같은 자리에 10cm를 놓은 대조에서는 CoT 0셀, vanilla 7셀이었습니다.
