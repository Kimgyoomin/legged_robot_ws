# Localization 지도 저장 위치

FAST-LIO SLAM의 `pcd_map` 인자로 이 폴더의 새 파일 경로를 지정합니다.
예: `~/legged_robot_ws/src/FAST_LIO_LOCALIZATION2/maps/site_a.pcd`.
`/map_save`의 success와 파일을 확인한 뒤 SLAM을 종료하고 이 PCD로 localization을 실행합니다.
기존 YAML/PGM은 2D 지도이며 새 PCD 좌표계와 일치할 때만 재사용합니다.
실측 PCD는 Git에서 제외되므로 납품 시 별도 복사해야 합니다.
