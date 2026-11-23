# Height Map 저장 위치

`mapping.launch.py save_dir:=.../src/rubi_height_mapping/maps`로 지정합니다.
`/rubi/global_elevation/save_map` 응답에 나온 `run_*/map.rghm`이 navigation 입력입니다.
같은 폴더의 `surface.pcd`는 표면 시각화용이며 localization의 전체 3D PCD와 다릅니다.
실측 지도는 Git에 넣지 않고 납품 데이터로 별도 복사합니다. 설치 폴더가 읽기 전용이면 쓰기 가능한 `save_dir`를 지정하세요.
