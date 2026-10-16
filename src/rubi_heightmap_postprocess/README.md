# RUBI 오프라인 height map 후처리 · 0.1.0

저장된 `.rghm` 지도에 기본 평균 필터와 평면 기반 보정을 적용하고 비교 보고서를
생성합니다. 작업용 PC의 NumPy/SciPy로 동작하며 CUDA, CuPy,
`elevation_mapping_cupy`, 실시간 mapper를 실행할 필요가 없습니다.
실시간 센서 융합·TF·전역 누적 알고리즘은 변경하지 않습니다.

현재 버전은 **검토용 처리본**을 생성합니다. 넓은 지도 왜곡을 자동으로 고치는 기능,
world와의 자동 정밀 정합, 이족 보행 가능 영역 판정은 포함하지 않습니다.

## 설치와 실행

Ubuntu 22.04 / ROS 2 Humble / Python 3.10에서 검증합니다. 필요한 Python 패키지는
NumPy, SciPy, Matplotlib, PyYAML입니다. 패키지 manifest에 rosdep 의존성이 있습니다.

```bash
cd ~/rubi_mapping_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select rubi_heightmap_postprocess
source install/setup.bash
```

이 대화에서 저장한 원본으로 비교를 재실행하는 예시입니다. 출력 디렉터리는 매번
새 경로를 지정합니다. 이미 존재하는 출력 경로와 원본 덮어쓰기는 거부합니다.

```bash
ros2 run rubi_heightmap_postprocess compare \
  --input /home/kim/rubi_mapping_ws/maps/frozen_20261014_122939/map.rghm \
  --output /home/kim/rubi_mapping_ws/maps/postprocess_comparison_new \
  --config /home/kim/rubi_mapping_ws/install/rubi_heightmap_postprocess/share/rubi_heightmap_postprocess/config/conservative.yaml \
  --qc-regions /home/kim/rubi_mapping_ws/install/rubi_heightmap_postprocess/share/rubi_heightmap_postprocess/config/frozen_20261014_qc.yaml
```

`--config` 생략 시 같은 보수적 기본값을 사용합니다. `--qc-regions`는 **이 동결본에
수동으로 정한 평가 영역**이며 다른 지도에는 생략하거나 새 좌표로 작성합니다.
ROI는 결과 측정에만 사용하고 필터 입력으로 사용하지 않습니다.

ROS 없이도 Python package 경로에서 다음과 같이 실행할 수 있습니다.

```bash
python3 -m rubi_heightmap_postprocess.cli --input map.rghm --output new_result
```

## 알고리즘

- **baseline**: 반경 6 cm의 유한값 원형 평균 + 원본 관측 마스크 복원.
  `grid_map_filters/MeanInRadiusFilter`의 관측 셀 계산을 독립 구현했습니다.
  upstream 필터가 미관측 중심을 채우는 동작은 관측 마스크를 복원해 제외합니다.
  전체 inpainting 체인을 실행한 결과는 아닙니다.
- **candidate**: 3×3 높이 범위로 경계 후보를 잡고 주변 1셀과 미관측 주변을
  보호합니다. 남은 연결 영역에 robust 가중 최소제곱 평면을 맞춥니다.
  전체 영역의 잔차 P95 ≤4 mm, 모든 변경량 ≤10 mm 등의 조건을 통과하면
  영역 평면을 적용합니다. 실패한 영역은 같은 연결 영역 내부 7×7 이웃에서만
  작은 평면을 맞추고 RMS ≤3 mm, 변경량 ≤10 mm 조건을 통과한 셀만 수정합니다.
  큰 휨은 전체 평면으로 강제 변환하지 않습니다.
- **보존**: XY, 원점, 해상도, 관측 마스크, 보호 셀 높이, 원본 시각을 유지합니다.
  미관측 영역 채움과 큰 이상점 삭제는 하지 않습니다. 보호된 곳에는 잡음도 남습니다.

이 candidate는 독립적인 첫 구현입니다. ETH `convex_plane_decomposition`을
그대로 실행한 결과가 아니며 RANSAC/발 디딤 다각형 추출을 포함하지 않습니다.
기본 경계 임계값은 25 mm입니다. 그보다 작은 구조까지 보존한다고 보장하지 않으므로
RUBI에 필요한 최소 지형 크기/단차에 맞춰 추가 검증해야 합니다.

참고: [grid_map mean filter](https://github.com/ANYbotics/grid_map),
[평면 분할 파이프라인](https://github.com/leggedrobotics/elevation_mapping_cupy/tree/main/plane_segmentation).

## 결과

| 파일 | 내용 |
|---|---|
| `report.html` | 같은 좌표·색상 범위의 지도, 바닥 확대, 단차 단면, 수치 |
| `baseline/map.rghm`, `candidate/map.rghm` | 원본과 별도인 처리본 |
| 각 디렉터리의 `surface.pcd` | binary XYZ, 원본 map 좌표계 |
| `layers.npz` | raw/baseline/candidate, XY, 경계·보호 마스크, 영역/평면 ID, 보정 방식 |
| `metrics.json` | 완료 manifest, 입력·출력·구현 SHA-256, 파라미터, 환경, 측정값 |
| `qc_regions.json` | 평가에 사용한 ROI |

`method` 레이어는 0=보정 안 함, 1=영역 평면, 2=국소 평면입니다.
`region_id`는 연결 영역으로, 모두 평면이라는 뜻이 아닙니다.
`plane_id`는 영역 전체 평면 검사를 통과한 곳만 양수입니다.

`.rghm`의 분산과 셀 시각은 원본을 보존한 **출처 정보**입니다. 수정한 높이의
사후 오차 분산이 아니며 후처리로 새 관측이 생긴 것도 아닙니다. 처리본은
`metrics.json` 및 마스크와 함께 보관합니다. 원본 지도는 별도 보존합니다.

## RViz 비교

```bash
ros2 launch rubi_heightmap_postprocess preview.launch.py \
  result_dir:=/home/kim/rubi_mapping_ws/maps/postprocess_comparison_new \
  rviz:=true
```

Displays에서 `raw`, `baseline`, `candidate`를 **한 번에 하나씩** 켜서 비교합니다.
기본값은 candidate입니다. 좌표계는 이 프로젝트의 `map`, 높이 색상 범위는 세
display에 동일하게 고정했습니다. 다른 좌표계/높이 범위를 가진 지도는 RViz의
Fixed Frame과 세 display의 Value Bounds를 같이 바꿉니다.
토픽은 `/rubi/postprocess_preview/{raw,baseline,candidate}`이며 실시간 mapping
토픽에 발행하지 않습니다. 원본 지도 시각으로 재발행하며 sim time이 필요 없습니다.
같은 namespace의 preview를 중복 실행하지 마세요.

## 검증

```bash
colcon test --packages-select rubi_heightmap_postprocess --event-handlers console_direct+
colcon test-result --test-result-base build/rubi_heightmap_postprocess
```

합성 5/15 cm 단차·경사면, 미관측 구멍, 큰 휨, 보호 영역, 변경량 제한,
비정방 격자의 XY, 큰 정수 nanosecond 시각 보존, 잘못된 입력 거부를 검사합니다.
국소 중앙값 잔차는 매끈함 지표이며 정답 지형에 대한 정확도 지표가 아닙니다.
