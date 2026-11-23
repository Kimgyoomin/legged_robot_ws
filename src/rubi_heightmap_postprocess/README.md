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
영역 편집 GUI에는 Tkinter(`python3-tk`)와 데스크톱 DISPLAY가 추가로 필요합니다.

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

## 마우스로 발 잔상 선택 · 작은 구멍 복원 GUI

`edit`는 위의 보수적 `compare`와 **별도 기능**입니다. 저장된 `.rghm` 원본을
읽어 작업 복사본만 수정하며, 실시간 ROS 토픽·TF·mapper에는 접근하지 않습니다.
사람을 자동 인식하는 도구가 아니므로 **실제로 바닥임을 아는 영역만** 선택하세요.
실제 턱·돌·계단도 작은 영역으로 선택하면 지워질 수 있습니다.

현재 Jetson에서 저장한 원본을 여는 예시:

```bash
cd ~/legged_robot_ws
source tools/height_env.sh
ros2 run rubi_heightmap_postprocess edit \
  --input "$PWD/maps/raw/run_1792642525523792868/map.rghm" \
  --output-root "$PWD/maps/bag_260731/repairs"
```

`--input`을 생략하면 파일 선택 창이 열립니다. 이 편집기는 CuPy/GPU를 쓰지 않습니다.
다른 PC에서는 일반 ROS 환경 + NumPy/SciPy/Matplotlib/Tkinter면 충분합니다.
새 스크립트 설치가 필요한 경우 위의 패키지 단독 빌드를 사용하세요.

### 발 잔상 지우기

v2에서는 **수정할 영역과 바닥 지지 영역을 분리**합니다. 주변에도 발 잔상이 있으면
자동 링의 최대 오차 검사에 계속 걸릴 수 있습니다. 이때 한도를 올리는 대신 깨끗한
바닥만 직접 고르세요. `Artifact repair` 탭에서 다음 순서로 진행합니다.

1. 왼쪽은 **수정하지 않는 원본**, 오른쪽은 작업본입니다. 휠로 확대하면 두 화면이
   같은 좌표로 확대됩니다. `View` 모드에서 아래 toolbar의 Pan으로 이동합니다.
2. `A. Artifact to erase`에서 `Rectangle`로 드래그하거나 `Polygon`으로 꼭짓점을
   클릭한 뒤 첫 점을 클릭해 닫습니다. 발 잔상 **전체를 포함하는 작은 영역**을 고릅니다.
   분홍 테두리가 실제로 수정될 영역입니다. A를 새로 그리면 이전 바닥 선택도 초기화합니다.
3. `B. Add clean floor patches`로 전환한 뒤, 발 잔상의 **양옆 깨끗한 바닥을 각각**
   드래그합니다. 선택을 여러 번 하면 초록 지지 영역이 추가됩니다. 바닥 패치는 발 영역의
   양 끝까지 충분히 넓게 잡으세요. 한쪽 바닥으로만 반대편을 추정하지 않도록 제한합니다.
   이미 보정한 셀·미관측·A와 겹친 부분은 지지점에서 제외됩니다.
4. `Preview selected patch repair`를 누릅니다. B만 사용해 경사까지 포함한 평면을
   맞추고, **A만** 그 높이로 대체하는 미리보기를 냅니다. B 자체는 수정하지 않습니다.
   B를 지정하지 않았거나 `Clear floor only`를 눌렀으면 기존 자동 링 방식을 사용합니다.
   적합에 실패하면 팝업 대신 설명과 **충돌하는 지지점의 빨간 X**가 나옵니다.
5. `Show edit masks`를 끄면 색상 마스크 없이 복원 표면을 비교할 수 있습니다.
   `Show height difference`는 오른쪽에 원본 대비 높이 변화(±10 cm 표시 범위)를
   보여 줍니다. 원래 미관측이던 셀은 높이 차이를 정의할 수 없으므로 마스크로만 표시합니다.
6. 바닥이 맞는지 확인한 뒤 `Apply preview` → 확인을 누릅니다. `Undo`/`Redo`
   (`Ctrl+Z`/`Ctrl+Y`)로 되돌릴 수 있습니다. `Esc`는 선택·미적용 미리보기만 취소합니다.

공통 안전 조건: 영역 최대 폭 2 m, **원본·미편집 지지점** 20개 이상, 지지점 전체 RMS ≤5 mm,
최대 잔차 ≤15 mm, 기울기 ≤15°, 관측 셀 변경량 ≤30 cm입니다. 강건 가중 최소제곱을
사용하지만, **검사는 제외점 없이 모든 지지점**으로 수행해 단차를 통과시키지 않습니다.
자동 모드: 지지 링 30 cm, 링의 절반 이상 관측, 사방 각각 지지점 3개 이상을 요구합니다.
수동 모드: 지지점은 A에서 1 m 이내이며, 지지점의 XY 볼록껍질 안에 A 전체가 들어가야
합니다. 따라서 한쪽 패치만 고르거나 너무 얇게 선택하면 거부합니다. 주변의 다른 잔상을
수치적으로 묵살하지 않고 **사용자가 선택하지 않은 영역을 지지점에서 제외**하는 방식입니다.
이는 보수적인 수치 검사이지 실제 바닥이라는 보장은 아니며, 서로 다른 높이의 바닥이나
실제 턱을 가로질러 선택하지 마세요. 이미 보정/보간한 셀을 다시 지지점으로 쓰지 않습니다.

`Holes / limits` 탭에서 링 폭·RMS·변경량 한도를 조정할 수 있지만, 실패했다고 한도를 먼저 키우지
마세요. 주변에 다른 발 잔상이나 턱이 섞이지 않았는지, 선택 영역이 적절한지 확인하세요.
큰 잔상을 여러 번 부분 선택하면 남은 잔상이 지지 링에 들어갈 수 있습니다.

### 작은 구멍 메우기

`Holes / limits` 탭의 `Find small holes (selection or all)`은 A 선택 영역이 있으면 그 안, 없으면 전체 지도에서
후보를 찾습니다. 외부 미관측과 8방향으로 연결되지 않은 **내부 구멍만**, 최대 4셀,
가로·세로 최대 10 cm 조건으로 제한하며 자동 링의 바닥 지지 검사도 통과해야 합니다.
작은 구멍 일괄 찾기에는 수동 B 패치를 사용하지 않습니다.
5 cm 격자에서는 최대 2×2셀입니다. 큰 빈 공간·벽 반대편·계단 틈을 자동으로 메우지 않습니다.

`Next hole` / `Previous hole`로 후보 위치를 확대 확인하세요.
`Keep only this hole in preview`를 누르면 현재 구멍 하나만 남습니다.
**그 버튼 없이 Apply를 누르면 현재 미리보기의 모든 후보가 적용**됩니다.
한 개씩 검토하려면 하나 적용 후 다시 후보 찾기를 실행하세요.
기존 미관측을 채운 결과는 측정값이 아닌 **추정 높이**입니다.

### 저장과 재개 — 원본은 덮어쓰지 않음

`Save review copy` (`Ctrl+S`)는 출력 루트 아래에 매번 새로운
`repair_YYYYMMDD_HHMMSS_microseconds/`를 만듭니다. 미적용 미리보기가 있으면 먼저
적용하거나 취소해야 저장할 수 있습니다. 분홍은 미리보기, 주황은 관측 셀 수정,
청록은 미관측을 채운 추정 셀입니다.

| 파일 | 용도 |
|---|---|
| `original/map.rghm` | 입력 원본의 바이트 단위 사본 |
| `repaired_preview.pcd` | 발 잔상 보정 + 구멍 채움을 모두 포함한 XYZ 표면 |
| `repair_layers.npz` | `repaired` 높이, `method`, `originally_observed`, `inferred` 마스크 |
| `observed_only/map.rghm` | 원래 관측한 셀의 보정만 반영; **새로 채운 구멍은 포함하지 않음** |
| `repair.json` | 좌표 정보, 입력/출력 해시, 적용/되돌리기 이력, 평면 적합 수치, 승인 상태 |

`method`는 0=원본 그대로, 1=직접 선택한 영역 복원, 2=작은 구멍 복원입니다.
`inferred`는 방법과 무관하게 원래 미관측이었던 셀을 표시합니다.

**현재 `.rghm` 형식에는 보간 셀의 출처/신뢰도를 구분하는 필드가 없습니다.** 따라서
채운 셀에 가짜 측정 시각·분산을 넣어 관측한 것으로 저장하지 않습니다. 구멍 채움까지
포함한 최종 표면은 PCD/NPZ에 있으며, 이를 실제 frozen height map 소비 경로에 연결하는
작업은 별도 검토가 필요합니다. `observed_only`의 분산과 시각도 원본 출처 정보이지
수정한 높이의 측정 정확도가 아닙니다. 저장했다고 Nav2 사용이 승인되는 것은 아니며
manifest의 `navigation_approved`는 항상 `false`입니다.

저장본을 이어서 편집하려면:

```bash
ros2 run rubi_heightmap_postprocess edit \
  --session /absolute/path/to/repair_YYYYMMDD_HHMMSS_microseconds \
  --output-root "$PWD/maps/bag_260731/repairs"
```

다시 열 때 원본·표면·레이어 해시를 확인합니다. 이전 적용 결과와 감사 이력은
유지되지만 Undo/Redo는 **다시 연 이후 작업부터** 가능합니다. 앱을 종료하기 전 저장하세요.
기본 출력 루트는 입력 파일 옆 `repairs/`이며, `--output-root`로 바꿀 수 있습니다.

### GUI 검증

기본 `colcon test`에는 복원 계산·단차 거부·구멍 크기 제한·좌표·되돌리기·원본 보존·
저장/재개 검사가 포함됩니다. 데스크톱이 없는 CI에서는 GUI smoke test만 건너뜁니다.
DISPLAY가 있는 Jetson에서 실제 Tk/Matplotlib 콜백을 검사하려면:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 RUBI_GUI_TEST=1 \
  python3 -m pytest -q src/rubi_heightmap_postprocess/test
```

이 테스트는 합성 지도와 임시 출력만 사용하며 실지도나 실행 중인 ROS 노드를 바꾸지 않습니다.
