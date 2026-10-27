# 07. 브랜치와 다음 작업 인계

[문서 처음으로](README.md)

## 저장 범위

브랜치 이름은 `codex/navigation-handoff-20261027`이다. 기존 main의 백업을 바탕으로 현재 navigation 관련 변경을 묶었다. main과 기존 `jetson/real` 브랜치는 직접 수정하지 않는다.

| 포함 | 내용 |
|---|---|
| `src/legged_range_trial` | 분리된 실험 launch/config, DWB/RPP 선택, 두 velocity 어댑터, freshness watchdog, tests |
| `src/legged_navigation` 변경 | C++ 비대칭 거리·BODY 높이 필터, 테스트 |
| `src/legged_bringup` 변경 | 공통 geometry/guard, simulation localization 연결, retry BT, 관련 tests |
| `src/FAST_LIO_LOCALIZATION2` 변경 | native twist 발행, bounded processing, 직접 simulation launch와 tests |
| `maps/cost_wall_lower10_v3`, `maps/cost_wall_main_v2` | 실행 가능한 site 자산과 provenance/QC |
| `docs/navigation` | 현재 인계 문서, 설정 snapshot, 선별한 과거 보고서·이미지·측정 요약 |

이 문서화 작업은 새 navigation 계수를 튜닝하는 작업이 아니다. 현재 실험 코드를 기록하고, 오래된 설명 중 현재 설정과 충돌하는 부분을 정리한다. 동작 중인 simulator·localization·navigation에 새 목표나 속도 명령을 보내지 않는다.

## 외부 의존성

PC 시뮬레이터/보행 정책/LCM bridge는 `/home/kim/rubi_ws`, world는 `/home/kim/cot_experiment/navigation_cost_comparison/...`를 사용한다. 이 경로는 현재 장비의 경로이며, 별도 PC에 자동 생성되지 않는다. `legged_robot_ws`는 localization·height mapping·global/local navigation의 중심 workspace다. 아직 simulator까지 완전히 포함한 단일 납품 bundle은 아니다.

RGB-D 장착/손잡이 모델 변경, 움직이는 사람 실험은 이번 navigation 인계와 분리한다. 사용하지 않는 pedestrian 기능을 실행 명령에 추가하지 않는다. 원본 bag·점군·대형 중간 dump는 로컬 `validation/`에 남기고, 보고서 배포본의 포함/미포함 파일과 SHA-256은 [manifest](reports/manifest.json)에 기록한다.

지도의 원본 raw 저장 경로는 provenance다. 실행에 필요한 height 파일은 site 안에 있으므로 raw 디렉터리 전체를 복사하지 않아도 site를 로드할 수 있다. 큰 데이터까지 완전 재분석하려면 [로컬 원본 안내](reports/LOCAL_DATA.md)를 참고한다.

## 알려진 simulator 문제

장시간 Gazebo 중단 중 하나는 외부 Livox simulation plugin의 lifetime scan index overflow와 관련이 있었다. plugin을 재빌드해도 기존 Gazebo 프로세스는 이전 `.so`를 계속 사용한다. [crash 분석](reports/sim_crash_20261027/report.html)

이 외부 plugin 수정은 이번 navigation 소스에 포함되어 있지 않다. 새 PC에 simulator를 옮길 때 `/home/kim/rubi_ws/src/livox_laser_simulation_ros2`의 수정 여부를 별도로 확인해야 한다. FAST-LIO 속도 adapter 추가와 Gazebo crash가 시간상 가까웠다는 이유만으로 인과관계를 단정하지 않는다.

## 다음 개발자에게 전달할 핵심

1. 먼저 [실행 절차](05-operation.md)와 [현재 기준값](README.md)을 읽는다. 옛 HTML은 실험 당시 값이다.
2. FAST-LIO native velocity를 유지하고 RPP/DWB를 같은 world·site·goal로 비교한다. MPPI는 현재 선택하지 않는다.
3. RPP가 따라갈 global path가 실제 통과 가능한지 확인한다. 현재 HeightPlanner는 grid node path를 직접 반환하고 oriented footprint/smoothing을 수행하지 않는다.
4. 급회전 실패를 inflation/footprint 축소만으로 덮지 않는다. command→실제 vx/wz 지연과 필요한 회전반경을 기록한다.
5. body-height 필터는 지면-relative 단차 분류기가 아니다. “15cm 통과” 요구와 현재 planner hard limit 10cm의 차이를 먼저 합의한다.
6. trial default `final_guard=false`여도 Collision Monitor 사각 Stop은 남는다. CM 정지 뒤의 자동 후진/회전 recovery는 아직 없다.
7. Jetson Orin NX에서 latency/CPU/메모리/전력을 측정한다. 알고리즘이 가볍다는 설명과 실제 deadline 충족은 다른 결과다.

## 검증 상태

문서·코드 링크, site 자산 해시, 현재 YAML과 문서 snapshot의 일치, 관련 패키지 빌드·기존 단위 테스트를 확인해 이 브랜치를 묶는다. 결과는 [publication_check.json](publication_check.json)에 남긴다. 이번 문서화 과정에서 새 주행 A/B 실험이나 Jetson 시험을 실행한 것으로 해석하지 않는다.
