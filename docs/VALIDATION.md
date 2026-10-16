# 검증 결과

## 2026-10-16 목표 수신 후 경로 생성 실패: 강제 외접원 철회

14:44 실행의 RViz 로그에서 목표 `(7.20272, -9.33327)` 수신을 확인했다.
Planner는 수천 개 셀을 탐색한 뒤 `frontier_exhausted`로 종료했다. 목표 도구의
문제가 아니라 경로 탐색 실패였다. 바로 앞 변경에서 launch가 정지 사각형의
외접 반경 0.672681 m를 강제 적용한 것이 원인이었다.

Live domain 45에서 원본 costmap·동결 height cloud·현재 TF만 읽어 저장했다.
같은 데이터와 목표를 domain 179의 **planner만** 실행하여 비교했다.

| 추가 planner clearance | 결과 | 추종 경로 점 수 |
|---|---|---:|
| 0 m (기존 costmap inflation 유지) | 성공 | 585 |
| 0.45 m | 성공 | 585 |
| 0.672681 m | `frontier_exhausted` | 0 |

성공한 경로의 선분 접선 방향마다 1.0 × 0.9 m 사각형을 원본 costmap과 검사했을
때 충돌 표본은 0개였다. 이는 이 snapshot의 이산 자세 검사이며 실제 보행,
모든 선분 사이 회전, 제동·추종 오차까지 검증한 결과는 아니다.

수정: 강제 외접원 덮어쓰기를 제거하고 두 planner YAML에
`costmap_clearance_m: 0.45`를 명시했다. 정지 사각형, 진행 방향의 최종 충돌 검사,
센서 시간 제한은 유지했다. Frozen height만으로 바꾸거나 occupancy 장애물을
무시하는 변경이 아니다. 아래 0.672681 m 적용은 시험 이력이며 현재 설정이 아니다.

Bringup 재빌드 및 Python 29개 검사를 통과했다. 격리 domain 174/173의
Pure Pursuit/Nav2 DWB 통합 시험에서 YAML의 0.45 m가 실제 planner에 전달되고,
경로 생성·새 장애물 우회·근접 정지·오래된 센서 정지가 동작함을 확인했다.
사용자의 navigation/Gazebo/localization은 재시작하지 않았고, 사용자 domain으로
목표나 속도 명령을 보내지 않았다. 적용은 **navigation만 동일 명령으로 재실행**한다.
실제 보행 성공이나 모든 방향에서의 footprint 통과는 아직 검증하지 않았다.

근거: [격리 재생 결과](../validation/planner_clearance_20261016/replay.json),
[입력 snapshot](../validation/planner_clearance_20261016/capture.json).
그림: [탐색 여유 비교](../validation/planner_clearance_20261016/clearance_comparison.png).
재생 도구: `tools/replay_planner_clearance.py` (domain 179 전용, controller 없음).
백업: `backups/planner_clearance_20261016_144905/`.

## 2026-10-16 Global costmap 정지와 경로 여유 불일치

사용자가 수동 조작 없이 자동 주행 중이라고 확인한 상태에서 약 35초 읽기 전용
기록을 수집했다. 이 구간의 `/legged/cmd_raw` 320개는 모두 전진 0이었고,
319개는 회전 명령이었다. `/cmd_vel` 699개는 모두 전진·회전 0이었다.
Motion status 66개 중 64개가 `costmap_collision`, 2개가 `safe_command_timeout`.
즉 **이번 기록의 직접적인 차단 원인은 원본 global costmap을 검사하는 watchdog**이다.
제자리 정렬 자체만으로 최종 정지를 설명하면 부정확하다.

당시 경로 탐색은 costmap의 robot_radius 0.30 m + padding 0.01 m에 의존했지만,
최종 정지 검사는 1.0 × 0.9 m 직사각형이었다. 이 둘의 여유가 일치하지 않아
planner가 유효하다고 보는 경로에서 watchdog가 막을 수 있었다. 기록 후 별도
geometry snapshot은 로봇/목표가 바뀐 시점이어서 앞선 stuck 위치의 개별 충돌
셀을 특정하는 근거로 사용하지 않았다. 해당 snapshot의 점군 변환 분석은
NumPy structured-array 변환 오류로 완료되지 않았다.

변경:

- launch에서 Stop 사각형의 외접 반경 0.672681 m를 planner clearance로 전달.
  Custom planner와 Nav2 Height plugin의 **계획용 복사본**에서 실제 obstacle/unknown
  (254/255), 지도 경계 주변을 금지(253) 처리. 셀 면적 여유 포함. 최종 watchdog와
  실제 costmap은 원본을 계속 사용하므로 확장된 장애물을 다시 검사하지 않는다.
- 기존 inflation 1~253을 seed로 다시 확장하지 않는다. Custom planner에서는
  원본 지도 내용이 바뀔 때만 clearance 복사본을 갱신한다.
- 이는 2D planner를 위한 회전 방향 무관의 보수적인 원형 여유이다. 약 1.35 m
  이하 통로는 격자 여유를 포함해 거절될 수 있다. 이미 겹친 시작점에서 자동으로
  빠져나오는 동작이나 yaw 상태를 고려한 좁은 통로 계획은 제공하지 않는다.
- Pure Pursuit lookahead 0.45→0.65 m, 정렬 진입 0.7→1.2 rad,
  정렬 해제 0.25→0.5 rad. 최대 속도 0.7 m/s / 추종 yaw 1.0 rad/s 유지.
- 정상 soft replan에서 속도 limiter와 정렬 상태를 이어받음. empty/invalid path와
  hard replan 후에는 기존처럼 zero부터 시작. 경로 이탈·센서·충돌 정지 유지.

검증:

- planner, legged_navigation, legged_bringup 빌드 성공.
- Planner 단위·설정 검사 27개 실행 항목, PP core 14개 case,
  bringup Python 29개 case 통과.
- 새 clearance 단위 검사: 원본 불변, inflation 중복 확장 없음, unknown/경계 차단,
  너무 좁은 지도 거절, boundary seed 최적화가 brute-force 원형 확장과 일치.
- Domain 178: 6회의 soft replan 동안 전진 명령 최저 0.7 m/s 유지;
  52° 방향 오차에서 전진 추종, 뒤쪽 목표에서 제자리 회전, 26°에서 정렬 해제,
  hard replan은 zero부터 재시작. 고정 합성 pose이며 실제 주행 성능 측정은 아니다.
- Domain 174/173: custom pursuit 및 Nav2 DWB에서 경로 생성, 장애물 우회,
  근접 정지·센서 중단 정지, 동결 지도 stamp 유지. 실제 planner 파라미터에
  0.672681 m 여유가 전달된 것을 확인.
- Domain 176: 지도 변화 없는 이탈 재계획, 목표 보존, blocked start 정지,
  공간이 열린 뒤 재계획, 오래된 pose 거부 회귀 검사 통과.
- 사용자 domain 45의 Gazebo/localization/navigation은 재시작하지 않았고,
  목표나 속도 명령을 보내지 않았다. 새 구성으로 실제 stuck 감소는 아직 미검증.

백업: `backups/stop_turn_20261016_143315/`.
근거: [live 기록 요약](../validation/stop_turn_20261016/summary.json),
[추종 연속성](../validation/stop_turn_20261016/pursuit_continuity.json),
[Pure Pursuit](../validation/stop_turn_20261016/pursuit_smoke.json),
[Nav2](../validation/stop_turn_20261016/nav2_smoke.json).
이번 수정 적용에는 **navigation만 같은 명령으로 재실행**하면 된다.

## 2026-10-15 1.0 × 0.9 m 정지 영역과 속도 상한 조정

- Pure Pursuit nominal/max 0.7 m/s, 추종 yaw 상한 1.0 rad/s,
  제자리 방향 정렬 상한 0.5 rad/s. 가속도·곡률 감속은 유지.
- Stop 영역은 base_link 기준 앞/뒤 0.50 m, 좌/우 0.45 m.
  `collision.yaml` 좌표에서 지도 검사 반폭도 읽어 같은 직사각형 사용.
  Costmap robot_radius 0.30 m + padding 0.01 m와는 별도의 정지 여유.
- Gazebo terrain 명령 입력 X 상한 0.7 m/s, 모든 축 명령 배율 1배.
  실행 중 라이브러리 inode를 백업 위치에 보존한 후 새 파일로 빌드.
- Bringup 및 simulator plugin 빌드 성공. Guard 24개 + delivery 5개,
  simulator command 선택·배율 11개 검사 통과.
- 격리 domain 177: 최종 0.7 m/s 명령 통과; 폭 밖 측벽 통과/폭 안 측벽 정지;
  지도에 알려진 거리 필터 사각지대 장애물 정지; unknown·오래된 pose/cloud·
  잘못된 costmap 정지; inflation 중복 적용 없음.
- 격리 domain 174/173: Pure Pursuit/Nav2 DWB 각각 경로 생성, 새 장애물 우회
  경로 생성, 근접 장애물 정지, 오래된 센서 정지 통과. 실제 노드의 파라미터를
  조회하여 두 검사의 길이 1.0 m / 폭 0.9 m 일치 확인. 동결 지도 timestamp 유지.
- Pure Pursuit 합성 경로 테스트의 최고 최종 전진 명령은 약 0.615 m/s.
  이는 설정된 상한 0.7 m/s와 다르며 곡률·가속도 제한을 거친 값이다.
  테스트의 pose는 합성 고정값이므로 실제 보행 속도 측정이 아니다.
- 첫 통합 시험 종료 시 테스트가 launch와 모든 자식에 SIGINT를 중복 전달하여
  Python 종료 예외가 나타났다. 테스트 harness를 launch에 한 번 전달하도록
  수정하고 Pure Pursuit를 재검증했으며 모든 노드가 정상 종료했다.
- 사용자 domain 45의 Gazebo와 navigation은 재시작하거나 목표를 전송하지 않았다.
  새 설정에서의 실제 보행·정지거리·좁은 통로 주행 성공은 아직 미검증이다.

백업: `backups/rectangle_speed_20261015_222553/`.
근거: [guard](../validation/rectangle_speed_20261015/watchdog.json),
[Pure Pursuit](../validation/rectangle_speed_20261015/pursuit_smoke.json),
[Nav2 DWB](../validation/rectangle_speed_20261015/nav2_smoke.json).
운영 방법과 장애물 처리 흐름은 [OPERATIONS.md](OPERATIONS.md)의 최신 설정 절 참고.

## 2026-10-15 0.6 m/s 설정과 몸체 크기 정지 영역

요청에 따라 Pure Pursuit의 nominal/max를 0.6 m/s, 곡률 감속 계수를 0.5,
lookahead를 0.45 m로 조정했습니다. 가속도·각속도 제한과 heading alignment는
유지했습니다. Gazebo terrain adapter의 X 입력 상한도 0.6 m/s로 올렸고
명령 배율은 1배입니다. Stop polygon은 반경 0.3 m + padding 0.01 m에서 만든
0.62 × 0.62 m 사각형입니다.

0.6 m LiDAR 거리 필터로 정지 영역 안의 점이 없어질 수 있으므로 최종 watchdog에
기존 global costmap을 이용한 현재/예상 몸체 영역 검사를 추가했습니다. 현재
속도 명령이 1초 유지된다고 가정한 궤적에서 회전하는 사각형과 grid cell의 겹침을
검사합니다. Cost 254/255와 지도 밖은 정지, inflation cost 1–253은 이 검사에서
반경을 중복 적용하지 않습니다. Map TF가 없거나 오래되어도 정지합니다.

검증:

- `legged_bringup`, `rubi_gazebo_plugins` 빌드 성공.
- Guard 단위 검사 10개, delivery 검사 5개, 기존 Pure Pursuit 14개,
  Gazebo 명령 선택·배율 검사 11개 통과.
- 격리 domain 177: 0.6 m/s 통과, 근처 측벽 통과, 거리 필터 사각지대 안에
  **지도상으로 알려진** 장애물 정지, unknown 정지, 오래된 TF/cloud 정지,
  잘못된 costmap 정지, 정상 입력 복귀 확인.
- 격리 domain 174/173: Pure Pursuit/Nav2 DWB 각각 경로 생성, 장애물 추가 후
  재계획, 근접 정지, 센서 중단 정지 통과.
- 격리 domain 175: 고정 장착 TF 요구, 뒤집힌 센서 변환, 거리 필터 유지,
  뒤쪽 목표의 제자리 정렬, 정렬 후 추종 복귀, 빈 경로 정지 통과.
- 현재 desktop에서 100만 셀 costmap을 캐시하고 0.6 m/s·0.8 rad/s 명령의
  1초 진행 영역을 검사한 계산 시간 중앙값 약 0.196 ms, P95 약 0.200 ms.
  이는 함수 구간만이며 ROS 처리 지연·실제 Jetson 성능이 아닙니다.

원본 백업: `backups/speed_footprint_20261015_220132/`.
사용자 domain의 Gazebo/localization/navigation은 재시작하지 않았습니다.
변경 후 실제 주행 속도, 추종 오차, 좁은 통로 통과 성공은 아직 미측정입니다.
일정 Twist 예측은 보행 제동 모델이 아니며, 지도에도 없는 새 장애물이 LiDAR
사각지대 안에 나타나는 상황을 보장하지 않습니다.

근거: [guard 통합 결과](../validation/speed_footprint_20261015/watchdog.json),
[Pure Pursuit](../validation/speed_footprint_20261015/pursuit_smoke.json),
[Nav2](../validation/speed_footprint_20261015/nav2_smoke.json),
[계산 시간](../validation/speed_footprint_20261015/guard_timing.json).

## 2026-10-15 시뮬레이터 navigation 명령 배율 수정

실행 중인 Gazebo가 `/home/kim/rubi_ws`의 terrain 정책 plugin을 사용함을 확인했습니다.
기존 변환은 입력 범위 안에서 X를 2배, Y/yaw를 1.5배 증폭했습니다. 수정본은
입력 제한(X/Y ±0.5 m/s, yaw ±1.0 rad/s)만 적용하여 전 축 1배로 전달합니다.
`(0.25, -0.25, 0.5)` 입력은 이제 같은 값으로 전달됩니다.

- `rubi_gazebo_plugins` 빌드 성공; gtest 개별 검사 **11개 통과**.
- 양/음/0 명령 및 상한의 단위 보존, 초과 명령 포화, 입력 선택·전환·시간 초과·
  비정상 입력·안전 정지 회귀검사 통과.
- 수정 전 소스와 실행 중 라이브러리 백업:
  `/home/kim/rubi_ws/backups/terrain_cmd_unity_20261015_212652/`.
- Pure Pursuit 추종기와 lookahead 변경 실험은 보류하고 수정 전 소스/설정으로
  복구했습니다. 이번 적용 대상은 시뮬레이터 navigation 명령 변환뿐입니다.
- 실행 중 Gazebo/localization/navigation은 재시작하지 않았습니다. 현재 Gazebo는
  기존 라이브러리를 계속 사용하므로, 재시작 전에는 새 배율로 주행하지 않습니다.

기록 시 `/clock`, odometry, TF가 갱신되지 않아 자동 주행의 실제 속도·지연을
측정하지 못했습니다. 이 결과는 명령 변환 검증이며, 좌우 흔들림 감소나 좁은 통로
통과 성공을 입증한 결과는 아닙니다. Gazebo 재시작 후 수동 조작 없이 자동 추종을
재현하여 확인해야 합니다. 학습 정책 내부의 observation 정규화는 유지했습니다.

근거: [수정 패치](patches/terrain_cmd_unity.diff),
[검사 출력](../validation/terrain_cmd_unity_20261015/tests.txt).

검증일: 2026-10-14. Ubuntu 22.04 / ROS 2 Humble / x86_64 PC에서 수행했습니다. 기존 실행 중인 시뮬레이션 domain과 분리했으며, 로봇 구동기를 실행하지 않았습니다.

## 결과

| 항목 | 결과 | 검증 범위 |
|---|---|---|
| SDK + ROS workspace 빌드 | 10개 package 성공 | 새 workspace의 SDK를 사용하여 빌드 |
| ROS package 자동 검사 | `250 tests, 0 errors, 0 failures, 0 skipped` | `colcon test-result` 집계; 개별 테스트 수와 suite 집계가 함께 포함될 수 있음 |
| 별도 GPU·ROS pose 검사 | 4 passed | 실제 CuPy kernel, 이동 grid, odometry 보간·장착·시간 reset |
| FAST-LIO / GenZ | 두 launch 시작 성공 | node import·설정·TF 구성 시작; localization 정확도 시험은 아님 |
| Pure Pursuit 통합 | 통과 | 경로 생성, 속도 출력, 장애물 재계획, 근접 정지, 센서 중단 정지 |
| Nav2 통합 | 통과 | lifecycle 활성화, NavigateToPose 수락·속도 출력, 경로 우회, 근접·센서 중단 정지 |
| Frozen 지도 | 통과 | 반복 발행 시 원래 acquisition stamp 유지 |
| 지도 도구 | 통과 | identity 재격자화 → 새 site bundle → checksum/frame/resolution 검사; 회전·translation 단위 시험 |
| 독립 환경 | 통과 | 원 workspace 환경 변수를 제거하고 새 패키지 경로·Python import 확인 |
| 공개 launch 5종 | 인자 로딩 성공 | sensor, localize, mapping, mapping_pcd, navigate |

PCD mapping launch도 sensor 입력 없이 시작하여 `Node init finished`까지 확인했습니다. 실제 PCD 취득 품질은 이 시작 시험에 포함되지 않습니다. 원본 FAST-LIO의 선택적 debug log 디렉터리 안내가 출력될 수 있습니다. 시험 종료를 위한 SIGINT 뒤 upstream process의 `exit code -2`가 표시되는 경우를 시작 실패와 구분했습니다.

`rosdep check`는 현재 PC의 apt 상태에서 `python3-open3d`, `libapr1-dev`, `libaprutil1-dev` 세 항목을 미충족으로 보고했습니다. 이 PC의 Open3D는 이미 설치된 Python 패키지를 사용하여 import·시작 검사를 통과했습니다. 새 장치에서는 [설치 안내](INSTALL.md)의 apt 설치와 `rosdep install`을 완료해야 하며, 깨끗한 Jetson에서 해당 설치 절차 전체를 실행한 결과는 아직 없습니다. [의존성 검사 출력](../validation/rosdep_check.log).

자동 검사에는 timestamp decoder, row padding·잘못된 단위 거부, IMU 180° 회전, 높이 좌표·누적·저장, 후처리, planner core, controller, 배포 입력 계약 검사가 포함됩니다. 기존에 포함된 회귀시험도 함께 실행했습니다. 숫자 자체를 실물 동작 보증으로 해석하지 않습니다.

## 장애물 통합 시험의 의미

합성 평면 지도에서 시작점 (1,1)과 목표 (4,1)를 사용했습니다. 두 모드 모두 최초 61점 경로를 만들었고, 새 장애물을 넣자 61점 우회 경로가 생성되며 Y방향 최대 이탈은 약 0.875 m였습니다. 경로를 따른 속도 출력, 근접 장애물에 대한 최종 속도 0, pointcloud 갱신 중단에 대한 최종 속도 0을 각각 확인했습니다.

이는 합성 TF/odom/cloud를 사용한 ROS 연결·정책 시험입니다. 로봇을 움직이며 목표에 도착한 폐루프 실물 시험이나 DWB의 복잡한 군중 회피 성능 시험은 아닙니다. Nav2의 우회 경로는 planner action으로 확인했고, PP의 우회는 경로 무효화 후 재계획으로 확인했습니다.

설치된 Humble Collision Monitor의 데이터 중단 동작만으로 최종 정지를 보장할 수 없었던 부분은 마지막 `command_watchdog`에 cloud freshness 검사를 추가하여 해결했습니다. 이 node는 cloud뿐 아니라 odometry·costmap·명령의 갱신도 확인합니다.

근거: [Nav2 결과](../validation/nav2_smoke.json), [Pure Pursuit 결과](../validation/pursuit_smoke.json), [Localization 시작 결과](../validation/localization_startup.json), [독립 환경](../validation/standalone_environment.json), [지도·launch 도구 결과](../validation/map_tools/result.json).

## 기존 GT snapshot의 오프라인 후처리

기존 시뮬레이션에서 저장한 world frame의 5 cm 지도, 유효 51,812셀에 복사한 후처리 패키지를 실행했습니다. 원본을 덮어쓰지 않았습니다.

| 항목 | Conservative candidate |
|---|---:|
| 유효 셀 수 | 51,812 — 유지 |
| Unknown mask | 유지 |
| 보호 경계에서 1 μm 초과 변경 | 0셀 |
| 전체 유효 셀의 높이 변경량 P95 | 1.413 mm |
| 높이 변경량 최대 | 5.153 mm |

이는 **후처리가 바꾼 양**이며 GT에 대한 절대 오차를 뜻하지 않습니다. 세계 모델에 대한 독립 오차 평가는 이번 실행에 추가하지 않았습니다. 평균 필터 baseline은 경계까지 수정하므로 배포 후보로 자동 채택하지 않습니다. [후처리 비교 보고서](../validation/postprocess/report.html), [원시 metrics](../validation/postprocess/metrics.json).

## 아직 확인하지 않은 사항

- MID-360S 실물의 실제 packet 시간 기준, cloud/IMU 동기, 180° 장착 축, 센서·IMU 원점 간 치수.
- Jetson Orin NX 16 GB / 25 W의 실제 JetPack, 전체 동시 실행 CPU/GPU/RAM, 발열 및 지속 처리율.
- 새 센서로 취득한 PCD에서 FAST-LIO / GenZ의 실제 localization 정확도와 복도 퇴화 상황.
- 실제 사이트의 PCD·occupancy·height 정합 잔차, 대표 경로 QC, 좁은 높이 경계에서 지역 controller의 움직임.
- 로봇 보행 제어기 연결, 실제 정지거리, 통신 단절, 센서 0.6 m 내부 사각지대에 대한 시스템 수준 검증.

## 재현 자료

빌드·시험 출력은 [validation](../validation)에 보관합니다. 반복 실행 명령은 [운영 안내](OPERATIONS.md#i-반복-검증)에 있습니다. GPU 검사는 다음과 같이 별도로 실행했습니다.

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ROS_DOMAIN_ID=176 FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
  python3 -m pytest -q src/rubi_height_mapping/test/test_gpu.py \
  src/rubi_height_mapping/test/test_ros_odometry.py
```

`test_ros_odometry.py`는 자체 격리 domain 184를 사용합니다. 시험용 domain과 실제 로봇 domain을 공유하지 않습니다.

## 2026-10-15 GPU 입력·지도 내보내기 경량화

현재 PC의 RTX 5090 / CuPy 13.6.0에서 이전 백업과 수정본을 교대로 측정했습니다. 8 × 8 m, 5 cm grid, 합성 평면·5/15 cm 단차 25,600점, 워밍업 15회 뒤 각 150회이며 매 구간 CUDA 완료를 기다렸습니다. 아래 값은 전체 ROS latency나 Jetson 결과가 아닙니다.

| 구간 | 이전 중앙값 / P95 | 수정 후 중앙값 / P95 |
|---|---:|---:|
| 검증된 XYZ의 GPU 전송·융합 | 0.439 / 0.607 ms | 0.312 / 0.418 ms |
| elevation·variance 내보내기 + 두 메시지 직렬화 | 2.803 / 3.045 ms | 0.222 / 0.409 ms |

입력 decoding·거리 필터·pose 대기·move_to·주기 variance 갱신·DDS 전송·전역 wrapper·RViz는 이 구간 시험에 포함되지 않습니다. GPU는 다른 실행 중인 프로그램과 공유하므로 전용 장비의 반복 측정과 다를 수 있습니다. 7개 내부 layer가 허용 오차 내에서 일치했으며 최대 절대 차이는 9.54e-7이었습니다. 알려진 flat/step 지형 회귀검사도 통과했습니다.

빌드 성공 후 contract, odometry cache, ROS wire serialization, GPU estimator, 실제 GPU + ROS direct-odometry 연결 검사를 실행하여 **17 passed**를 확인했습니다. 별도 domain 184를 사용했으며 사용자의 실행 중인 mapper는 재시작하지 않았습니다. 이전 250개 전체 workspace 검사를 이번에 다시 수행한 것은 아닙니다.

수정 내용은 CPU에서 이미 finite XYZ를 확인한 입력의 중복 GPU 검사를 제거하고, 두 layer의 host 복사를 합치고, Python float 목록 대신 float32 buffer로 ROS 메시지를 만드는 것입니다. 해상도·pose 정책·높이 추정식·visibility cleanup·traversability 계산을 유지했습니다. 기존 single-layer API와 기본 입력 검증 경로도 유지했습니다.

근거: [벤치마크 결과](../validation/performance_20261015/gpu_optimization.json). 재현 도구는 `tools/benchmark_height.py`이며 source 후 다음처럼 실행합니다.

```bash
python3 tools/benchmark_height.py \
  --output validation/jetson_height_microbenchmark.json
```

이전 버전과 비교하려면 `--baseline /path/to/copied/rubi_height_mapping`을 지정합니다. 현재 PC 백업은 `backups/gpu_light_20261015_131543/rubi_height_mapping`입니다. 합성 벤치마크만으로 Jetson NX/Nano에서 FAST-LIO·navigation 동시 실행이 충분하다고 판단하지 않습니다. JetPack·전력 모드·열 상태를 기록하고 실제 센서 또는 동일 bag으로 전체 파이프라인의 CPU·GPU·메모리·누락률·지연을 측정해야 합니다.

별도 12초간 현재 domain 45를 읽기 전용으로 조회했지만 raw/deskewed/localization의 새 메시지는 수신되지 않았습니다. 따라서 이번 작업에서 실제 입력 경로 A/B 지연을 측정했다고 주장하지 않습니다. [조회 기록](../validation/performance_20261015/input_probe.json).

## 2026-10-15 대표 ray 선택·구조 최적화

이 절은 snapshot fusion 이전 단계의 기록입니다. 현재 0.3.1 결과는 아래
`2026-10-15 snapshot fusion` 절을 기준으로 합니다.

구현 파일: `rubi_height_mapping/ray_selection.py`, `kernels.py`, `core.py`.
선택 작업을 기존 GPU 점별 계수 계산에 합쳤습니다. 높이 입력은 줄이지 않으며, 3D voxel별 원래 점 하나만 cleanup loop에 들어갑니다. 기존 `all`의 CUDA 코드·함수 인자·error buffer 할당 방식은 유지하고, voxel 경로의 GPU 임시 버퍼는 재사용합니다. 새 의존성은 없습니다.

RTX 5090 / CuPy 13.6.0, 8 × 8 m / 5 cm 지도에서 15회 워밍업 뒤 각 200회 교대로 측정했습니다. 매 입력마다 CUDA 완료를 기다린 wall-time 중앙값입니다. 입력 전송·융합을 포함하고 pose 대기, 지도 이동, ROS 전송, wrapper, RViz는 제외합니다. 실행 중인 다른 GPU 프로그램과 자원을 공유하며 Jetson 결과는 아닙니다.

| 입력 | 점 수 → 선택 ray | 수정 전 | 수정 후 all | 수정 후 voxel |
|---|---:|---:|---:|---:|
| 희소 합성 지형 | 6,400 → 6,400 | 0.209 ms | 0.208 ms | 0.212 ms |
| 조밀 합성 지형 | 25,600 → 6,400 | 0.283 ms | 0.282 ms | 0.288 ms |
| 중복이 많은 합성 지형 | 102,400 → 6,400 | 0.590 ms | 0.588 ms | 0.407 ms |
| 기존 저장 점군의 ROI | 32,307 → 6,538 | 0.267 ms | 0.267 ms | 0.266 ms |

큰 중복 입력은 수정 전 대비 약 31.0% 감소했습니다. 현재 저장 입력은 ray 수가 약 80% 감소해도 총 시간이 거의 같았으며, 수정 후 all과 voxel의 차이도 미미했습니다. 따라서 모든 입력에서 빠르다고 판단하지 않습니다. selector의 GPU 버퍼는 32,307점에서 0.875 MiB, 102,400점에서 3.5 MiB였습니다.

저장 점군은 이전 시뮬레이션 capture의 XYZ를 사용하되 R=I, t=(0,0,0.8)의 합성 고정 pose로 반복했습니다. 원래 주행·pose를 재생한 실험이 아니므로 그 행은 계산비용 확인용입니다. 이 반복에서는 all/voxel 높이 차이가 최대 약 73.0 mm, 통과한 관측 수도 달랐습니다. 입력점을 삭제하지 않는다는 것이 후속 상태·높이 결과까지 완전히 같다는 뜻은 아닙니다.

기존 kernel에는 같은 셀의 variance·유효성 등을 병렬로 읽고 쓰는 순서 민감성이 있습니다. 동일 저장 입력을 215회 반복한 추가 확인에서도 같은 버전 재실행 간 차이가 관측됐습니다. 최종 반복성 기록에서는 수정 전 최대 약 1.20 mm, 수정 후 all 최대 약 80.3 mm였습니다. all 경로는 CUDA 소스·signature와 임시 error buffer 할당 방식을 기존과 동일하게 유지했지만, 복잡한 입력의 수치 결과가 허용 오차 내에서 재현된다는 검증은 통과하지 못했습니다. 단순한 반올림 오차라고 단정하지 않으며 병렬 상태 갱신 순서·메모리 배치 영향 등을 추가로 분리해야 합니다. 가상 pose에서의 반복 시험이므로 실제 월드의 높이 오차로 해석하지 않지만, 실제 취득 정확도를 보장하는 근거로도 사용할 수 없습니다. 이 때문에 voxel을 배포 기본값으로 바꾸지 않았습니다. [반복성 기록](../validation/ray_optimization_20261015/repeatability.json).

합성 평면/5 cm·15 cm 단차에서는 유효 mask와 높이가 허용 오차 내에서 일치했습니다. 모든 높이 관측이 융합 계수에 포함되는지 별도로 검사했습니다. 알려진 수평 normal과 40개의 동일 ray를 사용하는 고립 잔상 시험에서 old ghost 제거는 **all 1 scan / voxel 9 scans**였습니다. 가려진 옆 셀은 유지되고 미관측 셀은 unknown으로 남았습니다. 10 Hz를 가정하면 9 scan은 약 0.9초지만, 실제 환경의 제거시간을 보장하는 수치는 아닙니다.

빌드 성공, mapper 전체 검사와 bringup 계약 검사 **30 passed**. GPU 선택·메모리 재사용·빈 입력·역방향 장착·음수 좌표·Z datum·hash 포화 fallback·좌표 범위 fallback·visibility 비활성·단차·가려짐을 포함했습니다. ROS direct-odometry 시험은 voxel로 실행했고, 기존 TF pipeline은 all로 실행하여 좌표·저장/복원·시각 reset 동작을 확인했습니다. 사용자 실행 중인 mapper는 재시작하지 않았습니다.

- [측정 JSON](../validation/ray_optimization_20261015/benchmark.json)
- [시험 로그](../validation/ray_optimization_20261015/tests.log)
- [공개 launch 인자](../validation/ray_optimization_20261015/launch_args.log)

재현 예시:

```bash
source install/setup.bash
python3 tools/benchmark_rays.py \
  --baseline backups/ray_selection_20261015_135924/rubi_height_mapping \
  --output validation/ray_compare.json
```

`--baseline`은 선택 사항입니다. 원본 backup이 없는 Jetson에서도 all/voxel 비교가 가능합니다. `--cloud capture.npz`는 `xyz` 배열을 가진 capture를 비용 비교에 추가합니다. 정식 품질 비교에는 같은 pointcloud/pose 기록을 함께 재생해야 합니다.

## 2026-10-15 snapshot fusion — 0.3.1

이 절은 위 초기 ray 최적화 이후 적용한 수정의 결과입니다. 이전 시험 결과를
삭제하지 않았으며, 당시의 코드와 현재 코드의 결과를 구분합니다.

변경은 **이전 지도 snapshot → 높이/visibility 후보 → 셀별 최종 확정**입니다.
판정 중 기존 지도에 쓰지 않고, 채택된 관측을 같은 scan의 cleanup보다 우선합니다.
float32 관측 후보를 float64로 합산하고 최종 지도는 float32로 유지합니다.
기본 ray_mode=all, 좌표/pose 계약, 발행 주기, 높이·noise 식, ray traversal은 유지했습니다.
새 패키지 의존성은 없습니다. 후보 합산 및 판정 순서가 달라져 구버전 결과와의
수치적 동일성 자체를 합격 조건으로 삼지는 않습니다.

### 반복성 검사

같은 저장 점군의 ROI 32,307점을 R=I, t=(0,0,0.8)의 **합성 고정 pose**로
215 scan씩 네 번 실행했습니다. 실제 주행 pose 재생이나 GT 정확도 검사가 아닙니다.

| 비교 | 실행 간 최대 높이 차이 | 유효 셀 mask |
|---|---:|---|
| 직전 코드 all | 7.861 mm | 동일 |
| snapshot all | 0 m 관측 | 동일 |
| snapshot voxel | 0 m 관측 | 동일 |
| snapshot all, 입력점 순서 반전 | 0 m 관측 | 동일 |

이번 시험에서는 일곱 내부 층도 bit 단위로 같았습니다. 모든 GPU·장면에서 항상
bit 단위로 같다는 보증은 아닙니다. 합격 기준은 같은 유효 mask와 최대 높이 차이
2e-6 m 이하였습니다. voxel의 대표점은 입력 index로 선택하므로 voxel의 점 순서
반전은 같은 ray 집합 시험으로 취급하지 않았습니다. all/voxel 지도끼리 같다는
주장도 아닙니다. 수정 전 수치가 과거 80.3 mm 기록과 다른 것은 서로 다른 실행이며,
그 자체가 기존 결과의 순서 민감성을 보여줍니다. 원래 주행에서 확인한 localization
오차와 이 고정 입력 반복성 문제는 서로 다른 실험입니다.

### 처리 시간과 메모리

RTX 5090 / CuPy 13.6.0 / 8×8 m / 5 cm, 15회 warmup + 200회 교차 측정.
CPU→GPU 복사와 GPU 완료를 기다린 core call 시간이며, decoding·pose 대기·grid 이동·
ROS/DDS·발행·wrapper·RViz는 포함하지 않습니다. 공유 PC에서 측정한 한 실행의 결과입니다.

| 입력 | 수정 전 all 중앙값 / p95 | snapshot all 중앙값 / p95 | snapshot voxel 중앙값 / p95 |
|---|---:|---:|---:|
| 저장 ROI 32,307점 | 0.270 / 0.436 ms | 0.285 / 0.486 ms | 0.288 / 0.477 ms |
| 조밀한 합성 102,400점 | 0.641 / 0.864 ms | 0.640 / 0.939 ms | 0.468 / 0.707 ms |

기본 all 경로의 32k 중앙값은 약 5.4% 늘었습니다. 이 변경은 안정성 개선이며
전체 파이프라인의 속도 향상으로 설명하지 않습니다. 기본 grid에서 추가 상주 버퍼는
2,099,528 bytes(약 2.0 MiB)입니다. FP64 후보 합산 비용은 Jetson에서 다시 측정해야 합니다.

### 기능 검사와 설치

mapper 전체 + bringup 계약 검사 **39 passed**. 새 GPU 검사는 snapshot 불변,
같은 scan의 outlier가 다른 점의 gate를 넓히지 않음, 관측/cleanup 충돌 시 관측 우선,
outlier-only 분산 증가, 음수 upper bound의 순서 독립 집계, unknown 보존,
복잡한 합성 scan의 215회 반복, 버퍼 재사용, strided XYZ·빈 입력·clear 후 재사용을 포함합니다.
기존 0/5/15 cm 단차, 이동 grid, 가려짐, ROS TF·직접 odometry, 전역 저장/복원,
시뮬레이션 시계 되감기 및 제외 의존성 차단 검사도 통과했습니다.

사용자의 실행 중인 mapper·simulator·localizer는 재시작하지 않았습니다.
source 및 설치에 수정본을 반영했으며 새 mapping launch부터 사용합니다.

- [측정 결과](../validation/snapshot_fusion_20261015/benchmark.json)
- [검사 로그](../validation/snapshot_fusion_20261015/tests.log)
- [빌드 로그](../validation/snapshot_fusion_20261015/build.log)
- [읽기·쓰기 구조](height_snapshot.svg)
- 원본 백업: `backups/snapshot_fusion_20261015_152106/` (SHA-256 포함)

재현 명령 (`capture.npz`는 xyz를 가진 저장 입력으로 지정):

```bash
source install/setup.bash
python3 tools/validate_snapshot.py \
  --baseline backups/snapshot_fusion_20261015_152106/src/rubi_height_mapping \
  --cloud capture.npz \
  --output validation/snapshot_compare.json
```


## 2026-10-15 mapping.yaml 운영 설정

`legged_bringup` 빌드 후 설치된 launch를 실제 LaunchContext로 펼쳐,
기본 FAST-LIO 시뮬레이션 값과 CLI의 실물 clock/RViz 변경, 기존 raw/GT 변경이
하위 mapper·wrapper에 전달되는지 확인했습니다. 상세 config 파일 해석과
저장 경로의 홈 디렉터리 확장도 확인했습니다. 실행 중인 ROS 노드는 건드리지
않는 launch 확장 검사이며, 새 live 주행을 했다는 의미는 아닙니다.

- [확인 스크립트](../validation/mapping_yaml_20261015/check_launch.py)
- [실제 전달 설정](../validation/mapping_yaml_20261015/expanded.json)
- 백업: `backups/mapping_yaml_20261015_165827/legged_bringup`

## Navigation recovery — 2026-10-15

- Build: legged_navigation, legged_bringup passed.
- C++ core 14 tests and Python delivery 5 tests passed.
- `tools/validate_navigation_recovery.py` on isolated domain 175 passed:
  missing fixed mount (even with dynamic TF) blocks final command; inverted fixed
  mount transforms coordinates correctly; near-range exclusion preserved; missing
  sensor stops; rear target produces rotation without translation; aligned heading
  resumes translation; empty path stops.
- Existing `tools/smoke_navigation.py` passed for pursuit (domain 174) and nav2
  (domain 173): path output, obstacle reroute, near-obstacle stop, stale-sensor stop,
  and frozen acquisition timestamp preservation.
- Duplicate launch on live domain 45 rejected before node creation.
- Live sensor observation: 69 transformed clouds in 8 seconds; base_link frame;
  last sampled cloud 34,056 points, zero points within configured Stop volume.
  Collision input reported ready. No previous collision future-extrapolation errors
  observed in the new launch log at inspection time. Costmap and collision monitor active;
  /cmd_vel and /legged/cmd_raw each have one publisher.
- Live goal tracking after this patch remains a user-observed trial. Synthetic tests
  do not prove RUBI gait stability, hardware stopping distance, or eliminate oscillation.

Evidence: validation/navigation_recovery/integration.json, live_input.json, live.log;
validation/pursuit_smoke.json and nav2_smoke.json. The fixed-mount test initially had
an incorrectly formed test quaternion; setting the intended 180-degree quaternion
(x=1,w=0) corrected that fixture before the successful run.

## Tracking deviation recovery and Nav2 RPP — 2026-10-15

- Observed live sequence in `navigation_recovery/live.log`: Stop polygon stop,
  then cross-track errors 0.409 and 0.415 m exceeding the 0.4 m limit. The planner
  kept the valid old route because the candidate route cost more. The specific
  obstacle points inside the polygon were not captured at this stop.
- Added a deviation-triggered hard replan, including on unchanged map content.
  Stop until a traversability-checked replacement from the current robot position
  is available. Failed routes retry; stale dynamic poses are rejected. All-static
  zero-stamp TF remains valid for static fixtures.
- `tools/validate_tracking_replan.py`, isolated domain 176: unchanged-map
  deviation recovery, stop before replacement, same goal, no repeated replacements
  after recovery, blocked start stays stopped, cleared start resumes planning,
  stale pose not used. All passed (`navigation_recovery/tracking_replan.json`).
- `tools/smoke_navigation.py nav2 --controller rpp`, domain 173: actual RPP plugin
  creation and local costmap/controller lifecycle activation, height path output,
  controller motion commands, global obstacle detour (0.875 m lateral deviation),
  near-obstacle stop, stale-sensor stop, frozen timestamp unchanged. All passed
  (`nav2_rpp_smoke.json`, `nav2_rpp_launch.log`). Detour is a planner test, not a
  claim that RPP generates local bypass trajectories.
- Planner package suite initially caught two static-TF replan regressions after
  pose-age checks were added. Allowing zero-stamp all-static transforms fixed them;
  the affected launch test and the dynamic stale-pose test were rerun successfully.
  Final colcon result: planner 167 reported tests and bringup 6, zero failures.
- Backups: `backups/local_costmap_20261015_191603/`.
- Original Stop geometry and costmap footprint were retained. No actual RUBI gait,
  pillar passage, hardware stopping-distance or reduced-oscillation claim is made.
- Live domain 45: replaced only navigation with `mode:=nav2 controller:=rpp`;
  Gazebo and FAST-LIO stayed running. Controller and local costmap both active,
  one final `/cmd_vel` publisher. A subsequent goal was received externally (no
  goal sent by the assistant). During a 6-second observation, 44 transformed clouds
  had zero Stop-volume hits and final forward commands reached 0.5 m/s with guard
  state `pass`. This is not a reconstruction of the earlier stopped position and
  does not establish goal arrival or gait stability. Evidence:
  `navigation_recovery/live_rpp.log`, `navigation_recovery/current_stop.json`.
- Later in that live trial the fixed Stop polygon fired again, then RPP reported
  `detected collision ahead` and the navigation goal aborted. A subsequent 5-second
  capture contained 28 clouds with 37–305 Stop-volume hits. Last cloud: 139 hits,
  body x [-0.700,-0.674] m and y [0.391,0.450] m — the rear-left corner of the fixed
  rectangle, far outside the costmap's 0.30 m radius. This confirms a restrictive
  fixed-zone stop at the sampled position, not that the predicted forward/turning
  trajectory is collision free. At a later costmap snapshot the body-center cost
  was 35 locally and 170 globally (not lethal); the RPP predicted arc at the abort
  was not recorded, so that additional collision event remains unlocalized.
  Evidence: `navigation_recovery/rpp_stop.json`, `rpp_stop_cloud.npz`,
  `rpp_stop_cloud.png`, `stop_costmap.json`. Navigation is still running in RPP mode,
  but this goal has aborted and final commands are zero. No further goal was sent.
