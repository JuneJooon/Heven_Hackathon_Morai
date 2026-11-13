# 2026 성균관대 자율주행 해커톤 — MORAI 기준 및 작업 맥락

## 출처와 사용자 의도

- 사용자 제공일: 2026-11-13, Asia/Seoul.
- 문서 제목: **2026 성균관대 자율주행 해커톤**, 21쪽.
- 원본 사본: [PDF](2026_skku_autonomous_driving_hackathon.pdf).
- 검색 가능한 전체 추출문: [텍스트](2026_skku_autonomous_driving_hackathon.txt). 표는 추출 과정에서 열 정렬이 흐트러질 수 있으므로 불명확하면 PDF를 확인한다.
- 원본 제공 경로: `/home/juna/Downloads/004afb34-a832-4d00-ba38-d6f626a6e6ae_2026_성균관대_자율주행_해커톤.pdf`.
- 사용자 최초 요청: “이거 6번부터 morai 짜보자”. 후속 요청: “이거야 이거는 절대까먹지마”.
- 기준은 **6장 MORAI(인쇄 14~16쪽)**와 **7장 공통 토픽과 TF(16~17쪽)**, **8.3 MORAI 규정(19쪽)**이다. **10.2는 운영진 절차(20~21쪽)**로 참가자 코드의 구현 지시와 구분한다.
- 노션에서 찾았던 “헤븐 자율주행대회 음영구역팀 과제”는 다른 문서다. 그 문서의 DWA/FAST-LIO2 파라미터를 이번 해커톤 요구사항으로 가져오지 않는다.

## 경기와 우선순위

- K-City 전역 경로 한 바퀴, 약 2.2km. 체크포인트 14개를 반경 3m 이내로 순서대로 통과한다.
- MORAI 공식 시도는 팀당 1회, 제한시간 600초. 시작 후 20초 이내 출발한다.
- 기본 주행 속도 3m/s는 제한시간 내 완주에 부족하다고 안내한다. 6.2에서 GPS 필터링·GPS 사이 보간, 제한속도 및 곡률 기반 속도 계획, 조향/속도 이득 개선을 제안한다.
- 제한속도: 기본 60km/h(약 16.67m/s), **CP4→CP5 어린이 보호구역 30km/h(약 8.33m/s)**, **CP10→CP12 고속도로 제한 없음**. 제한 없음이 차량·제어기의 능력과 무관한 속도를 의미하지 않는다.
- 차선(실선·중앙선) 침범과 과속: 위반 순간 15초, 지속 시 3초마다 15초 추가. 주행 불능 체크포인트 복귀는 회당 15초.
- 완주 보정 기록 = (실제 주행시간 + 패널티 시간) × 노이즈 배율. 완주 팀이 미완주 팀보다 앞선다.
- GPS/IMU 노이즈 배율: 켬/켬 1.0, 끔/켬 1.3, 켬/끔 1.1, 끔/끔 1.5. 설정 선택은 추후 실제 주행 검증과 사용자 의사에 따라 정한다.
- 종합 비중: MIT 60%, MORAI 30%, 발표 10%. 발표는 5분, 참신성 30%·기술 이해도 70%.
- 최종 제출: 2026-11-14 08:00. MORAI 연습 PC 이용은 2026-11-13 20:00부터 다음 날 05:00까지 팀별 배정 시간에 한다.

## 코드·실행 계약

- 참가자 수정 위치: `src/hk_morai/hk_morai/`의 `localization.py`, `planner.py`, `control.py`, `lidar_perception.py`, `camera_perception.py`.
- 경로/코스: `src/hk_morai/course/morai_global_path.csv`(0.5m 간격)와 `morai_course.json`(원점·출발·체크포인트·도착선·속도 구간).
- 노트북 컨테이너에서는 빌드 확인만 가능하다. 실제 MORAI 주행은 운영진 PC 2대에서만 가능하다.
- 운영진 PC에 `morai_ws/src`만 가져가 해당 PC의 ROS 2 Humble 환경에서 다시 빌드한다. 노트북의 `build/`, `install/`은 가져가지 않는다.
- 참가자 빌드·실행 예시: `colcon build && source install/setup.bash`, `ros2 launch hk_morai bringup.launch.py`.
- 선택 인자: `perception:=true`, `rviz:=true`, `target_speed:=5.0` 등.
- MORAI에는 `/clock`이 없다. **`use_sim_time=false`**. 센서 타임스탬프는 UDP 수신 시각이다.
- START 이벤트를 받아 출발하고, RESPAWN에서 체크포인트 자세에 맞춰 상태를 초기화하며, END에서 멈춘다. 노드 실행만으로 출발하지 않는다.
- 공식 주행 직전에는 설정 파일·실행 인자만 변경할 수 있다. 현장에서 조정할 값은 미리 ROS 파라미터로 노출한다. 시도 중 PC 조작은 불가하다.
- 운영진 PC의 시스템 전체 설치(`sudo apt`, 시스템 `pip`)는 금지. 팀 폴더 가상환경은 가능하다는 대회 안내가 있다. 이를 근거로 설치나 운영진 도구 실행을 자동 수행하지 않는다.

## 공개 토픽과 QoS

| 토픽 | 형식 | 주기 / 용도 |
| --- | --- | --- |
| `/hk/sensors/gps/fix` | `sensor_msgs/NavSatFix` | GPS 20Hz |
| `/hk/sensors/imu/data` | `sensor_msgs/Imu` | IMU 50Hz, 브리지에서 지도 기준 자세 제공 |
| `/hk/sensors/lidar/points` | `sensor_msgs/PointCloud2` | VLP-16, 10Hz |
| `/hk/sensors/camera/{front,left,right,traffic_light}/compressed` | `sensor_msgs/CompressedImage` | JPEG, 각 20Hz |
| `/sim/events` | `hk_interfaces/SimEvent` | START, RESPAWN, END |
| `/hk/local/odom` | `nav_msgs/Odometry` | localization 출력, 50Hz |
| `/hk/plan/local_path` | `nav_msgs/Path` | planner 출력, 20Hz |
| `/hk/control/command` | `hk_interfaces/CtrlCmd` | 제어 출력, 기본 50Hz |

- 센서 구독: SensorDataQoS(best effort).
- 제어 명령: reliable, depth 1.
- 이벤트: reliable, transient local, depth 10.
- 공개 센서·토픽·경로·코스 파일만 사용한다. `/dev/ground_truth`는 연습 비교용이며 공식 주행 코드의 입력으로 사용하지 않는다. 시뮬레이터 내부 파일이나 웹 화면 연결을 읽지 않는다.
- 문서가 IMU의 제공 자세를 “정답값”이라고 표현하지만 `/hk/sensors/imu/data`는 명시적으로 제공되는 참가자용 센서 입력이다. 이를 금지된 `/dev/ground_truth` 입력과 혼동하지 않는다.

## 좌표계·TF

- `map = UTM 52N - (302595, 4124145)`m. 런타임에는 코스 파일의 원점을 읽는다.
- `base_link`: 후륜 축 중심을 지면에 내린 점. x 전방, y 좌측, z 상방.
- localization이 `map → base_link`와 동일한 pose의 odometry를 발행한다. 센서 정적 TF는 bringup의 robot_state_publisher가 `config/sensor_mounts.yaml`에서 생성한다.
- 정적 트리: `base_link → rear_axle_link`(z +0.3685m) → GPS, IMU, LiDAR, 카메라.
- LiDAR 위치: base_link 기준 (1.15, 0, 1.7685)m. GPS/IMU 높이: 1.5685m.
- 영상 frame_id는 `camera_<name>_link`(x 전방). 픽셀 투영은 `camera_<name>_optical_frame`(z 전방, x 우측, y 하방)을 사용한다.

## 확인된 구현 상태와 다음 작업

2026-11-13 문서를 저장한 시점의 확인 사항이며, 이후 구현에 맞게 갱신한다.

- `hk_morai`는 제공된 기본 코드 상태: GPS 원본 위치와 0.5초 차분 속도, 전역 경로 앞 30m/뒤 2m, Stanley 조향 + PI 속도 제어(3m/s).
- LiDAR/카메라 노드는 수신만 하며 기본 launch에서는 실행되지 않는다.
- Python 파일 11개의 AST 문법 검사는 통과했다. 당시 Python 환경에서 `rclpy`와 `ament_index_python`은 보였지만 `hk_interfaces`는 import 경로에서 발견되지 않았다. ROS 실행 검증 완료를 의미하지 않는다.
- **6장 대응 주행 알고리즘 수정 완료:** GPS 예측/보정·회귀 속력, 곡률/제한속도/감속거리 프로파일, 조향 변화율·출발 램프, 입력 단절 제동. 상세와 검증 범위는 `src/hk_morai/README.md` 참조. 실제 MORAI 주행은 미검증.
- **직선 반복 제동 대응(2026-11-13):** 사용자가 GNSS/IMU 노이즈 OFF 상태에서 약 10km/h 재가속·제동 반복을 보고했다. 실행은 속도 인자 없는 `ros2 launch bringup.launch.py`로 기억한다고 답했으며 실제 실행본·런타임 로그는 확보하지 못했다. 입력 단절 후 0으로 되감던 속도 램프를 현재 속력에서 재개하고, 제동 시작/해제 히스테리시스(0.3/0.1m/s), 속력 회귀 창 0.3초·alpha 0.5, 적용 목표속도·정지 사유 로그를 추가했다. 17개 회귀 테스트 통과, MORAI 현장 재주행은 필요하다. 노이즈 OFF여도 센서 지연·단절이 없다는 뜻은 아니다.
- 우선 후보: 제한속도·곡률·감속거리를 고려한 속도 계획, GPS 위치/속력 안정화, 제어 이득 및 데이터 끊김 처리. 실제 MORAI 주행 검증은 운영진 PC에서 필요하다.

- **참고 ZIP 추가 반영:** Downloads의 `heven-ad-2026-dev.zip`을 직접 비교해 독립 제동 P 이득, 해제 경계 deadband 차감, 갱신 후보 기준 anti-windup을 적용. 오프라인 20개 테스트 통과. 참고 구현의 바퀴 속도 의존성 때문에 위치 추정기는 통째로 이식하지 않음. 사용자는 약 3시간 뒤 현장 디버깅 가능하다고 알림. 상세는 README 마지막 절 참조.
