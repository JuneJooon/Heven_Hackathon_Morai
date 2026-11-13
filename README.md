# HEVEN Hackathon 2026 — MORAI

2026 성균관대 자율주행 해커톤의 MORAI 주행 코드입니다.

## Build

ROS 2 Humble과 대회용 `hk_interfaces`가 제공되는 운영진 PC에서 실행합니다.

```bash
colcon build --packages-select hk_morai
source install/setup.bash
```

## Run

```bash
ros2 launch hk_morai bringup.launch.py
```

약 10.8 km/h로 먼저 검증하려면 다음과 같이 실행합니다.

```bash
ros2 launch hk_morai bringup.launch.py target_speed:=3.0
```

MORAI에는 `/clock`이 없으므로 `use_sim_time=false`를 유지합니다. START 이벤트를 받은 뒤 출발합니다.

구현 및 파라미터 설명은 [패키지 문서](src/hk_morai/README.md)를 참고하세요.

## Test

ROS 메시지 생성 패키지가 없는 환경에서도 제어 계산을 확인할 수 있습니다.

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q src/hk_morai/test
```
