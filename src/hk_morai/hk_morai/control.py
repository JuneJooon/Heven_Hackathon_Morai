"""HEVEN 2026 MORAI 기본 코드 3/3: 제어 (control). 참가자가 고칠 곳.

localization.py의 /hk/local/odom(위치·방향·속력)과 planner.py의 /hk/plan/local_path(따라갈 경로)를 받아
50 Hz로 /hk/control/command에 구동·제동·조향을 냅니다. 고칠 곳은 아래 `drive()`입니다.

  drive()에서 쓸 수 있는 값                       돌려줄 값
  self.x, self.y   지도 위치 (m, 뒤차축)            accel  0~1   (구동)
  self.yaw         진행 방향 (rad)                 brake  0~1   (제동)
  self.speed       속력 (m/s)                      steer  -1~1  (+ 왼쪽, - 오른쪽, 1 = 35.4°)
  self.path        따라갈 경로 [(x, y, 경로 방향), ...] (비었으면 경로 끝)
  self.nearest(x, y)  경로에서 (x, y)와 가장 가까운 점 (x, y, 경로 방향)

Stanley + 조향 변화율 제한, 곡률/법정속도/감속거리 기반 속도 계획과 PI 제어.
입력 시간 초과와 빈 경로에서는 제동합니다. 현장 주행 검증 및 이득 조정이 필요합니다.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from nav_msgs.msg import Odometry, Path
from hk_interfaces.msg import CtrlCmd

from .sim_events import SimEvents
from .course import declare_file_params, load_route, load_course
from .speed_profile import build_profile

# 조정할 값은 config/bringup.yaml의 파라미터입니다(공식 주행 직전에는 코드가 아니라 이 파일만 바꿀 수 있음).
# 아래는 yaml에 값이 없을 때 쓰는 기본값입니다.
TARGET_SPEED_DEFAULT = 8.0
PARAMS = {
    'lateral_accel': 1.8,
    'planning_decel': 1.5,
    'speed_margin': 0.5,
    'zone_buffer': 5.0,
    'input_timeout': 0.5,
    'launch_accel': 1.0,
    'steer_rate': 1.0,
    'target_speed': TARGET_SPEED_DEFAULT,   # 목표 속력 (m/s). 실행할 때 target_speed:=… 로도 바꿀 수 있습니다.
    'k_stanley': 1.0,      # 경로에서 옆으로 벗어난 거리에 대한 조향 세기
    'k_soft': 1.0,         # 저속에서 조향이 너무 세지지 않게 하는 값 (m/s)
    'kp': 0.5,             # 속력 P 이득
    'ki': 0.05,            # 속력 I 이득
    'brake_gain': 0.6,     # 너무 빠를 때 제동 세기
    'brake_kp': 0.5,       # 가속 kp와 독립적인 제동 P 이득 (최종 배율은 brake_gain)
    'brake_on_error': 0.3,  # 목표보다 이만큼 빠를 때 제동 시작 (m/s)
    'brake_off_error': 0.1, # 제동 해제 기준 (m/s), 시작 기준보다 작아야 함
}
WHEELBASE = 3.0        # 축간 거리 (m)
MAX_STEER = math.radians(35.4)   # steer = 1 일 때 앞바퀴 각도
RATE_HZ = 50.0
SETTLE_S = 0.5         # 시작·복귀 뒤 이 시간 동안은 제동만 (시뮬레이터도 복귀 뒤 0.5초 명령을 무시함)


class Control(Node):
    def __init__(self):
        super().__init__('hk_morai_control')
        for name, default in PARAMS.items():           # self.target_speed, self.k_stanley, ... 로 씁니다
            self.declare_parameter(name, default)
            setattr(self, name, float(self.get_parameter(name).value))
        declare_file_params(self)
        self.route = None
        self.odom_time = self.path_time = -math.inf
        for name in ('target_speed', 'lateral_accel', 'planning_decel', 'input_timeout', 'launch_accel', 'steer_rate', 'k_soft'):
            if not math.isfinite(getattr(self, name)) or getattr(self, name) <= 0:
                raise ValueError(f'{name} must be positive and finite')
        if not (math.isfinite(self.brake_on_error) and math.isfinite(self.brake_off_error)
                and 0 <= self.brake_off_error < self.brake_on_error):
            raise ValueError('Need 0 <= brake_off_error < brake_on_error (finite)')
        self.dt = 1.0 / RATE_HZ
        for name in ('kp', 'ki', 'brake_kp', 'brake_gain'):
            if not math.isfinite(getattr(self, name)) or getattr(self, name) < 0:
                raise ValueError(f'{name} must be nonnegative and finite')
        self.odom = None
        self.path = None
        self.active = False
        self.reset_time = None
        self.last_hold_reason = None
        self.status_time = -math.inf
        self.get_logger().info(f'MORAI target_speed={self.target_speed:.2f} m/s '
                               f'({self.target_speed * 3.6:.1f} km/h)')
        self.cmd_pub = self.create_publisher(CtrlCmd, '/hk/control/command', QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(Odometry, '/hk/local/odom', self.on_odom, 10)
        self.create_subscription(Path, '/hk/plan/local_path', self.on_path, 10)
        SimEvents(self, self.on_reset, self.on_end)
        self.create_timer(self.dt, self.tick)

    # ---- 시작·복귀 -----------------------------------------------------------------------------------
    def on_reset(self, checkpoint, reload):
        if reload or self.route is None:
            self.route = load_route(self)
            self.profile = build_profile(self.route.points, load_course(self).data,
                self.target_speed, self.lateral_accel, self.planning_decel,
                self.speed_margin, self.zone_buffer)
        self.route.reset()
        self.odom_time = self.path_time = -math.inf
        self.reset_control()
        self.odom = self.path = None
        self.reset_time = self.now()
        self.active = True

    def on_end(self):
        self.active = False
        self.send(0.0, 1.0, 0.0)

    def reset_control(self):
        """시작·복귀 때 불립니다. 제어기 상태를 처음으로 돌립니다."""
        self.speed_error_sum = 0.0
        self.previous_steer = 0.0
        # 입력 단절 뒤에도 현재 속력에서 재개한다. 0으로 되감으면 달리는 중 전제동을 반복한다.
        self.ramp_target = None
        self.braking = False

    def hold(self, reason, now, detail=''):
        self.reset_control()
        if reason != self.last_hold_reason or now - self.status_time >= 2.0:
            self.get_logger().warning(f'MORAI hold: {reason} {detail}'.rstrip())
            self.status_time = now
        self.last_hold_reason = reason
        self.send(0.0, 1.0, 0.0)

    # ---- 받은 값 -------------------------------------------------------------------------------------
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_odom(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        if not all(math.isfinite(v) for v in (p.x, p.y, q.x, q.y, q.z, q.w, m.twist.twist.linear.x)):
            self.odom = None
            return
        self.odom = m
        self.odom_time = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        self.x, self.y = p.x, p.y
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.speed = m.twist.twist.linear.x

    def on_path(self, m):
        self.path_time = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        self.path = [(ps.pose.position.x, ps.pose.position.y,
                      2 * math.atan2(ps.pose.orientation.z, ps.pose.orientation.w)) for ps in m.poses]

    def nearest(self, x, y):
        return min(self.path, key=lambda p: math.hypot(p[0] - x, p[1] - y))

    # ---- 50 Hz ------------------------------------------------------------------------------------------
    def tick(self):
        if not self.active:
            return
        now = self.now()
        if now - self.reset_time < SETTLE_S:
            self.hold('start/respawn settling', now)
            return
        if self.odom is None or self.path is None:
            self.hold('missing odometry/path', now)
            return
        for name, t in (('odometry', self.odom_time), ('path', self.path_time)):
            if not 0 <= now - t <= self.input_timeout:
                self.hold(f'{name} timestamp invalid/stale', now, f'(age={now-t:.3f}s)')
                return
        if not self.path or not all(math.isfinite(v) for point in self.path for v in point):
            self.hold('empty/invalid path', now)
            return
        command = self.drive()
        if self.last_hold_reason is not None or now - self.status_time >= 2.0:
            self.get_logger().info(
                f'MORAI tracking: speed={self.speed * 3.6:.1f} '
                f'target={self.ramp_target * 3.6:.1f} km/h '
                f'accel={command[0]:.2f} brake={command[1]:.2f}')
            self.status_time = now
        self.last_hold_reason = None
        self.send(*command)

    def drive(self):
        if not self.path or not all(math.isfinite(v) for point in self.path for v in point):
            self.reset_control()
            return 0.0, 1.0, 0.0
        # 1) 앞바퀴 축 위치와, 경로에서 그와 가장 가까운 점·그 점의 경로 방향
        fx = self.x + WHEELBASE * math.cos(self.yaw)
        fy = self.y + WHEELBASE * math.sin(self.yaw)
        if not self.path:                            # 경로 끝을 지남: 곧게 서서 멈춤
            steer, target = 0.0, 0.0
        else:
            px, py, path_yaw = self.nearest(fx, fy)
            # 2) 조향: Stanley = 방향 오차 + atan(K × 옆 오차 / 속력)
            heading_error = angle_diff(path_yaw, self.yaw)
            cross_error = -math.sin(path_yaw) * (px - fx) + math.cos(path_yaw) * (py - fy)   # 경로가 왼쪽이면 +
            wheel_angle = heading_error + math.atan2(self.k_stanley * cross_error, self.k_soft + self.speed)
            steer = clamp(wheel_angle / MAX_STEER, -1.0, 1.0)
            if self.now() - self.reset_time < 1.0:
                self.route.reset()
            index = self.route.nearest(self.x, self.y)
            target = self.profile[index]
            if self.ramp_target is None:
                self.ramp_target = min(target, max(0.0, self.speed))
            self.ramp_target = min(target, self.ramp_target + self.launch_accel * self.dt)
            target = self.ramp_target
            change = self.steer_rate * self.dt / MAX_STEER
            steer = clamp(steer, self.previous_steer - change, self.previous_steer + change)
            self.previous_steer = steer

        # 3) 속력: PI 구동, 초과속력 기반 제동. 작은 오차에서는 타력 주행한다.
        # GPS 지연으로 목표를 살짝 넘을 때마다 구동/제동이 바뀌지 않게 히스테리시스를 둔다.
        error = target - self.speed
        overspeed = -error
        if self.braking:
            self.braking = overspeed > self.brake_off_error
        else:
            self.braking = overspeed > self.brake_on_error
        if self.braking:
            self.speed_error_sum = 0.0
            # heven-ad longitudinal/pid.cpp의 deadband 차감 방식을 적용한다.
            # 히스테리시스를 유지하면서 해제 경계에서 제동이 0으로 부드럽게 줄어든다.
            brake_error = max(0.0, overspeed - self.brake_off_error)
            return 0.0, min(self.brake_kp * brake_error * self.brake_gain, 1.0), steer
        # 포화 중 양의 오차는 쌓지 않고, 과속 중에는 적분을 줄인다.
        candidate = clamp(self.speed_error_sum + error * self.dt, 0.0, 5.0)
        candidate_output = self.kp * error + self.ki * candidate
        # 갱신 후 출력을 검사해야 포화 직전의 마지막 적분 증가도 막을 수 있다.
        if not (candidate_output > 1.0 and candidate > self.speed_error_sum):
            self.speed_error_sum = candidate
        u = self.kp * error + self.ki * self.speed_error_sum
        return (clamp(u, 0.0, 1.0) if error > 0 else 0.0), 0.0, steer

    def send(self, accel, brake, steer):
        c = CtrlCmd()
        c.header.stamp = self.get_clock().now().to_msg()
        c.ctrl_mode = CtrlCmd.CTRL_MODE_AUTO
        c.gear = CtrlCmd.GEAR_DRIVE
        c.long_cmd_type = CtrlCmd.LONG_CMD_THROTTLE
        c.accel, c.brake, c.steering = float(accel), float(brake), float(steer)
        self.cmd_pub.publish(c)


def angle_diff(a, b):
    """a - b 를 -π~π 로."""
    return (a - b + math.pi) % (2 * math.pi) - math.pi


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def main(args=None):
    rclpy.init(args=args)
    node = Control()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
