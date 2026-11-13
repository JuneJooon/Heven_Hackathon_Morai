"""HEVEN 2026 MORAI 기본 코드 1/3: 위치 추정 (localization). 참가자가 고칠 곳.

센서를 받아 차의 위치·방향·속력을 구하고, 50 Hz로 /hk/local/odom (nav_msgs/Odometry, frame map)에 냅니다.
같은 값을 TF(map -> base_link)로도 냅니다. planner.py와 control.py가 이 값을 씁니다.
  위치  : GNSS 위경도를 지도 좌표로 변환한 뒤 예측/보정합니다.
  방향  : IMU 메시지의 자세(orientation)를 그대로 씁니다(브리지가 지도 기준으로 맞춰 줌).
  속력  : 바퀴 속도 센서가 없으므로 최근 speed_window초(기본 0.3초)의 GNSS 회귀로 구합니다.

위치 예측/보정, 회귀 기반 속도 추정, GPS 사이 외삽을 적용합니다. 센서가 오래되면 발행을 멈춥니다.
LiDAR·카메라는 lidar_perception.py, camera_perception.py에서 받을 수 있습니다.
"""
import math
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix
from tf2_ros import TransformBroadcaster

from .course import declare_file_params, load_course
from .sim_events import SimEvents
from .utm import ll_to_map

RATE_HZ = 50.0


def stamp(m):
    """메시지의 측정 시각 (s)."""
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


class Localization(Node):
    def __init__(self):
        super().__init__('hk_morai_localization')
        declare_file_params(self)
        for name, default in {'sensor_timeout': 0.5, 'position_alpha': 0.35,
                              'speed_alpha': 0.5, 'speed_window': 0.3}.items():
            self.declare_parameter(name, default)
            setattr(self, name, float(self.get_parameter(name).value))
        if not (all(math.isfinite(v) for v in (self.position_alpha, self.speed_alpha,
                                               self.sensor_timeout, self.speed_window))
                and 0 < self.position_alpha <= 1 and 0 < self.speed_alpha <= 1
                and self.sensor_timeout > 0 and self.speed_window >= 0.25):
            raise ValueError('Invalid localization filter parameters')
        self.course = None
        self.reset(None)

        self.create_subscription(NavSatFix, '/hk/sensors/gps/fix', self.on_gps, qos_profile_sensor_data)
        self.create_subscription(Imu, '/hk/sensors/imu/data', self.on_imu, qos_profile_sensor_data)
        self.pub = self.create_publisher(Odometry, '/hk/local/odom', 10)
        self.tf = TransformBroadcaster(self)
        SimEvents(self, self.on_reset, self.on_end)
        self.create_timer(1.0 / RATE_HZ, self.publish)

    # ---- 시작·복귀 -----------------------------------------------------------------------------------
    def on_reset(self, checkpoint, reload):
        if reload or self.course is None:
            self.course = load_course(self)
        self.reset(self.course.pose(checkpoint))

    def on_end(self):
        self.ready = False

    def reset(self, pose):
        """(x, y, yaw) 자세에서 처음부터. pose가 None이면 자세를 모르는 상태(내보내지 않음)."""
        self.x, self.y, self.yaw = pose if pose else (0.0, 0.0, 0.0)
        self.ready = pose is not None
        self.speed = 0.0
        self.last_gps = self.last_imu = -math.inf
        self.reset_stamp = self.get_clock().now().nanoseconds * 1e-9
        self.gps_history = deque()       # 최근 speed_window초의 (시각, x, y): 속력 계산용

    # ---- 센서마다 불림 ----------------------------------------------------------------------------
    def on_imu(self, m):
        if not self.ready:
            return
        if stamp(m) <= self.reset_stamp:
            return
        q = m.orientation                            # 지도 기준 자세
        if not all(math.isfinite(v) for v in (q.x, q.y, q.z, q.w)) or sum(v*v for v in (q.x, q.y, q.z, q.w)) < 0.5:
            return
        self.last_imu = stamp(m)
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def on_gps(self, m):
        if not self.ready:
            return
        if not (math.isfinite(m.latitude) and math.isfinite(m.longitude) and -90 <= m.latitude <= 90 and -180 <= m.longitude <= 180):
            return
        if stamp(m) <= max(self.reset_stamp, self.last_gps):
            return
        x, y = ll_to_map(m.latitude, m.longitude, self.course.origin)
        t = stamp(m)
        dt = t - self.last_gps
        if self.gps_history and dt < self.sensor_timeout:
            predicted_x = self.x + self.speed * math.cos(self.yaw) * dt
            predicted_y = self.y + self.speed * math.sin(self.yaw) * dt
            if math.hypot(x - predicted_x, y - predicted_y) > 10.0:
                return  # isolated GPS jump; sustained failure expires odometry
            self.x = predicted_x + self.position_alpha * (x - predicted_x)
            self.y = predicted_y + self.position_alpha * (y - predicted_y)
        else:
            self.gps_history.clear()
            self.x, self.y = x, y
            self.speed = 0.0
        self.gps_history.append((t, x, y))
        while t - self.gps_history[0][0] > self.speed_window + 1e-6:
            self.gps_history.popleft()
        # Fit all samples, rather than differentiating two noisy endpoints.
        if len(self.gps_history) >= 4 and t - self.gps_history[0][0] > 0.2:
            n = len(self.gps_history)
            mt = sum(v[0] - t for v in self.gps_history) / n
            mx = sum(v[1] for v in self.gps_history) / n
            my = sum(v[2] for v in self.gps_history) / n
            denom = sum((v[0] - t - mt)**2 for v in self.gps_history)
            vx = sum((v[0]-t-mt)*(v[1]-mx) for v in self.gps_history) / denom
            vy = sum((v[0]-t-mt)*(v[2]-my) for v in self.gps_history) / denom
            measured = max(0.0, vx*math.cos(self.yaw) + vy*math.sin(self.yaw))
            self.speed += self.speed_alpha * (min(measured, 50.0) - self.speed)
        self.last_gps = t

    # ---- 50 Hz로 내보냄 ----------------------------------------------------------------------------
    def publish(self):
        if not self.ready:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if any(not 0 <= now - t <= self.sensor_timeout for t in (self.last_gps, self.last_imu)):
            return
        age = now - self.last_gps
        m = Odometry()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'map'
        m.child_frame_id = 'base_link'
        m.pose.pose.position.x, m.pose.pose.position.y = float(self.x + self.speed * math.cos(self.yaw) * age), float(self.y + self.speed * math.sin(self.yaw) * age)
        m.pose.pose.orientation.z = math.sin(self.yaw / 2)
        m.pose.pose.orientation.w = math.cos(self.yaw / 2)
        m.twist.twist.linear.x = float(self.speed)
        self.pub.publish(m)
        t = TransformStamped()
        t.header = m.header
        t.child_frame_id = 'base_link'
        t.transform.translation.x, t.transform.translation.y = m.pose.pose.position.x, m.pose.pose.position.y
        t.transform.rotation = m.pose.pose.orientation
        self.tf.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = Localization()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
