"""Exercise controller guards without a ROS graph or generated competition messages."""
import ast
import math
from pathlib import Path
from types import SimpleNamespace

import pytest

# Load the actual method bodies; ROS imports and Node construction need the operator PC.
source = Path(__file__).resolve().parents[1] / 'hk_morai/control.py'
tree = ast.parse(source.read_text())
cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'Control')
cls.bases = []
namespace = {'math': math}
definitions = [n for n in tree.body if isinstance(n, (ast.Assign, ast.FunctionDef))]
exec(compile(ast.Module(body=definitions + [cls], type_ignores=[]), str(source), 'exec'), namespace)
Control = namespace['Control']


def controller():
    c = object.__new__(Control)
    for name, default in namespace['PARAMS'].items():
        setattr(c, name, default)
    c.commands = []
    c.send = lambda *args: c.commands.append(args)
    c.now = lambda: 10.0
    c.logs = []
    c.get_logger = lambda: SimpleNamespace(info=c.logs.append, warning=c.logs.append)
    c.last_hold_reason = None
    c.status_time = -math.inf
    c.active = True
    c.dt = 0.02
    c.reset_time = 0
    c.odom = object()
    c.odom_time = c.path_time = 10.0
    c.x = c.y = c.yaw = 0.0
    c.speed = 3.0
    c.path = [(float(i), 0.0, 0.0) for i in range(100)]
    c.route = SimpleNamespace(nearest=lambda x, y: 0, reset=lambda: None)
    c.profile = [3.0]
    c.reset_control()
    return c


def test_empty_path_holds_brake_even_with_integral():
    c = controller()
    c.path = []
    c.speed_error_sum = 5
    assert c.drive() == (0, 1, 0)
    assert c.speed_error_sum == 0


def test_stale_odometry_brakes_with_fresh_path():
    c = controller()
    c.active = True
    c.odom, c.path = object(), [(0, 0, 0)]
    c.reset_time = 0
    c.odom_time, c.path_time = 8.0, 10.0
    c.input_timeout = 0.5
    c.tick()
    assert c.commands == [(0, 1, 0)]


def test_end_brakes_and_disables():
    c = controller()
    c.active = True
    c.on_end()
    assert not c.active
    assert c.commands == [(0, 1, 0)]


def test_small_overspeed_coasts_even_with_integral():
    c = controller()
    c.speed = 3.2
    c.speed_error_sum = 5
    assert c.drive()[:2] == (0, 0)


def test_braking_has_distinct_engage_and_release_thresholds():
    c = controller()
    c.speed = 3.4
    assert c.drive()[1] > 0
    c.speed = 3.2
    assert c.drive()[1] > 0
    c.speed = 3.05
    assert c.drive()[:2] == (0, 0)
    c.speed = 3.2
    assert c.drive()[:2] == (0, 0)


def test_input_recovery_ramps_from_current_speed():
    c = controller()
    c.speed = 2.8
    c.odom_time = 8.0
    c.tick()
    assert c.commands[-1] == (0, 1, 0)
    c.odom_time = 10.0
    c.tick()
    assert c.commands[-1][0] > 0
    assert c.commands[-1][1] == 0
    assert c.ramp_target == pytest.approx(2.82)
    assert any('odometry timestamp' in log for log in c.logs)
    assert 'tracking' in c.logs[-1]


def test_start_still_limits_acceleration_and_curve_still_brakes():
    c = controller()
    c.speed = 0.0
    c.profile = [8.0]
    assert c.drive()[1] == 0
    assert c.ramp_target == pytest.approx(0.02)
    c.speed = c.ramp_target = 8.0
    c.profile = [2.0]
    accel, brake, _ = c.drive()
    assert c.ramp_target == 2.0
    assert accel == 0 and brake > 0


def test_timeout_logs_are_throttled_and_path_timeout_still_brakes():
    c = controller()
    c.path_time = 8.0
    for now in (10.0, 10.02, 10.04):
        c.now = lambda: now
        c.tick()
    assert all(cmd == (0, 1, 0) for cmd in c.commands)
    assert len(c.logs) == 1
    assert 'path timestamp' in c.logs[0]


def test_throttle_tuning_does_not_change_braking():
    c = controller()
    c.speed = 4.0
    first = c.drive()[1]
    c.kp = 2.0
    assert c.drive()[1] == pytest.approx(first)


def test_braking_tapers_to_zero_at_release():
    c = controller()
    c.speed = 3.4
    assert c.drive()[1] > 0
    c.speed = 3.10001
    assert 0 < c.drive()[1] < 0.00001


def test_integral_candidate_cannot_wind_up_into_saturation():
    c = controller()
    c.speed = 1.0
    c.ramp_target = 3.0
    c.kp, c.ki = 0.4995, 1.0
    c.drive()
    assert c.speed_error_sum == 0.0
