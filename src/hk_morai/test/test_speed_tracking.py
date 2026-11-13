"""Closed-loop regression with actual GPS/control methods and a simple vehicle model.

This checks delay and timeout recovery; it is not a MORAI vehicle model.
"""
import ast
from collections import deque
import math
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

from test_control_guards import controller

source = Path(__file__).resolve().parents[1] / 'hk_morai/localization.py'
tree = ast.parse(source.read_text())
cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'Localization')
cls.bases = []
namespace = {'math': math, 'deque': deque,
             'll_to_map': lambda lat, lon, origin: (lat * 1000, lon * 1000)}
definitions = [n for n in tree.body if isinstance(n, (ast.Assign, ast.FunctionDef))]
exec(compile(ast.Module(body=definitions + [cls], type_ignores=[]), str(source), 'exec'), namespace)
Localization = namespace['Localization']


def localization():
    l = object.__new__(Localization)
    # Read the deployed defaults, including speed_window/speed_alpha.
    l.declare_parameter = lambda name, default: setattr(l, name, default)
    l.get_parameter = lambda name: NS(value=getattr(l, name))
    defaults = next(n for n in ast.walk(cls) if isinstance(n, ast.For))
    exec(compile(ast.Module(body=[defaults], type_ignores=[]), str(source), 'exec'), {'self': l})
    l.get_clock = lambda: NS(now=lambda: NS(nanoseconds=0))
    l.reset((0.0, 0.0, 0.0))
    l.course = NS(origin=None)
    return l


def gps(l, t, x):
    sec = int(t)
    l.on_gps(NS(latitude=x / 1000, longitude=0.,
                header=NS(stamp=NS(sec=sec, nanosec=round((t-sec)*1e9)))))


def test_acceleration_estimate_has_limited_delay():
    l = localization()
    acceleration = 1.0
    for i in range(101):
        t = i * 0.05
        gps(l, t + 1000, 0.5 * acceleration * t**2)
    assert 0 <= acceleration * 5.0 - l.speed < 0.25


@pytest.mark.parametrize('target', [3.0, 8.0])
@pytest.mark.parametrize('dropouts', [False, True])
def test_straight_tracking_and_recovery(target, dropouts):
    c, l = controller(), localization()
    c.profile = [target]
    c.path = [(float(i), 0., 0.) for i in range(1000)]
    x = speed = accel = brake = 0.0
    steady = []
    tracking_brakes = 0
    for step in range(6000):
        t = 1000 + step * 0.01
        if step % 5 == 0:
            gps(l, t, x)
        if step % 2 == 0:
            c.now = lambda: t
            c.x, c.speed = x, l.speed
            c.path_time = c.odom_time = t
            stale = dropouts and step > 0 and step % 300 == 0
            if stale:
                c.odom_time = t - c.input_timeout - 0.01
            c.tick()
            accel, brake, _ = c.commands[-1]
            if step > 2000:
                steady.append(speed)
                tracking_brakes += brake > 0 and not stale
        # Simple drag + throttle/brake plant, not MORAI physics.
        speed = max(0., speed + 0.01 * (3 * accel - 6 * brake - 0.12 * speed))
        x += speed * 0.01
    assert min(steady) > target * 0.8
    assert max(steady) < target * 1.15
    assert tracking_brakes == 0
