import csv
import json
import math
from pathlib import Path
import sys
import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from hk_morai.speed_profile import build_profile


def test_brakes_before_school_zone():
    points = [(float(i), 0.0) for i in range(201)]
    course = {'checkpoints': [{'number': 4, 'x': 100, 'y': 0}, {'number': 5, 'x': 150, 'y': 0}],
              'speed_limits': {'default_kmh': 60, 'zones': [{'from_cp': 4, 'to_cp': 5, 'limit_kmh': 30}]}}
    v = build_profile(points, course, cruise=16, decel=1.5)
    assert v[95] <= 30/3.6-0.5
    assert v[80] < 16
    assert v[155] <= 30/3.6-0.5
    for i in range(200):
        assert v[i]**2 <= v[i+1]**2 + 3.0 + 1e-9


def test_circle_curvature():
    points = [(10*math.cos(i/100), 10*math.sin(i/100)) for i in range(200)]
    c = {'checkpoints': [], 'speed_limits': {'default_kmh': 60, 'zones': []}}
    v = build_profile(points, c, cruise=16, lateral=2)
    assert v[100] == pytest.approx(math.sqrt(20), rel=1e-6)


def test_real_course_finite():
    root = Path(__file__).resolve().parents[1] / 'course'
    with (root/'morai_global_path.csv').open() as f:
        pts = [(float(r['x']), float(r['y'])) for r in csv.DictReader(f)]
    c = json.loads((root/'morai_course.json').read_text())
    v = build_profile(pts, c)
    assert len(v) == len(pts)
    assert all(math.isfinite(x) and 0 < x <= 8 for x in v)
    for i in range(len(v)-1):
        assert v[i]**2 <= v[i+1]**2 + 3*math.dist(pts[i], pts[i+1]) + 1e-8


def test_invalid_parameters():
    with pytest.raises(ValueError):
        build_profile([(0, 0)]*3, {}, decel=0)
