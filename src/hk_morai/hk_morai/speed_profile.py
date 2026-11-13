"""ROS-independent curvature and legal-speed envelope, inspired by heven-ad route_speed_profile."""
import math


def build_profile(points, course, cruise=8.0, lateral=1.8, decel=1.5, margin=0.5, zone_buffer=5.0):
    if len(points) < 3 or not all(math.isfinite(v) and v > 0 for v in (cruise, lateral, decel)):
        raise ValueError('Need >=3 route points and positive finite speed/acceleration limits')
    if not all(math.isfinite(v) and v >= 0 for v in (margin, zone_buffer)):
        raise ValueError('Margins must be finite and nonnegative')
    if not all(math.isfinite(v) for point in points for v in point):
        raise ValueError('Invalid route coordinate')
    s = [0.0]
    for a, b in zip(points, points[1:]):
        s.append(s[-1] + math.dist(a, b))
    cp = {}
    start = 0
    for c in course['checkpoints']:
        start = min(range(start, len(points)), key=lambda i: math.dist(points[i], (c['x'], c['y'])))
        cp[c['number']] = s[start]
    limits = course['speed_limits']
    speeds = []
    for i, b in enumerate(points):
        legal = limits['default_kmh']
        for zone in limits['zones']:
            # Extend restricted zones; shrink unrestricted ones around checkpoints.
            pad = zone_buffer if zone['limit_kmh'] is not None else -zone_buffer
            if cp[zone['from_cp']] - pad <= s[i] <= cp[zone['to_cp']] + pad:
                legal = zone['limit_kmh']
        cap = cruise if legal is None else min(cruise, max(0.0, legal / 3.6 - margin))
        a, c = points[max(0, i-5)], points[min(len(points)-1, i+5)]
        ab, bc, ac = math.dist(a, b), math.dist(b, c), math.dist(a, c)
        denom = ab * bc * ac
        curvature = 0.0 if denom < 1e-9 else 2 * abs((b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])) / denom
        speeds.append(min(cap, math.sqrt(lateral / max(curvature, 1e-9))))
    # Backward braking envelope: slow down BEFORE curves and restricted zones.
    # Do not stop at the route endpoint: the official finish line must be crossed;
    # END (or the existing past_end guard) stops the vehicle.
    for i in range(len(points)-2, -1, -1):
        speeds[i] = min(speeds[i], math.sqrt(speeds[i+1]**2 + 2*decel*(s[i+1]-s[i])))
    return speeds
