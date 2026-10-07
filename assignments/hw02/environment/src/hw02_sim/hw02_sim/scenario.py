"""Physical layout and raw map only. No configuration-space or planning code."""
import math
import numpy as np

WIDTH, HEIGHT, RESOLUTION = 8.0, 6.0, 0.025
ROBOT_RADIUS, SAFETY_MARGIN = 0.25, 0.05
PILLAR_RADIUS = 0.4
START, GOAL = (1.0, 3.0), (7.0, 3.0)
LAYOUTS = {
    'wide': [(x, y) for x in (3.0, 5.0) for y in (.45, 2.15, 3.85, 5.55)],
    'mixed': [(3.0, y) for y in (.45, 1.70, 2.85, 4.40, 5.55)] +
             [(5.0, y) for y in (.45, 1.60, 3.15, 4.30, 5.55)],
    'blocked': [(4.0, y) for y in (.45, 1.47, 2.49, 3.51, 4.53, 5.55)],
}


def raw_map(layout):
    """Cell intersects physical obstacle => occupied. Origin (0,0), row-major."""
    w, h = round(WIDTH/RESOLUTION), round(HEIGHT/RESOLUTION)
    xs, ys = np.meshgrid(np.arange(w)*RESOLUTION, np.arange(h)*RESOLUTION)
    occupied = np.zeros((h, w), bool)
    occupied[[0, -1], :] = True
    occupied[:, [0, -1]] = True
    for x, y in LAYOUTS[layout]:
        dx = np.maximum(np.maximum(xs-x, x-(xs+RESOLUTION)), 0)
        dy = np.maximum(np.maximum(ys-y, y-(ys+RESOLUTION)), 0)
        occupied |= dx*dx + dy*dy <= PILLAR_RADIUS**2
    return np.where(occupied, 100, 0).astype(np.int8)


def validate_target(frame, x, y, z=0.0):
    if frame != 'map':
        raise ValueError('target frame must be map')
    if not all(math.isfinite(v) for v in (x, y, z)):
        raise ValueError('target coordinates must be finite')
    if abs(z) > 1e-6:
        raise ValueError('this 2D target must use z=0')
    if not ROBOT_RADIUS <= x <= WIDTH-ROBOT_RADIUS or not ROBOT_RADIUS <= y <= HEIGHT-ROBOT_RADIUS:
        raise ValueError('target is outside usable room bounds')


def steering(pose, target):
    """Direct go-to control only: never computes a route around an obstacle."""
    x, y, yaw = pose
    dx, dy = target[0]-x, target[1]-y
    distance = math.hypot(dx, dy)
    angle = math.atan2(dy, dx)-yaw
    angle = math.atan2(math.sin(angle), math.cos(angle))
    if distance <= 0.02:
        return 0.0, 0.0, distance
    angular = max(-0.8, min(0.8, 2.0*angle))
    # Turn in place before driving, including when a waypoint is behind us.
    linear = min(0.20, 0.8*distance) if abs(angle) < 0.08 else 0.0
    return linear, angular, distance
