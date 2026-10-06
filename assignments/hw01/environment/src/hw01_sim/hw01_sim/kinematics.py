"""Provided arm kinematics. Students do not need to implement these equations."""
import math

L1, L2 = 0.30, 0.25
TIP_HEIGHT = 0.14
JOINTS = ['shoulder_joint', 'elbow_joint']
LIMITS = [(-math.pi, math.pi), (0.10, math.pi - 0.10)]
HOME = (-1.5, 2.5)
DEMO = (-1.2, 2.2)
ACTION = '/hw01/move_to_point'
TRAJECTORY_ACTION = '/arm_controller/follow_joint_trajectory'


def forward(q):
    a, b = q
    return (L1 * math.cos(a) + L2 * math.cos(a + b),
            L1 * math.sin(a) + L2 * math.sin(a + b), TIP_HEIGHT)


def inverse(frame, xyz):
    """Return positive-elbow joint angles, or raise ValueError with a useful reason."""
    if frame != 'base_link':
        raise ValueError('target frame must be base_link; transform the point with TF2 first')
    if len(xyz) != 3 or not all(math.isfinite(v) for v in xyz):
        raise ValueError('target coordinates must be finite')
    x, y, z = xyz
    if abs(z - TIP_HEIGHT) > 0.01:
        raise ValueError('this planar arm only reaches z=0.14 m (tolerance 0.01 m)')
    cosine = (x*x + y*y - L1*L1 - L2*L2) / (2 * L1 * L2)
    if not -1 <= cosine <= 1:
        raise ValueError('target is outside the arm reach')
    elbow = math.acos(cosine)
    if abs(math.sin(elbow)) < 0.12:
        raise ValueError('target is too close to a singular configuration')
    shoulder = math.atan2(y, x) - math.atan2(L2 * math.sin(elbow), L1 + L2 * math.cos(elbow))
    shoulder = math.atan2(math.sin(shoulder), math.cos(shoulder))
    q = (shoulder, elbow)
    if any(not low <= value <= high for value, (low, high) in zip(q, LIMITS)):
        raise ValueError('target violates the joint limits')
    return q


def trajectory_samples(start, end, duration=5.0, count=50):
    """Quintic interpolation with zero endpoint velocity and acceleration."""
    for index in range(count + 1):
        u = index / count
        s = 10*u**3 - 15*u**4 + 6*u**5
        ds = (30*u**2 - 60*u**3 + 30*u**4) / duration
        dds = (60*u - 180*u**2 + 120*u**3) / duration**2
        delta = [b-a for a, b in zip(start, end)]
        yield (duration*u, [a+s*d for a, d in zip(start, delta)],
               [ds*d for d in delta], [dds*d for d in delta])
