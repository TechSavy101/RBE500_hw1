"""Standard DH calculation using exactly the supplied table; lengths in metres."""
import math

DH = ((0.0, 0.06025, math.pi / 2),
      (0.128, 0.024, 0.0),
      (0.0, 0.124, 0.0),
      (0.0, 0.1334, 0.0))  # (a, d, alpha)


def dh_matrix(theta, a, d, alpha):
    c, s = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return [[c, -s*ca, s*sa, a*c],
            [s, c*ca, -c*sa, a*s],
            [0.0, sa, ca, d],
            [0.0, 0.0, 0.0, 1.0]]


def multiply(a, b):
    return [[sum(a[i][k]*b[k][j] for k in range(4))
             for j in range(4)] for i in range(4)]


def forward_kinematics(positions):
    if len(positions) != 4 or not all(math.isfinite(q) for q in positions):
        raise ValueError('Expected four finite joint angles in radians')
    t = [[float(i == j) for j in range(4)] for i in range(4)]
    for theta, (a, d, alpha) in zip(positions, DH):
        t = multiply(t, dh_matrix(theta, a, d, alpha))
    return t


def quaternion_from_matrix(r):
    """Rotation matrix to normalized quaternion (x, y, z, w)."""
    trace = r[0][0] + r[1][1] + r[2][2]
    if trace > 0:
        s = 2 * math.sqrt(trace + 1)
        w = s / 4
        x, y, z = (r[2][1]-r[1][2])/s, (r[0][2]-r[2][0])/s, (r[1][0]-r[0][1])/s
    elif r[0][0] > r[1][1] and r[0][0] > r[2][2]:
        s = 2 * math.sqrt(1+r[0][0]-r[1][1]-r[2][2])
        x = s / 4
        w, y, z = (r[2][1]-r[1][2])/s, (r[0][1]+r[1][0])/s, (r[0][2]+r[2][0])/s
    elif r[1][1] > r[2][2]:
        s = 2 * math.sqrt(1+r[1][1]-r[0][0]-r[2][2])
        y = s / 4
        w, x, z = (r[0][2]-r[2][0])/s, (r[0][1]+r[1][0])/s, (r[1][2]+r[2][1])/s
    else:
        s = 2 * math.sqrt(1+r[2][2]-r[0][0]-r[1][1])
        z = s / 4
        w, x, y = (r[1][0]-r[0][1])/s, (r[0][2]+r[2][0])/s, (r[1][2]+r[2][1])/s
    norm = math.sqrt(x*x+y*y+z*z+w*w)
    return tuple(v/norm for v in (x, y, z, w))
