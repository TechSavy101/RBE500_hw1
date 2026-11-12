"""
ROS 2 inverse kinematics service for the OpenMANIPULATOR-X
(Si et al. 2013 closed-form method, specialised to this arm: D = 0, a1 = 0).
 
Conventions match /joint_states on the real robot:
  - joint angles in RADIANS, positive joint2/3/4 tilt the arm forward/down
  - all zeros = the "L" pose (upper arm up, forearm + gripper forward)
  - positions in METERS in the base ('world') frame, z = 0 at the bottom of the base
 
Pitch (gripper angle below horizontal) is read from target_pose.orientation.
If that pitch can't be reached within joint limits, the service tries the
nearest pitches that work and says which one it used in `message`.
"""
 
import math
import numpy as np
import rclpy
from rclpy.node import Node
from ik_interfaces.srv import IK
 
 
# ---- Geometry (mm) ----
D1 = 96.326                      # base to joint 2 axis
L2 = math.hypot(128.0, 24.0)     # 130.23, joint 2 -> joint 3 (straight line)
L3 = 124.0                       # joint 3 -> joint 4
L4 = 133.4                       # joint 4 -> end effector (ROBOTIS URDF uses 126.0)
OFF = math.atan2(24.0, 128.0)    # 10.62 deg tilt of the 128/24 link
PHI0 = math.pi / 2 - OFF         # 79.38 deg
 
# Joint limits (rad) -- check these against your open_manipulator_x xacro
LIMITS = [
    (-math.pi, math.pi),                 # joint1
    (-0.57 * math.pi, 0.50 * math.pi),   # joint2
    (-0.30 * math.pi, 0.44 * math.pi),   # joint3
    (-0.57 * math.pi, 0.65 * math.pi),   # joint4
]
 
# Pitches tried when the requested one fails: -90..90 deg in 1 deg steps
PITCH_CANDIDATES = np.radians(np.arange(-90, 91, 1))
 
 
def wrap(a):
    """Wrap an angle to [-pi, pi)."""
    return (a + math.pi) % (2 * math.pi) - math.pi
 
 
def within_limits(q):
    return all(lo <= a <= hi for a, (lo, hi) in zip(q, LIMITS))
 
 
def pitch_from_quaternion(qx, qy, qz, qw):
    """
    Pitch (rotation about y, positive = gripper tilted down) from a quaternion,
    using the ZYX (yaw-pitch-roll) convention: pitch = asin(-R[2,0]).
    For a pure pitch theta: quaternion = (0, sin(theta/2), 0, cos(theta/2)).
    """
    s = 2.0 * (qw * qy - qx * qz)
    return math.asin(max(-1.0, min(1.0, s)))
 
 
def solve_ik(x, y, z, elbow_up=True, pitch=0.0):
    """
    x, y, z : end-effector position in meters
    pitch   : gripper angle below horizontal, RADIANS
              (0 = horizontal, pi/2 = pointing straight down). Equals q2 + q3 + q4.
    elbow_up: True -> elbow above the shoulder-wrist line
 
    Returns (q1, q2, q3, q4) in radians.
    Raises ValueError if unreachable or outside joint limits.
    """
    x, y, z = 1000.0 * x, 1000.0 * y, 1000.0 * z     # m -> mm
 
    # q1 (eq. 9: no lateral offset on this arm)
    r = math.hypot(x, y)
    q1 = 0.0 if r < 1e-9 else math.atan2(y, x)       # r = 0: waist singularity
 
    # wrist (joint 4) position in the arm plane, relative to joint 2
    u = r - L4 * math.cos(pitch)                     # horizontal, forward +
    v = (D1 - z) - L4 * math.sin(pitch)              # vertical, DOWN +
 
    # planar 2R for the 130.23 / 124 links (eqs. 14-19)
    c3 = (u * u + v * v - L2 * L2 - L3 * L3) / (2 * L2 * L3)
    if abs(c3) > 1.0 + 1e-9:
        raise ValueError('Unreachable: target outside the workspace')
    c3 = max(-1.0, min(1.0, c3))
    s3 = math.sqrt(1.0 - c3 * c3) * (1.0 if elbow_up else -1.0)
    th3 = math.atan2(s3, c3)
    th2 = math.atan2(v, u) - math.atan2(L3 * s3, L2 + L3 * c3)
 
    # undo the link offset to get motor angles, then eq. (20)
    q1 = -q1
    q2 = th2 + PHI0
    q3 = th3 - PHI0
    q4 = pitch - q2 - q3
 
    q = tuple(wrap(a) for a in (q1, q2, q3, q4))
    if not within_limits(q):
        raise ValueError('Outside joint range')
    return q
 
 
def solve_ik_any(x, y, z, pitch, search=True):
    """
    Try the requested pitch (elbow up, then elbow down). If neither works and
    search is True, try other pitches, closest to the requested one first.
 
    Returns (q, pitch_used, elbow_up). Raises ValueError if nothing works.
    """
    candidates = [pitch]
    if search:
        candidates += sorted(PITCH_CANDIDATES, key=lambda p: abs(p - pitch))
 
    last_err = None
    for p in candidates:
        for elbow_up in (True, False):
            try:
                return solve_ik(x, y, z, elbow_up, p), p, elbow_up
            except ValueError as e:
                last_err = e
    raise ValueError(str(last_err) if not search
                     else 'No solution within joint limits at any pitch')
 
 
class IKService(Node):
    def __init__(self):
        super().__init__('ik_service')
 
        # ros2 run ... --ros-args -p allow_pitch_search:=false  to require the exact pitch
        self.declare_parameter('allow_pitch_search', True)
 
        self.srv = self.create_service(IK, 'inverse_kinematics', self.ik_callback)
        self.get_logger().info('IK service ready on /inverse_kinematics')
 
    def ik_callback(self, request, response):
        p = request.target_pose.position
        o = request.target_pose.orientation
        pitch = pitch_from_quaternion(o.x, o.y, o.z, o.w)
        search = self.get_parameter('allow_pitch_search').value
 
        self.get_logger().info(
            f'Request: pos=({p.x:.3f}, {p.y:.3f}, {p.z:.3f}) m, '
            f'pitch={math.degrees(pitch):.1f} deg')
 
        try:
            q, pitch_used, elbow_up = solve_ik_any(p.x, p.y, p.z, pitch, search)
            response.joint_positions = [float(j) for j in q]
            response.success = True
            elbow = 'elbow up' if elbow_up else 'elbow down'
            if abs(pitch_used - pitch) < 1e-9:
                response.message = f'IK solved ({elbow})'
            else:
                response.message = (
                    f'Requested pitch {math.degrees(pitch):.1f} deg is outside joint range; '
                    f'solved at pitch {math.degrees(pitch_used):.1f} deg ({elbow})')
        except ValueError as e:
            response.joint_positions = []
            response.success = False
            response.message = str(e)
 
        self.get_logger().info(
            f'Response: {[round(math.degrees(j), 2) for j in response.joint_positions]} deg '
            f'| {response.message}')
        return response
 
 
def main(args=None):
    rclpy.init(args=args)
    node = IKService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
 
 
if __name__ == '__main__':
    main()
 
