import math
import rclpy
from rclpy.node import Node
from ik_interfaces.srv import IK


# def solve_ik(x, y, z, elbow_up=True):
#     """Return [q1, q2, q3, q4] for the target pose. Raise Value error if unreachable"""
#     # Link Lengths & constants
#     l1 = 0.096326 # [m]
#     l2 = 0.13023  # [m]
#     l3 = 0.1240   # [m]
#     l4 = 0.1334   # [m]
#     phi = math.radians(90) # deg input

#     # Parameters
#     r = math.sqrt(x**2 + y**2) # Radial distance
#     rw = r - l4*math.cos(phi) # wrist position
#     zw = z - l4*math.sin(phi) # wrist position
#     L = math.sqrt(l1**2 + l2**2) # equivalent link length
#     alpha = math.atan2(l2, l1) # offset angle
#     D = (rw**2 + zw**2 - L**2 -l3**2) / (2*L*l3) # reachability
#     beta = math.atan2(24, 128) # offset angel

#     # Reachability condition
#     if not -1 <= D <= 1:
#         raise ValueError("Requested point out of robot's reach")
#     delta = math.acos(D) if elbow_up else -math.acos(D) # elbow angle

#     # joint angles
#     q1 = math.atan2(y, x)
#     q2 = math.atan2(zw, rw) + alpha - math.atan2(l3*math.sin(delta), L+l3*math.cos(delta))
#     q3 = (math.pi/2) + alpha - delta
#     q4 = phi - q2 - (math.pi/2) + q3

#     # Joint angle constraints
#     if not -(math.pi/2) < q2 < (math.pi/2):
#         raise ValueError('Outside q2 Joint Range')
#     #if not -beta  < q3 < (math.pi/2) + beta:
#         #raise ValueError('Outside q3 Joint Range')
#     # if not -(math.pi/2) < q4 < (math.pi/2):
#     #     raise ValueError('Outside q4 Joint Range')

#     return [q1, q2, q3, q4]

def solve_ik(x, y, z, elbow_up=True):
    """Return [q1, q2, q3, q4] (robot joint angles, rad). Raise ValueError if unreachable."""
    phi_deg = -90
    d1 = 0.096326   # base to joint 2 [m]
    l1 = 0.128      # link 2 vertical part [m]
    l2 = 0.024      # link 2 offset [m]
    l3 = 0.124      # link 3 [m]
    l4 = 0.1334     # joint 4 to end effector [m]
    phi = math.radians(phi_deg)  # gripper pitch above horizontal (-90 = pointing down)

    r = math.sqrt(x**2 + y**2)
    rw = r - l4*math.cos(phi)
    zw = z - d1 - l4*math.sin(phi)
    L = math.sqrt(l1**2 + l2**2)
    alpha = math.atan2(l2, l1)

    D = (rw**2 + zw**2 - L**2 - l3**2) / (2*L*l3)
    if not -1 <= D <= 1:
        raise ValueError("Requested point out of robot's reach")
    delta = -math.acos(D) if elbow_up else math.acos(D)

    # geometric angles (your equations 8-10)
    th2 = math.atan2(zw, rw) + alpha - math.atan2(l3*math.sin(delta), L + l3*math.cos(delta))
    th3 = math.pi/2 + alpha - delta
    th4 = phi - th2 - math.pi/2 + th3

    # convert to OpenManipulator-X joint angles
    q1 = -math.atan2(y, x)
    q2 = math.pi/2 - th2
    q3 = th3 - math.pi
    q4 = th4

    if not -(math.pi/2) < q2 < (math.pi/2):
        raise ValueError('q2 outside joint range')
    if not -(math.pi/2) + alpha < q3 < (math.pi/2) + alpha:
        raise ValueError('q3 outside joint range')
    if not -(math.pi/2) < q4 < (math.pi/2):
        raise ValueError('q4 outside joint range')

    return [q1, q2, q3, q4]

class ik_service(Node):
    def __init__(self):
        super().__init__('ik_service')
        self.srv = self.create_service(IK, 'inverse_kinematics', self.ik_callback)
        self.get_logger().info('IK service ready on /inverse_kinematics')

    def ik_callback(self, request, response):
        # initialize shorthand so don’t have to type request.target_pose.position.x everywhere
        p = request.target_pose.position 

        self.get_logger().info(f'Request: pos=({p.x:.3f}, {p.y:.3f}, {p.z:.3f})')

        try:
            joints = solve_ik(p.x, p.y, p.z, True)
            response.joint_positions = [float(j) for j in joints]
            response.success = True
            response.message = 'IK solved'
        except ValueError as e:
            response.joint_positions = []
            response.success = False
            response.message = str(e)

        self.get_logger().info(f'Response: {response.joint_positions}')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ik_service()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()