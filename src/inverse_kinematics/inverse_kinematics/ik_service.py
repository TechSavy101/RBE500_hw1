import math
import rclpy
from rclpy.node import Node
from ik_interfaces.srv import IK


class ik_service(Node):
    def __init__(self):
        super().__init__('ik_service')
        self.srv = self.create_service(IK, 'inverse_kinematics', self.ik_callback)
        self.get_logger().info('IK service ready on /inverse_kinematics')

    def ik_callback(self, request, response):
        # initialize shorthand so don’t have to type request.target_pose.position.x everywhere
        p = request.target_pose.position 
        q = request.target_pose.orientation

        self.get_logger().info(f'Request: pos=({p.x:.3f}, {p.y:.3f}, {p.z:.3f})')

        try:
            joints = self.solve_ik(p.x, p.y, p.z, q)
            response.joint_positions = [float(j) for j in joints]
            response.success = True
            response.message = 'IK solved'
        except ValueError as e:
            response.joint_positions = []
            response.success = False
            response.message = str(e)

        self.get_logger().info(f'Response: {response.joint_positions}')
        return response