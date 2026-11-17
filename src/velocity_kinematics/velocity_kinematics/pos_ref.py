"""Send position updates to robot joints based on desired joint velocities"""

import numpy as np
import rclpy
from rclpy.node import Node
from velocity_interfaces.srv import EEToJointVel
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


JOINTS = ['joint1', 'joint2', 'joint3', 'joint4']



class PositionReference(Node):
    def __init__(self):
        super().__init__('pos_ref')
        self.dt = 0.02
        self.q_ref = None # init state from /joint_states, then updated using desired vel and dt
        self.q_dot = np.zeros(4) # joint velocities from pub

        self.create_subscription(JointState, '/joint_states', self.joint_state_cb, 10)
        self.create_subscription(Float64MultiArray, '/joint_vel_ref', self.joint_vel_cb, 10) # look for input joint vel reference
        self.pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)

        self.create_timer(self.dt, self.step) # dt is increment, step is timer callback

        self.get_logger().info('ready for /joint_vel_ref...')


    def joint_state_cb(self, msg):
        if self.q_ref is not None:
            return

        joints = dict(zip(msg.name, msg.position))
        if all(j in joints for j in JOINTS):
            self.q_ref = np.array([joints[j] for j in JOINTS])

    def joint_vel_cb(self, msg):
        if len(msg.data) != 4:
            self.get_logger().warn(f'expected 4 joint velocities, got {len(msg.data)}')
            return
        q_dot_new = np.array(msg.data)
        if not np.array_equal(q_dot_new, self.q_dot): # log only on change
            self.get_logger().info('stopping...' if not q_dot_new.any() else f'moving at q_dot={q_dot_new}') # stops if all 4 q_dot are zero
        self.q_dot = q_dot_new

    def step(self):
        if self.q_ref is None:
            return

        self.q_ref = self.q_ref + self.q_dot * self.dt
        pt = JointTrajectoryPoint(positions=self.q_ref.tolist(), time_from_start=Duration(nanosec=int(self.dt*1e9)))
        self.pub.publish(JointTrajectory(joint_names=JOINTS, points=[pt]))


def main():
    rclpy.init()
    node = PositionReference()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()








