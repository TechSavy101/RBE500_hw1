#!/usr/bin/env python3
"""Publish DH forward kinematics whenever a complete JointState arrives."""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from .kinematics import forward_kinematics, quaternion_from_matrix


class ForwardKinematicsNode(Node):
    def __init__(self):
        super().__init__('openmanipulator_fk')
        self.declare_parameter('joint_state_topic', '/joint_states')
        self.declare_parameter('pose_topic', '/end_effector_pose')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('joint_names', ['joint1', 'joint2', 'joint3', 'joint4'])
        self.joint_names = list(self.get_parameter('joint_names').value)
        if len(self.joint_names) != 4 or len(set(self.joint_names)) != 4:
            raise ValueError('joint_names must contain four distinct names in DH order')
        self.base_frame = self.get_parameter('base_frame').value
        self.publisher = self.create_publisher(
            PoseStamped, self.get_parameter('pose_topic').value, 10)
        self.subscription = self.create_subscription(
            JointState, self.get_parameter('joint_state_topic').value,
            self.joint_state_callback, qos_profile_sensor_data)
        self.get_logger().info('FK ready: joint angles in radians, published position in metres')
        self.warned = False

    def joint_state_callback(self, msg):
        # Map by name: do not assume incoming joint order or include the gripper.
        indices = {name: i for i, name in enumerate(msg.name)}
        if any(name not in indices or indices[name] >= len(msg.position)
               for name in self.joint_names):
            if not self.warned:
                self.get_logger().warning('JointState missing required joint names/positions; skipping')
                self.warned = True
            return
        q = [float(msg.position[indices[name]]) for name in self.joint_names]
        if not all(math.isfinite(value) for value in q):
            return
        self.warned = False
        # Calculation and publication both happen inside this callback.
        t = forward_kinematics(q)
        x, y, z, w = quaternion_from_matrix(t)
        pose = PoseStamped()
        pose.header.stamp = msg.header.stamp
        pose.header.frame_id = self.base_frame
        pose.pose.position.x = t[0][3]
        pose.pose.position.y = t[1][3]
        pose.pose.position.z = t[2][3]
        pose.pose.orientation.x = x
        pose.pose.orientation.y = y
        pose.pose.orientation.z = z
        pose.pose.orientation.w = w
        self.publisher.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ForwardKinematicsNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
