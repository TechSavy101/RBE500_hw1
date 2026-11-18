"""Command a short straight-line motion in the positive y direction."""

from math import isfinite

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from velocity_interfaces.srv import EEToJointVel


class VelocityCommander(Node):
    def __init__(self):
        super().__init__('vel_commander')
        self.speed = 0.01  # m/s
        self.duration = 2.0  # seconds
        self.start = None
        self.position = None
        self.pending = None

        # Read the tip position from forward kinematics and send joint velocities to pos_ref.
        self.create_subscription(PoseStamped, '/end_effector_pose', self.pose_cb, 10)
        self.client = self.create_client(EEToJointVel, '/ee_to_joint_vel')
        self.pub = self.create_publisher(Float64MultiArray, '/joint_vel_ref', 10)
        # Recompute the command every 20 ms as the arm moves.
        self.timer = self.create_timer(0.02, self.step)

    def pose_cb(self, msg):
        self.position = msg.pose.position

    def step(self):
        now = self.get_clock().now()
        # Start the two-second motion once a pose, the service, and pos_ref are available.
        if self.start is None:
            if self.position is None or not self.client.service_is_ready():
                return
            if self.pub.get_subscription_count() == 0:
                return
            self.start = now

        if (now - self.start).nanoseconds >= self.duration * 1e9:
            # Keep sending zero because pos_ref holds the last velocity it received.
            self.pub.publish(Float64MultiArray(data=[0.0] * 4))
            return

        # Allow only one outstanding service request so calls do not pile up.
        if self.pending is not None and not self.pending.done():
            return

        x, y = self.position.x, self.position.y
        radius_sq = x * x + y * y
        # The yaw-rate formula divides by radius squared, so hold near the base axis.
        if radius_sq < 0.01 ** 2:
            self.pub.publish(Float64MultiArray(data=[0.0] * 4))
            return

        request = EEToJointVel.Request()
        # Command translation along +y. The x and z components default to zero.
        request.ee_vel.linear.y = self.speed
        # Base yaw follows atan2(y, x). With x fixed, its rate is x * dy/dt / (x² + y²).
        request.ee_vel.angular.z = x * self.speed / radius_sq
        # Ask the Jacobian service for joint velocities without blocking the timer.
        self.pending = self.client.call_async(request)
        self.pending.add_done_callback(self.joint_vel_cb)

    def joint_vel_cb(self, future):
        elapsed = (self.get_clock().now() - self.start).nanoseconds / 1e9
        # Discard a late reply so it cannot restart motion after the timed stop.
        if elapsed < self.duration:
            response = future.result()
            q_dot = response.q_dot if response is not None else []
            # Hold if the reply lacks four finite velocities within 0.5 rad/s.
            if len(q_dot) != 4 or any(not isfinite(q) or abs(q) > 0.5 for q in q_dot):
                q_dot = [0.0] * 4
            self.pub.publish(Float64MultiArray(data=q_dot))


def main():
    rclpy.init()
    node = VelocityCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
