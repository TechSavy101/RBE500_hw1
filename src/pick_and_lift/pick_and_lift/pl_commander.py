import numpy as np
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from inverse_kinematics.ik_service import solve_ik_any
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand
from control_msgs.action import FollowJointTrajectory

GRIPPER_OPEN = -0.01     # joint position when fully open 
GRIPPER_CLOSED = 0.019   # joint position when fully closed



class pl_commander(Node):

    # Initializes the class. Tells ros2 that the node exists
    def __init__(self):
        super().__init__('pl_commander')

        self.declare_parameter('move_time', 2.0)
        self.arm_client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.gripper_client = ActionClient(self, GripperCommand, '/gripper_controller/gripper_cmd')


    def move_arm(self, x, y, z, pitch):
        """Move the end effector to (x, y, z). Returns True when the arm arrives."""

        if not self.arm_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Arm action server not available')
            return False

        try:
            q, pitch_used, elbow_up = solve_ik_any(x, y, z, pitch)
        except ValueError as e:
            self.get_logger().warning(f'No IK solution: {e}')
            return False

        if abs(pitch_used - pitch) > 1e-6 or not elbow_up:
            self.get_logger().warning(
                f'Requested pitch unreachable; using pitch {np.rad2deg(pitch_used):.1f} deg, '
                f'elbow {"up" if elbow_up else "down"}')

        t = self.get_parameter('move_time').value
        trajectory = JointTrajectory()
        trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
        pt = JointTrajectoryPoint()
        pt.positions = [float(a) for a in q]
        pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
        trajectory.points = [pt]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory 

        send_future = self.arm_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Arm goal rejected')
            return False

        self.get_logger().info(f'Moving to {pt.positions}')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=t + 5.0)
        if not result_future.done():
            self.get_logger().error('Arm did not finish in time')
            return False

        result = result_future.result().result
        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info('Arm reached target')
            return True

        self.get_logger().error(f'Arm move failed ({result.error_code}): {result.error_string}')
        return False


    def open_grab(self, max_effort=1.0):
        """
        Open the grabber
        max_effort is max force gripper may apply (required field but not used)

        Returns True when open false otherwise
        """
        position = -0.01

        if not self.gripper_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Gripper action server not available')
            return False

        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = float(max_effort)

        send_future = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Gripper goal rejected')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        if result.reached_goal:
            self.get_logger().info(f'Gripper Open (fingers at {result.position:.3f})')
            return True

        self.get_logger().warning(f'Gripper not fully open (fingers at {result.position:.3f})')
        return False


    def grab_obj(self, max_effort=1.0):
        """
        Close grabber on an object
        max_effort is max force gripper may apply (required field but not used)

        returns True if the object was grasped, False otherwise.
        """

        position = GRIPPER_CLOSED

        if not self.gripper_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Gripper action server not available')
            return False

        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = float(max_effort)

        send_future = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Gripper goal rejected')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        if result.stalled and not result.reached_goal:
            self.get_logger().info(f'Grasped object (fingers at {result.position:.4f})')
            return True

        self.get_logger().warning('Gripper closed fully, no object grasped')
        return False



def main():
    rclpy.init()
    node = pl_commander()
    try:
        while rclpy.ok():
            line = input("Enter X Y Z pitch, z_off [m m m deg m] or q to quit: ").strip()
            if line.lower() == 'q':
                break
            try:
                x, y, z, pitch, z_off= map(float, line.split())

            except ValueError:
                print("Please enter five numbers seperated by spaces, e.g. 0.0 2.0 1.5 90 0.1")
                continue

            pitch_rad = np.deg2rad(pitch)
            steps_ok = (node.open_grab()                        # open gripper
                and node.move_arm(x, y, z + z_off, pitch_rad)   # approach above object
                and node.move_arm(x, y, z, pitch_rad)           # descend
                and node.grab_obj()                             # grab object
                and node.move_arm(x, y, z + z_off, pitch_rad))  # lift

            if steps_ok:
                node.get_logger().info('Pick and lift complete')
            else:
                node.get_logger().warning('Pick and lift stopped: a step failed (see messages above)')

    except (KeyboardInterrupt, EOFError):
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
