import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped


def dh_matrix(a, d, theta, alpha):
    """
    Build a single 4x4 DH transformation matrix (A_i).
    theta, alpha are in DEGREES.
    """
    theta = np.radians(theta)
    alpha = np.radians(alpha)

    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)

    A_i = np.array([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [0,      sa,     ca,    d],
        [0,       0,      0,    1]
    ])

    return A_i


def robot_transformation_matrix(link_transformation_matricies):
    """
    Joins the individual link transformation matricies into the robot
    transformation matrix via post multiplication from the first
    to the last transformation matrix. 
    """

    T = link_transformation_matricies[0]
    for matrix in link_transformation_matricies[1:]:
        T = T @ matrix

    return T

def end_effector_pose(dh_table, degrees=True):
    """
    Computes the end-effector pose using dh_matrix and robot_transformation_matrix.

    dh_table: list of rows, each row = (a_i, d_i, theta_i, alpha_i)
              theta_i, alpha_i in DEGREES (matching dh_matrix's convention)

    degrees:  if True, returns orientation (roll, pitch, yaw) in degrees;
              if False, returns radians

    Returns:
        T           : 4x4 homogeneous transformation matrix (base to end effector)
        position    : (x, y, z) end-effector position
        orientation : (roll, pitch, yaw) end-effector orientation
    """
    # Build each link's transformation matrix using dh_matrix function
    link_matrices = []
    for a, d, theta, alpha in dh_table:
        link_matrices.append(dh_matrix(a, d, theta, alpha))


    # Create robot transformation matrix using your robot_transformation_matrix function
    T = robot_transformation_matrix(link_matrices)

    # Extract position
    position = T[:3, 3]

    # Extract orientation (roll-pitch-yaw, ZYX convention)
    R = T[:3, :3]

    pitch = np.arcsin(-R[2, 0])
    roll = np.arctan2(R[2, 1], R[2, 2])
    yaw = np.arctan2(R[1, 0], R[0, 0])

    orientation = np.array([roll, pitch, yaw])

    if degrees:
        orientation = np.degrees(orientation)

    return T, position, orientation



class forward_kinematic_solver(Node): # Forward Kinematics Solver Subscriber

    # Initializes the class. Tells ros2 that the node exists
    def __init__(self):
        super().__init__('fk_solver_subscriber')

        # Create a subscrition to /joint_states topic
        self.subscription = self.create_subscription(JointState,'/joint_states', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning

        # Create a publisher for the end effector pose
        self.pose_pub = self.create_publisher(PoseStamped, '/end_effector_pose', 10)

        # Print initilization message
        self.get_logger().info(
            'Forward Kinematic Solver subscriber ready. Listening to /joint_states topic.'
        )

    # When a message arrives check if input is valid then solve the forward kinematic problem
    def listener_callback(self, msg):
        # Map each joint name to its position (radians)
        joints = dict(zip(msg.name, msg.position))

        needed = ['joint1', 'joint2', 'joint3', 'joint4']
        if not all(j in joints for j in needed):
            self.get_logger().warn('Missing arm joints in /joint_states', throttle_duration_sec=2.0)
            return

        q1, q2, q3, q4 = np.degrees(joints['joint1'], joints['joint2'], joints['joint3'], joints['joint4'])


        # (a, d, theta, alpha) for each link in the manipulator
        # Lengths in mm
        dh_table = [
            (0,      96.326, q1, 90),   # Link 1
            (130.23, 0,      q2, 0),    # Link 2
            (124.0,  0,      q3, 0),    # Link 3
            (133.4,  0,      q4, 0)     # Link 4
        ]       

        T, position, _ = end_effector_pose(dh_table, degrees=True) # Manipulator can not control orientation of end effector, so only position output
        x, y, z = position / 1000.0  # mm -> m


        pose = PoseStamped()
        pose.header.stamp = msg.header.stamp   # same time as the joint reading
        pose.header.frame_id = 'world'         # frame fk is measured from
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)

        # Orientation: rotation part of T -> quaternion to complete pose
        qx, qy, qz, qw = Rotation.from_matrix(T[:3, :3]).as_quat()
        pose.pose.orientation.x = float(qx)
        pose.pose.orientation.y = float(qy)
        pose.pose.orientation.z = float(qz)
        pose.pose.orientation.w = float(qw)

        self.pose_pub.publish(pose)
        


                  
# Initializes the class and runs the code in an execution loop
def main(args=None):
    rclpy.init(args=args)
    FKS_Solver = forward_kinematic_solver()

    rclpy.spin(FKS_Solver)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    FKS_Solver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
