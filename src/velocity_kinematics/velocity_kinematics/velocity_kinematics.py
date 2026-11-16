"""Converts from joint velocities to end effector velocities and vice versea"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from velocity_interfaces.srv import JointToEEVel, EEToJointVel


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


def get_jacobian(dh_table):
    """Get components of jacobian from Subsequent A_i matricies from DH table (6X4 jacobian)"""
    T = np.eye(4) # Initialize base frame
    Z = [T[:3, 2]]  # Z_0 = [0, 0, 1]
    O = [T[:3, 3]]  # O_0 = [0, 0, 0]

    # Get Z and O for every transformation
    for a, d, theta, alpha in dh_table:
        T = T @ dh_matrix(a, d, theta, alpha)
        Z.append(T[:3, 2])
        O.append(T[:3, 3])


    O_n = O[-1]
    J = np.zeros((6,4))
    for i in range(4):
        J[:3, i] = np.cross(Z[i], O_n - O[i]) # Rows 0 --> 2 column i
        J[3:, i] = Z[i] # Rows 3 --> 5 column i

    J[:, 0] *= -1 # invert column 0 because theta1 = -q1 in dh table
    J[:3, :] /= 1000.0 # linear component mm --> m
    return J




class VelocityKinematics(Node):
    def __init__(self):
        super().__init__('velocity_kinematics')
        self.q = np.zeros(4) # To store joint angles

        # subscribe to joint state topic and execute joint_cb when message arrives (10 msg queue (buffer))
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_cb, 10) 

        # Use created custom service messages, execute function when request is recieved
        self.create_service(JointToEEVel, 'joint_to_ee_vel', self.joint_to_ee_cb)
        self.create_service(EEToJointVel, 'ee_to_joint_vel', self.ee_to_joint_cb)

    def joint_cb(self, msg):
        joints = dict(zip(msg.name, msg.position))
        needed = ['joint1', 'joint2', 'joint3', 'joint4']
        if all(j in joints for j in needed):
            self.q = np.array([joints[j] for j in needed])


    def get_dh_table(self):
        if not self.joint_sub.wait_for_message(timeout=5.0):
            self.get_logger().error('joint_states topic not avaliable')
            return None

        q1, q2, q3, q4 = np.degrees(self.q)
        # (a, d, theta, alpha) for each link in the manipulator
        # Lengths in mm
        offset = np.degrees(np.arctan2(24, 128)) 
        dh_table = [
            (0,       96.326, -q1,                  -90),
            (130.23,  0,       q2 - (90 - offset),    0),
            (124.0,   0,       q3 + (90 - offset),    0),
            (133.4,   0,       q4,                    0),
            ]
        return dh_table

    
    def joint_to_ee_cb(self, request, response):
        dh_table = self.get_dh_table()
        J = get_jacobian(dh_table)
        twist = J @ np.array(request.q_dot)

        x_lin_vel = twist[0]
        y_lin_vel = twist[1]
        z_lin_vel = twist[2]
        x_ang_vel = twist[3]
        y_ang_vel = twist[4]
        z_ang_vel = twist[5]

        response.ee_vel.linear.x = x_lin_vel
        response.ee_vel.linear.y = y_lin_vel
        response.ee_vel.linear.z = z_lin_vel
        response.ee_vel.angular.x = x_ang_vel
        response.ee_vel.angular.y = y_ang_vel
        response.ee_vel.angular.z = z_ang_vel
        return response


    def ee_to_joint_cb(self, request, response):
        dh_table = self.get_dh_table()
        J = get_jacobian(dh_table)
        twist = np.zeros((1,6))

        x_lin_vel = request.ee_vel.linear.x 
        y_lin_vel = request.ee_vel.linear.y 
        z_lin_vel = request.ee_vel.linear.z 
        x_ang_vel = request.ee_vel.angular.x 
        y_ang_vel = request.ee_vel.angular.y
        z_ang_vel = request.ee_vel.angular.z 

        twist[0] = x_lin_vel
        twist[1] = y_lin_vel
        twist[3] = z_lin_vel
        twist[4] = x_ang_vel
        twist[5] = y_ang_vel
        twist[6] = z_ang_vel

        q_dot = np.linalg.ipnv(J) @ twist # J is not square so pseudoinverse
        response.q_dot = q_dot.tolist()
        return response


def main():
    rclpy.init()
    node = VelocityKinematics()
    rclpy.spin(Node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ =='__main__':
    main()



