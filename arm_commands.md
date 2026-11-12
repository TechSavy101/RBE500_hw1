Commands to use open manipulator x (remember to source install first)

Bringup command (initializes arm):
    ros2 launch open_manipulator_bringup open_manipulator_x.launch.py

move arm to specified joint angles (radians):
    ros2 topic pub --once /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
    "{joint_names: [joint1, joint2, joint3, joint4], points: [{positions: [1.0, 0.0, 0.0, 0.0], time_from_start: {sec: 2}}]}"

Run forward_kinematics subscriber and publisher:
    ros2 run forward_kinematics fk_subpub

Listen to end_effector_pose topic published to by fk_subpub
    ros2 topic echo /end_effector_pose
