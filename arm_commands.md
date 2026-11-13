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


run ik_service
    ros2 run inverse_kinematics ik_service

perform service call:
    ros2 service call /inverse_kinematics ik_interfaces/srv/IK \
  "{target_pose: {position: {x: 0.2, y: 0.0, z: 0.1}}}"

  run pick and lift
    ros2 run pick_and_lift pl_commander

    note: command for pl_commander: 0.175 0.075 0.01 90 0.1