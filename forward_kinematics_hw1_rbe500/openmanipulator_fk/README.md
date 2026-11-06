# OpenManipulator-X forward kinematics

Uses the supplied DH parameters unchanged. No joint-angle offsets or extra tool transforms are added.

| Joint | theta | a (mm) | d (mm) | alpha (degrees) |
|---|---|---|---|---|
| 1 | q1 | 0 | 60.25 | 90 |
| 2 | q2 | 128 | 24 | 0 |
| 3 | q3 | 0 | 124 | 0 |
| 4 | q4 | 0 | 133.4 | 0 |

## Kinematic model description

The arm is modeled as four revolute joints using standard DH transforms:
A_i = Rz(q_i) Tz(d_i) Tx(a_i) Rx(alpha_i).

Each transform is constructed explicitly as:

```
[cos(q)  -sin(q)cos(alpha)   sin(q)sin(alpha)  a*cos(q)]
[sin(q)   cos(q)cos(alpha)  -cos(q)sin(alpha)  a*sin(q)]
[0        sin(alpha)        cos(alpha)        d       ]
[0        0                 0                1       ]
```

The base-to-end-effector transformation is T = A1 A2 A3 A4.
The last column supplies the position. The upper-left 3x3 block supplies the
orientation, converted explicitly to a normalized quaternion (x,y,z,w).
Only Python's math module is used for numerical kinematics.
All input joint positions are radians; all published translations are metres.
The base_frame parameter names the DH base frame; changing its name does not
apply any coordinate transformation. Set it to the actual matching base frame.

The node subscribes to sensor_msgs/JointState and matches joint1 through joint4
by name, excluding any gripper joints. Each complete, finite message triggers
both calculation and publication inside the callback. Messages missing required
positions are skipped. PoseStamped uses the incoming JointState timestamp.

## Build and run (inside your ROS 2 Linux environment)

Extract this package folder into ~/ros2_ws/src so that
~/ros2_ws/src/openmanipulator_fk/package.xml exists.
Use the ROS distribution already installed on your system:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select openmanipulator_fk
source install/setup.bash
ros2 run openmanipulator_fk fk_node
```

Replace jazzy with humble if that is the installed distribution. Start your
normal robot/simulator bringup separately; this package does not start a driver.

In another terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 topic echo /end_effector_pose
```

Inspect incoming names and topic:

```bash
ros2 topic list -t
ros2 topic echo /joint_states --once
```

If names/topics differ, use parameters:

```bash
ros2 run openmanipulator_fk fk_node --ros-args \
  -p joint_state_topic:=/joint_states \
  -p pose_topic:=/end_effector_pose \
  -p base_frame:=base_link \
  -p "joint_names:=['joint1','joint2','joint3','joint4']"
```

## Standalone ROS callback smoke test (does not move the robot)

Run the node on a separate input topic:

```bash
ros2 run openmanipulator_fk fk_node --ros-args -p joint_state_topic:=/fk_test_joint_states
```

With the pose echo running, send each message separately:

```bash
ros2 topic pub --once /fk_test_joint_states sensor_msgs/msg/JointState "{name: ['joint1', 'joint2', 'joint3', 'joint4'], position: [0.0, 0.0, 0.0, 0.0]}"
ros2 topic pub --once /fk_test_joint_states sensor_msgs/msg/JointState "{name: ['joint1', 'joint2', 'joint3', 'joint4'], position: [0.3, -0.2, 0.4, -0.1]}"
ros2 topic pub --once /fk_test_joint_states sensor_msgs/msg/JointState "{name: ['joint1', 'joint2', 'joint3', 'joint4'], position: [-0.4, 0.25, -0.3, 0.2]}"
```

These test messages use zero timestamps unless a header is provided. They check
callback output only and do not replace the assignment's movement tests.

## Three robot configurations and required evidence

Candidate references in radians (check against your robot's configured limits
and workspace before commanding):

| Configuration | joint1 | joint2 | joint3 | joint4 |
|---|---|---|---|---|
| 1 | 0.0 | 0.0 | 0.0 | 0.0 |
| 2 | 0.3 | -0.2 | 0.4 | -0.1 |
| 3 | -0.4 | 0.25 | -0.3 | 0.2 |

Use your existing joint-position GUI, service, or trajectory controller to send
these references. The command interface depends on your robot bringup; discover
it with:

```bash
ros2 topic list -t
ros2 service list -t
ros2 action list -t
```

Publishing JointState does not command the robot. Do not publish fake states onto
the real /joint_states topic. For the actual assignment, restart the FK node on
the real joint-state topic, command each reference through the robot controller,
wait for the motion to finish, photograph/screenshot the robot, and capture:

```bash
ros2 topic echo /end_effector_pose --once
```

The pose is calculated from measured JointState values, which can differ slightly
from references. Record both when comparing. Include the command values, robot
image, and terminal screenshot for each of the three configurations.

## Validation status

The numerical core was checked against the supplied symbolic transformation,
including nonzero joint values, and quaternion reconstruction was checked across
multiple rotation branches. ROS runtime and physical robot tests must be run on
your ROS installation; no robot screenshots or flight/hardware results are claimed.
