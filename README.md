# rover_m20

ROS Noetic software layout for your six-wheeled, corner-steered rover + 5-axis arm, split across:

- Master PC (ROS core + high-level control, teleop, nav, MoveIt)
- Raspberry Pi CM4 (ROS nodes + rosserial bridges to Arduinos)
- Arduino Mega #1 (drive/steer/encoders/motor drivers/PCA9685)
- Arduino Mega #2 (arm steppers + limit switches)

## 0) High-level architecture

### Topics
#### Base
- /cmd_vel (geometry_msgs/Twist) — desired rover motion from teleop/nav
- /rover/wheel_states (sensor_msgs/JointState) — wheel angular pos/vel from encoders
- /rover/steer_states (sensor_msgs/JointState) — corner steering angles
- /rover/odom (nav_msgs/Odometry) — fused odometry
- /tf — odom -> base_link -> wheel_links, steer_links

#### Arm
- /arm/joint_states (sensor_msgs/JointState)
- /arm/joint_cmd (trajectory_msgs/JointTrajectory or custom)
- /arm/homing (std_srvs/Trigger)



### Hardware split
#### Arduino Mega #1 (Drive/Steer)
- Reads 6 quadrature encoders (A/B)
- Drives 6 motors via DRI0002 / Cytron / BTS-type drivers (PWM+DIR)
- Drives 4 steering servos via PCA9685 (I2C)
- Speaks rosserial on /dev/ttyACM0

#### Arduino Mega #2 (Arm)
- Drives 5 NEMA17 stepper axes using A4988 (STEP/DIR/EN)
- Reads 5 homing switches
- Provides joint positions/limits to ROS
- Speaks rosserial on /dev/ttyACM1

#### Raspberry Pi CM4
- Runs rosserial_python bridges
- Runs rover_controller converting /cmd_vel -> wheel RPM + steering angles
- Runs odom integrator


## 1) Catkin workspace + packages

#### On the Pi and PC, same workspace layout:
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
```

#### Create packages:
```bash
cd ~/catkin_ws/src
catkin_create_pkg rover_base roscpp rospy std_msgs sensor_msgs geometry_msgs nav_msgs tf2 tf2_ros
catkin_create_pkg rover_msgs std_msgs sensor_msgs geometry_msgs
catkin_create_pkg rover_bringup rospy rosserial_python
catkin_create_pkg rover_arm roscpp rospy std_msgs sensor_msgs trajectory_msgs std_srvs
```

## 2) ``rover_msgs`` (custom messages)
``rover_msgs/msg/WheelCmd.msg``
```msg
msgfloat32[] wheel_rpm   # length 6
```

``rover_msgs/msg/SteerCmd.msg``
```msg 
float32[] steer_deg   # length 4 (FL, FR, RL, RR)
```

``rover_msgs/msg/ArmCmd.msg``
```msg
float32[] joint_deg   # length 5
float32   speed_scale # 0..1
```

Edit ``rover_msgs/CMakeLists.txt``:
```cmake
find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

add_message_files(
  FILES
  WheelCmd.msg
  SteerCmd.msg
  ArmCmd.msg
)

generate_messages(
  DEPENDENCIES std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)
```
``rover_msgs/package.xml``: add ``message_generation``, ``message_runtime``.

## 3) Base controller on Pi (rover_base)
### Parameters (rover_base/config/base.yaml)
```yaml
wheel_radius: 0.12           # meters (set to your tire radius)
track_width: 0.55            # distance left-right wheel centers
wheelbase: 0.75              # distance front-rear axle centers
max_wheel_rpm: 120.0         # safe RPM limit
max_steer_deg: 35.0          # servo geometry limit
steer_zero_deg: [90,90,90,90] # PCA9685 neutral pulse mapping
rpm_pid:
  kp: 0.8
  ki: 0.1
  kd: 0.0
```

### Node: ``/rover_controller``
#### Converts ``cmd_vel`` to:
- 4 steering angles (Ackermann-ish for corners)
- 6 wheel RPM targets (skid/4WS blend)

``rover_base/src/rover_controller.cpp:``
```cpp

```
