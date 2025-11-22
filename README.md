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

  ~$ mkdir -p ~/catkin_ws/src
  ~$ cd ~/catkin_ws/src
  ~$ catkin_init_workspace



