#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray


# Map joystick axes to joint increments
# Example mapping (Xbox style): left stick vertical -> joint2, left stick horizontal -> joint1
# right stick vertical -> joint3, right stick horizontal -> joint4
# bumper buttons -> joint5 +/-


axis_joint_map = {
1: 1, # left stick vertical -> joint_2 (indexing of published array using 0-based joint order)
0: 0, # left stick horizontal -> joint_1
4: 2, # right stick vertical -> joint_3
3: 3, # right stick horizontal -> joint_4
}


joint5_inc_button_plus = 5 # RB
joint5_inc_button_minus = 4 # LB


increment_scale = 0.02 # radians per joystick full deflection per update


pub = None


# keep track of current target joint positions (5 joints)
targets = [0.0, 0.0, 0.0, 0.0, 0.0]




def joy_cb(msg: Joy):
global targets
# axes
for ax, joint_idx in axis_joint_map.items():
if abs(msg.axes[ax]) > 0.05:
targets[joint_idx] += msg.axes[ax] * increment_scale
# buttons for joint5
if msg.buttons[joint5_inc_button_plus]:
targets[4] += 0.01
if msg.buttons[joint5_inc_button_minus]:
targets[4] -= 0.01


# publish target array
arr = Float32MultiArray()
arr.data = targets
pub.publish(arr)




def main():
rospy.init_node('arm_joystick_teleop')
rospy.Subscriber('joy', Joy, joy_cb)
global pub
pub = rospy.Publisher('arm/command', Float32MultiArray, queue_size=1)
rospy.spin()


if __name__ == '__main__':
main()