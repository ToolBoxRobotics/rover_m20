#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray


WHEEL_RADIUS = 0.083
WHEEL_BASE = 0.7
TRACK_WIDTH = 0.6


pub = None




def cmdvel_cb(msg: Twist):
v = msg.linear.x
omega = msg.angular.z
# simple ackermann steering angle approximation
if abs(omega) < 1e-6:
steer_angle = 0.0
else:
R = v / omega
steer_angle = math.atan(WHEEL_BASE / R)
wheel_speed = v / WHEEL_RADIUS
data = Float32MultiArray()
# motors: fl, fr, ml, mr, rl, rr
data.data = [wheel_speed]*6 + [math.degrees(steer_angle)]*4
pub.publish(data)




def main():
rospy.init_node('ackermann_mapper')
rospy.Subscriber('cmd_vel', Twist, cmdvel_cb)
global pub
pub = rospy.Publisher('chassis/cmd', Float32MultiArray, queue_size=1)
rospy.spin()




if __name__ == '__main__':
main()