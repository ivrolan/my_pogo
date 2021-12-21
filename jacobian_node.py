#!/usr/bin/env python3


import rospy as rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
import numpy as np


command = Float32MultiArray()
command.data = [0, 0, 0, 0]

l_pogo = 0.19  # axe roue -> centre robot
w_pogo = 0.18  # roue -> centre entre deux roues
r_pogo = 0.075 # rayon roue

"""pogo = (1/r_pogo) * np.matrix([[-l_pogo-w_pogo, 1, -1],
                               [ l_pogo+w_pogo, 1,  1],
                               [ l_pogo+w_pogo, 1, -1],
                               [-l_pogo-w_pogo, 1,  1]])"""
pogo = np.matrix([[-1, 1, -1],
                  [ 1, 1,  1],
                  [ 1, 1, -1],
                  [-1, 1,  1]])

def twist2motors(data):
    global command, pogo
    x = data.linear.x
    # ! changed: previously was :
    # y = -data.linear.y
    y = data.linear.y
    z = data.angular.z
    max_nb_rot_per_sec = 4
    max_speed = max_nb_rot_per_sec*2*3.14159

    direction = np.matrix([[z],[x],[y]])
    motors = np.matmul(pogo, direction)
    
    for i in range(4):
        command.data[i] = motors.item(i)*max_speed/3.0  # /3 => normaliser la jacobienne entre -1 et 1
    

def talker():
    global command
    rospy.init_node('JacobianNode', anonymous=True)
    pub = rospy.Publisher('jacob_cmd_vel', Float32MultiArray, queue_size=10)
    # NOTE: previously /secured_cmd_vel, now /cmd_vel to make it work with the navigation
    rospy.Subscriber("/cmd_vel", Twist, twist2motors)
    rate = rospy.Rate(10)  # 10Hz

    while not rospy.is_shutdown():
        pub.publish(command)
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
