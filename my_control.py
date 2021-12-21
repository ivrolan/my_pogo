#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import os

import pygame
from pygame.locals import *
import sys

from std_msgs import msg

LINEAR_INC = 0.1
ROT_INC = 0.2


msg_to_send = Twist()

msg_to_send.linear.x = 0.0
msg_to_send.linear.y = 0.0
msg_to_send.linear.z = 0.0

msg_to_send.angular.x = 0.0
msg_to_send.angular.y = 0.0
msg_to_send.angular.z = 0.0

def reset_vels():
    # stop the motor sending 0 to everything
    global msg_to_send
    msg_to_send.linear.x = 0.0
    msg_to_send.linear.y = 0.0
    msg_to_send.linear.z = 0.0

    msg_to_send.angular.x = 0.0
    msg_to_send.angular.y = 0.0
    msg_to_send.angular.z = 0.0

def inc_left():
    global msg_to_send
    msg_to_send.linear.y += LINEAR_INC

def inc_right():
    global msg_to_send
    msg_to_send.linear.y -= LINEAR_INC

def inc_forward():
    global msg_to_send
    msg_to_send.linear.x += LINEAR_INC

def inc_backward():
    global msg_to_send
    msg_to_send.linear.x -= LINEAR_INC

def inc_rot():
    global msg_to_send
    msg_to_send.angular.z += ROT_INC
def dec_rot():
    global msg_to_send
    msg_to_send.angular.z -= ROT_INC

def main():
    global msg_to_send

    pygame.init()
    display = pygame.display.set_mode((300, 300))

    rospy.init_node("my_control_node")

    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)


    rate = rospy.Rate(5)

    while not rospy.is_shutdown():
        for event in pygame.event.get():
            if event.type == QUIT:
                pygame.quit()
                sys.exit()
            if event.type == pygame.KEYDOWN:
                
                if event.key == pygame.K_DOWN:
                    inc_backward()
                if event.key == pygame.K_UP:
                    inc_forward()
                if event.key == pygame.K_RIGHT:
                    inc_right()
                if event.key == pygame.K_LEFT:
                    inc_left()
                if event.key == pygame.K_k:
                    inc_rot()
                if event.key == pygame.K_l:
                    dec_rot()
                if event.key == pygame.K_SPACE:
                    reset_vels()
            

        pub.publish(msg_to_send)
        
        os.system('clear')
        print(msg_to_send)
        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pygame.quit()
        sys.exit()
