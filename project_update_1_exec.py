#!/usr/bin/env python

'''
We get inspirations of Tower of Hanoi algorithm from the website below.
This is also on the lab manual.
Source: https://www.cut-the-knot.org/recurrence/hanoi.shtml
'''

import os
import argparse
import copy
import time
import rospy
import rospkg
import numpy as np
import yaml
import sys
from math import pi
from lab2_header import *

# 20Hz
SPIN_RATE = 20

# UR3 home location
home = np.radians([120, -90, 90, -90, -90, 0])

# Hanoi tower location 1
#Q11 = [120*pi/180.0, -56*pi/180.0, 124*pi/180.0, -158*pi/180.0, -90*pi/180.0, 0*pi/180.0]
#Q12 = [120*pi/180.0, -64*pi/180.0, 123*pi/180.0, -148*pi/180.0, -90*pi/180.0, 0*pi/180.0]
#Q13 = [120*pi/180.0, -72*pi/180.0, 120*pi/180.0, -137*pi/180.0, -90*pi/180.0, 0*pi/180.0]
Q11 = [178.43*pi/180.0,-53*pi/180.0,117*pi/180.0,-155.64*pi/180.0,-90.3*pi/180.0,91.63*pi/180.0]
Q12 = [178.43*pi/180.0,-59.78*pi/180.0,116.34*pi/180.0,-148.22*pi/180.0,-90.3*pi/180.0,91.62*pi/180.0]
Q13 = [178.43*pi/180.0,-66.6*pi/180.0,114.36*pi/180.0,-139.4*pi/180.0,-90.31*pi/180.0,91.61*pi/180.0]
# Hanoi tower location 2 
Q21 = [190.32*pi/180.0,-49.24*pi/180.0,108.98*pi/180.0,-151.27*pi/180.0,-90.63*pi/180.0,103.51*pi/180.0]
Q22 = [190.31*pi/180.0,-55.92*pi/180.0,108.35*pi/180.0,-143.97*pi/180.0,-90.63*pi/180.0,103.50*pi/180.0]
Q23 = [190.31*pi/180.0,-61.84*pi/180.0,106.55*pi/180.0,-136.25*pi/180.0,-90.63*pi/180.0,103.49*pi/180.0]
# Hanoi tower location 3
Q31 = [199.49*pi/180.0,-45.14*pi/180.0,98.09*pi/180.0,-144.34*pi/180.0,-90.87*pi/180.0,112.69*pi/180.0]
Q32 = [199.49*pi/180.0,-51.12*pi/180.0,97.39*pi/180.0,-137.68*pi/180.0,-90.87*pi/180.0,112.69*pi/180.0]
Q33 = [199.49*pi/180.0,-55.78*pi/180.0,95.80*pi/180.0,-131.41*pi/180.0,-90.87*pi/180.0,112.68*pi/180.0]
S1 = [178.50*pi/180.0,-78.47*pi/180.0,106.47*pi/180.0,-120.06*pi/180.0,-90.12*pi/180.0,91.77*pi/180.0]
S2 =[190.47*pi/180.0,-70.51*pi/180.0,100.44*pi/180.0,-121.88*pi/180.0,-90.53*pi/180.0,103.75*pi/180.0]
S3 =[199.38*pi/180.0,-62.71*pi/180.0,91.35*pi/180.0,-120.48*pi/180.0,-90.82*pi/180.0,112.65*pi/180.0]
thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

digital_in_0 = 0
analog_in_0 = 0

suction_on = True
suction_off = False
current_io_0 = False
current_position_set = False

# UR3 current position, using home position for initialization
current_position = copy.deepcopy(home)

############## Your Code Start Here ##############
"""
TODO: Initialize Q matrix
"""

Q = [ [Q11, Q12, Q13,S1], \
      [Q21, Q22, Q23,S2], \
      [Q31, Q32, Q33,S3] ]
############### Your Code End Here ###############

############## Your Code Start Here ##############

"""
TODO: define a ROS topic callback funtion for getting the state of suction cup
Whenever ur3/gripper_input publishes info this callback function is called.
"""
def topic_callback(msg):
    global digital_in_0    
    digital_in_0 = msg.DIGIN



############### Your Code End Here ###############


"""
Whenever ur3/position publishes info, this callback function is called.
"""
def position_callback(msg):

    global thetas
    global current_position
    global current_position_set

    thetas[0] = msg.position[0]
    thetas[1] = msg.position[1]
    thetas[2] = msg.position[2]
    thetas[3] = msg.position[3]
    thetas[4] = msg.position[4]
    thetas[5] = msg.position[5]

    current_position[0] = thetas[0]
    current_position[1] = thetas[1]
    current_position[2] = thetas[2]
    current_position[3] = thetas[3]
    current_position[4] = thetas[4]
    current_position[5] = thetas[5]

    current_position_set = True


def gripper(pub_cmd, loop_rate, io_0):

    global SPIN_RATE
    global thetas
    global current_io_0
    global current_position

    error = 0
    spin_count = 0
    at_goal = 0

    current_io_0 = io_0

    driver_msg = command()
    driver_msg.destination = current_position
    driver_msg.v = 1.0
    driver_msg.a = 1.0
    driver_msg.io_0 = io_0
    pub_cmd.publish(driver_msg)

    while(at_goal == 0):

        if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
            abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
            abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
            abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
            abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
            abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

            at_goal = 1

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error


def move_arm(pub_cmd, loop_rate, dest, vel, accel):

    global thetas
    global SPIN_RATE

    error = 0
    spin_count = 0
    at_goal = 0

    driver_msg = command()
    driver_msg.destination = dest
    driver_msg.v = vel
    driver_msg.a = accel
    driver_msg.io_0 = current_io_0
    pub_cmd.publish(driver_msg)

    loop_rate.sleep()

    while(at_goal == 0):

        if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
            abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
            abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
            abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
            abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
            abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

            at_goal = 1
            rospy.loginfo("Goal is reached!")

        loop_rate.sleep()

        if(spin_count >  SPIN_RATE*5):

            pub_cmd.publish(driver_msg)
            rospy.loginfo("Just published again driver_msg")
            spin_count = 0

        spin_count = spin_count + 1

    return error


############## Your Code Start Here ##############

def move_block(pub_cmd, loop_rate, start_loc, start_height, \
               end_loc, end_height):
    global Q

    ### Hint: Use the Q array to map out your towers by location and "height".  
    gripper(pub_cmd, loop_rate, suction_off)    
    move_arm(pub_cmd,loop_rate,Q[start_loc][3],4,4)
    move_arm(pub_cmd,loop_rate,Q[start_loc][start_height],4,4)
    gripper(pub_cmd, loop_rate, suction_on) 
    time.sleep(1.0)
    move_arm(pub_cmd,loop_rate,Q[start_loc][3],4,4)
    move_arm(pub_cmd,loop_rate,Q[end_loc][3],4,4)
    move_arm(pub_cmd,loop_rate,Q[end_loc][end_height],4,4)
    gripper(pub_cmd, loop_rate, suction_off)
    time.sleep(1.0)
    move_arm(pub_cmd,loop_rate,Q[end_loc][3],4,4)
    
    error = 0



    return error

############## Your Code Start Here ##############

def draw_line(pub_cmd, loop_rate):
    global Q

    ### Hint: Use the Q array to map out your towers by location and "height".     
    move_arm(pub_cmd,loop_rate,Q[0][0],4,4)
    move_arm(pub_cmd,loop_rate,Q[2][0],4,4)
    error = 0



    return error
############### Your Code End Here ###############


def main():

    global home
    global Q
    global SPIN_RATE

    # Initialize ROS node
    rospy.init_node('lab2node')

    # Initialize publisher for ur3/command with buffer size of 10
    pub_command = rospy.Publisher('ur3/command', command, queue_size=10)

    # Initialize subscriber to ur3/position and callback fuction
    # each time data is published
    sub_position = rospy.Subscriber('ur3/position', position, position_callback)

    ############## Your Code Start Here ##############
    # TODO: define a ROS subscriber for ur3/gripper_input message and corresponding callback function

    sub_topic = rospy.Subscriber('ur3/gripper_input',gripper_input,topic_callback)
    ############### Your Code End Here ###############


    ############## Your Code Start Here ##############
    # TODO: modify the code below so that program can get user input

    input_done = 0
    loop_count = 0

    while(not input_done):
	task_string = raw_input("Enter task will be performed 1:hanoi, 2:drawline <Either 1 2 or 0 to quit> ")
        print("You entered " + task_string + "\n")
	task_sel = int(task_string)
	input_done = 1
	if(task_sel == 1):
	    input_string = raw_input("Enter number of loops <Either 1 2 3 or 0 to quit> ")
            print("You entered " + input_string + "\n")
            start_string = raw_input("Enter starting position <Either 1 2 3> ")
            print("You entered " + start_string + "\n")
            end_string = raw_input("Enter ending position <Either 1 2 3> ")
            print("You entered " + end_string + "\n")
	
	
            start_pos = int(start_string ) - 1
            end_pos = int(end_string) - 1
            inter_pos = 3 - start_pos - end_pos
            if(int(input_string) == 1):
                input_done = 1
                loop_count = 1
            elif (int(input_string) == 2):
                input_done = 1
                loop_count = 2
            elif (int(input_string) == 3):
                input_done = 1
                loop_count = 3
            elif (int(input_string) == 0):
                print("Quitting... ")
                sys.exit()
            else:
                print("Please just enter the character 1 2 3 or 0 to quit \n\n")





    ############### Your Code End Here ###############

    # Check if ROS is ready for operation
    while(rospy.is_shutdown()):
        print("ROS is shutdown!")

    rospy.loginfo("Sending Goals ...")

    loop_rate = rospy.Rate(SPIN_RATE)

    ############## Your Code Start Here ##############
    # TODO: modify the code so that UR3 can move tower accordingly from user input
    if(task_sel == 1):
    	while(loop_count > 0):

        #move_block(pub_command, loop_rate,0,2, \
        #       1, 0)
        #move_block(pub_command, loop_rate,0,1, \
        #       2, 0)
        #move_block(pub_command, loop_rate,1,0, \
        #       2, 1)
        #move_block(pub_command, loop_rate,0,0, \
        #       1, 0)
        #move_block(pub_command, loop_rate,2,1, \
        #       0, 0)
        #move_block(pub_command, loop_rate,2,0, \
        #       1, 1)
        #move_block(pub_command, loop_rate,0,0, \
        #       1, 2)


            move_block(pub_command, loop_rate,start_pos,2, \
               end_pos, 0)
            move_block(pub_command, loop_rate,start_pos,1, \
               inter_pos, 0)
            move_block(pub_command, loop_rate,end_pos,0, \
               inter_pos, 1)
            move_block(pub_command, loop_rate,start_pos,0, \
               end_pos, 0)
            move_block(pub_command, loop_rate,inter_pos,1, \
               start_pos, 0)
            move_block(pub_command, loop_rate,inter_pos,0, \
               end_pos, 1)
            move_block(pub_command, loop_rate,start_pos,0, \
               end_pos, 2)
        #rospy.loginfo("Sending goal 1 ...")
        #move_arm(pub_command, loop_rate, Q[0][0], 4.0, 4.0)

        #gripper(pub_command, loop_rate, suction_on)
        # Delay to make sure suction cup has grasped the block
        #time.sleep(1.0)

        #rospy.loginfo("Sending goal 2 ...")
        #move_arm(pub_command, loop_rate, Q[1][1], 4.0, 4.0)

        #rospy.loginfo("Sending goal 3 ...")
        #move_arm(pub_command, loop_rate, Q[2][0], 4.0, 4.0)

	
            loop_count = loop_count - 1

    if(task_sel == 2):
        draw_line(pub_command,loop_rate)


    gripper(pub_command, loop_rate, suction_off)



    ############### Your Code End Here ###############


if __name__ == '__main__':

    try:
        main()
    # When Ctrl+C is executed, it catches the exception
    except rospy.ROSInterruptException:
        pass
