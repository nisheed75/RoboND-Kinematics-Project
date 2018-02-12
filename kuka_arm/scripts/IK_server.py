#!/usr/bin/env python

# Copyright (C) 2017 Udacity Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *

import os 
import sys

os.path.pardir = "kuka_arm/scripts"
sys.path.append(
    os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir)))

from Kinematics import Kinematics

def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        ### Your FK code here
        # Create symbols
	#
	#
	# Create Modified DH parameters
	#
	#
	# Define Modified DH Transformation matrix
	#
	#
	# Create individual transformation matrices
	#
	#
	# Extract rotation matrices from the transformation matrices
	#
	#
        ###
        q1, q2, q3, q4, q5, q6, q7  = symbols('q1:8') # joint angles
    	d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8') # lik offset
    	a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7') # link lenght
    	alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7') # twist angles
    
    	    
    	T0_1, T1_2, T2_3, T3_4, T4_5, T5_6,  T6_G, T0_EE = kinematics.create_individual_tf_matrices(q1, q2, q3, q4, q5, q6, q7, d1, d2, 							   d3, d4, d5, d6, d7, a0, a1, a2, a3, a4, a5, a6, alpha0, alpha1, alpha2,
							   alpha3, alpha4, alpha5, alpha6)

        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

	    # Extract end-effector position and orientation from request
	    # px,py,pz = end-effector position
	    # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

            ### Your IK code here
	    # Compensate for rotation discrepancy between DH parameters and Gazebo
	    #
	    #
	    # Calculate joint angles using Geometric IK method
	    #
	    #
            ###
	    R_ee, WC = kinematics.calculate_wrist_center(roll, pitch, yaw, px, py, pz)
            theta1, theta2, theta3, theta4, theta5, theta6 = kinematics.calculate_thetas(WC, T0_1, T1_2, T2_3, R_ee, q1, q2, q3, q4, q5, q6, q7)
	    #if x >= len(req.poses):
		#theta5 = theta5_fin
		#theta6 = theta6_fin

	    theta1_fin = theta1
            theta2_fin = theta2
            theta3_fin = theta3
            theta4_fin = 0 #theta4
            theta5_fin = 0 #theta5
            theta6_fin = 0 #theta6

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
	    joint_trajectory_point.positions = [theta1_fin, theta2_fin, theta3_fin, theta4_fin, theta5_fin, theta6_fin]
	    joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    global kinematics
    kinematics = Kinematics()	  	
    IK_server()
