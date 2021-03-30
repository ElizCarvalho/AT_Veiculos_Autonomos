#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#############################################
#                                           #
# Library for robot control with different  #
# approaches and other usefull functions.   #
#                                           #
# Changes:                                  #
#    * Threshold option for cartesian       #
#      ans polar control functions          #
#    * IBVS funtion for image based visual  #
#      servoin control.                     #
#    * Function for recover the coordinate  #
#      of a point in the image with respect #
#      to the camera frame.                 #
#                                           #
# Author: Adalberto Oliveira                #
# Autonomous Vehicle - Infnet	            #
# Version: 1.4                              #
# Date: 21-10-2021                          #
#                                           #
#############################################


import rospy, math, angles
import numpy as np
from geometry_msgs.msg import Twist, PointStamped


def cartesian_control(robot_pose, goal, K_v, K_omega, threshold=0, max_lin=0.5, max_ang=0.5, ):
    """
    This function computes the control signal to guides the 
    robot to the desired goal. It's based on the Cartesian
    Control Algorithm
    """

    # Computing the position error
    error_x = goal.x - robot_pose.x
    error_y = goal.y - robot_pose.y
    error_lin = round(math.sqrt(error_x**2 + error_y**2)-threshold,2)
    v = K_v*error_lin

    # Computing the heading
    heading = math.atan2(error_y,error_x)
    error_th = round(angles.shortest_angular_distance(robot_pose.theta,heading),2)
    
    omega = K_omega*error_th

    # velocity limitation
    v = max_lin*np.sign(v) if abs(v) > max_lin else v
    omega = max_ang*np.sign(omega) if abs(omega) > max_ang else omega

    #print('Error lin:',error_lin,'Erro heading:',error_th)

    u = Twist()
    u.linear.x = v
    u.angular.z = omega

    return u

def get_img_point(image_point, camera_matrix, frame_id='camera_link'):
    """
    This function receives the image point in pixel and returns the point 
    coordinate in the camera frame as a PointStamped object with frame_id.
    """
    
    # Recovering image point
    u = image_point.x
    v = image_point.y

    # getting camera parameters
    Z = image_point.theta
    f = camera_matrix[0]
    u0 = camera_matrix[1]
    v0 = camera_matrix[2]

    # Transforming from pixel to image coordinates
    x_i = (1/f)*(u - u0)
    y_i = (1/f)*(v - v0)

    # Converting from image frame to camera frame
    x_c = Z 
    y_c = -x_i * Z 
    z_c = -y_i * Z

    # Creating camera point
    camera_point = PointStamped()
    camera_point.header.stamp = rospy.Time()
    camera_point.header.frame_id = frame_id
    camera_point.point.x = x_c
    camera_point.point.y = y_c
    camera_point.point.z = z_c

    return camera_point
