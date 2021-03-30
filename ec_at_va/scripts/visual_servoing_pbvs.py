#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import image_geometry
import tf2_ros as tf2
import tf2_geometry_msgs
import rospy, sys, math, control_lib, tf
from geometry_msgs.msg import Pose2D, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo


def callback_camera_info(msg):
    global model
    global camera_matrix
    model.fromCameraInfo(msg)
    K = np.array(msg.K).reshape([3, 3])
    f = K[0][0]
    u0 = K[0][2]
    v0 = K[1][2]
    camera_matrix[0] = f
    camera_matrix[1] = u0
    camera_matrix[2] = v0

def callback_img_point(msg):
    global camera_height
    global image_point
    global mask_is_true

    # recovering point
    u = msg.x
    v = msg.y
    base_point = [u, v]
    mask_is_true = msg.theta
    distance = 0

    try:
        # finding distance to the point 
        pixel_rectified = model.rectifyPoint(base_point)
        line = model.projectPixelTo3dRay(pixel_rectified)
        th = math.atan2(line[2],line[1])
        distance = math.tan(th) * camera_height

        image_point.x = u
        image_point.y = v
        image_point.theta = distance

    except:
        pass

def callback_odom(msg):
    global robot_pose
    robot_pose.x = round(msg.pose.pose.position.x, 3)
    robot_pose.y = round(msg.pose.pose.position.y, 3)
    
    q = [msg.pose.pose.orientation.x, 
        msg.pose.pose.orientation.y, 
        msg.pose.pose.orientation.z, 
        msg.pose.pose.orientation.w]

    euler = tf.transformations.euler_from_quaternion(q)
    theta = round(euler[2],3)
    robot_pose.theta = theta

def control_robot():
    global goal
    global image_point
    global robot_pose
    global gains_cart
    global camera_matrix
    global threshold
    global mask_is_true
    

    rospy.init_node('pvbs_control', anonymous=False)
    
    # creating transformation engine
    tfBuffer = tf2.Buffer()
    listener = tf2.TransformListener(tfBuffer)

    # Subscribers
    rospy.Subscriber('camera_info', CameraInfo, callback_camera_info)   # receives the info camera
    rospy.Subscriber('img_point',Pose2D, callback_img_point)   # receives the goal coordinates
    rospy.Subscriber('odom', Odometry, callback_odom)    # receives thr robot odometry

    # Publishers
    cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)  # send control signals

    # control rate
    rate = rospy.Rate(30)

    # main loop
    while not rospy.is_shutdown():
        print("#################### PBVS Control ####################\r")
        # Computing the control signal
        control_signal = Twist()
        
    #try:
        if mask_is_true:
            #breakpoint()
            print("\rDistance to the target: ", image_point.theta)
            camera_point = control_lib.get_img_point(image_point, camera_matrix)
            print("\rCamera Coordinates:\r")
            print(camera_point)
            camera_goal = tfBuffer.transform(camera_point, "odom")
            print("\rGlobal Coordinates:\r")
            print(camera_goal)
            goal.x = camera_goal.point.x
            goal.y = camera_goal.point.y
            control_signal = control_lib.cartesian_control(robot_pose, goal, gains_cart[0], gains_cart[1], threshold)
        else:
            control_signal = Twist()
            control_signal.linear.x = 0.
            control_signal.angular.z = 0.5
    #except:
    #    pass

        cmd_vel.publish(control_signal)
        print("\rControl Signal: ", control_signal.linear.x, control_signal.angular.z)

        rate.sleep()
    

############ MAIN CODE ############
# Readin from launch
K_eu = float(sys.argv[1])   # Control gain for linear velocity
K_ev = float(sys.argv[2])   # Control gain for angular velocity
camera_height = float(sys.argv[3])

# Inner values
robot_pose = Pose2D()
image_point = Pose2D()
goal = Pose2D()
gains_cart = [K_eu, K_ev]
camera_matrix = np.zeros((3,1))
mask_is_true = False
threshold = 0.37

# creating a camera model
model = image_geometry.PinholeCameraModel()

if __name__ == '__main__':
    control_robot()
