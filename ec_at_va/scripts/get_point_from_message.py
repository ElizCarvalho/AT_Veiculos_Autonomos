#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#############################################
#                                           #
# Node to receive image messagens and send  #
# the centroind and base point coordinates  #
# of segmented images.                      #
#                                           #
# Changes:                                  #
#    * Using new libraty image_lib_v2;      #
#    * Updated to Python3 and ROS Noetic    #
#                                           #
# Author: Adalberto Oliveira                #
# Autonomous Vehicle - Infnet	            #
# Version: 1.1                              #
# Date: 13 mar 2021                         #
#                                           #
#############################################


import rospy, time, sys, cv2
import numpy as np
import image_lib as img
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def callback_img(msg):
    global input_img
    bridge = CvBridge()
    input_img = bridge.imgmsg_to_cv2(msg,desired_encoding="bgr8")   


def camera_main():
    """
    This function is called from the main conde and calls 
    all work methods of fucntions into the codde.
    """
    # Global variables
    global mask_h
    global mask_l
    global num_masks
    global input_img
    global show_img

    # Initializing ros node
    rospy.init_node('camera_node', anonymous=False)
    
    # Publisher
    pub_goal_base = rospy.Publisher('goal_base', Pose2D, queue_size=10)
    
    # Subscribers
    rospy.Subscriber('image_raw', Image, callback_img)
 
    # control rate
    rate = rospy.Rate(30)
    pub_img = Image()
    input_img = []

    time.sleep(1)

    # main loop
    while not rospy.is_shutdown():

        # Creating variables
        mask = []
        base = []

        for i in range(num_masks):        
            try:
                # Creating masks         
                mask.append(img.get_mask(input_img,mask_l[i],mask_h[i],im_blur=True))

                # Getting base points
                b_, img_cont = img.get_base(input_img,mask[i])

                # add points
                base.append(b_)
                print('Mask Ready')
            except:
                base = None
                img_cont = input_img
                print('Searching object...')

        # Prepating publishing object
        goal_base = Pose2D()       
        if base is not None:
            goal_base.x = base[0][0]
            goal_base.y = base[0][1]
            goal_base.theta = 1
            
        # Publishin... 
        pub_goal_base.publish(goal_base)

        # showing images
        if show_img:
            cv2.namedWindow('BasePoint')
            cv2.imshow('BasePoint',img_cont)
            cv2.waitKey(1)   
        
        rate.sleep()


################### MAIN CODE  ###################
# Loading initial values of global variables
show_img = int(sys.argv[1])
print('Showing image:',show_img)

# loading params from rosparam
num_masks = rospy.get_param('/num_masks')

# creating masks
mask_h = np.empty([num_masks,3],dtype=np.uint8)
mask_l = np.empty([num_masks,3],dtype=np.uint8)

for i in range(0,num_masks):
    mask_l[i,:] = rospy.get_param('/mask_'+str(i+1)+'/low')
    mask_h[i,:] = rospy.get_param('/mask_'+str(i+1)+'/high')

if __name__ == '__main__':
    camera_main()