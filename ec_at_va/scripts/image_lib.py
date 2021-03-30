#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#############################################
#                                           #
# Library for image processing using        #
# OpensCV.                                  #
#                                           #
# Changes:                                  #
#    * Updated to Python3 and OPencv 4.2.0  #
#    * Get the great area in the image to   #
#      compute centroid and base point.     #
#                                           #
# Author: Adalberto Oliveira                #
# Autonomous Vehicle - Infnet	            #
# Version: 1.22                             #
# Date: 21 mar 2021                         #
#                                           #
#############################################


import cv2
import numpy as np
from cv_bridge import CvBridge


def get_mask(image, low, high, im_blur=False):
    """
    Receives an image and lower and upper values for color segmentation
    image: a RGB type image
    low, high: numpy array
    im_blur: applying Gaussian blur
    """
    # converting from RGB to HSV
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # creating mask
    mask = cv2.inRange(hsv,low, high)
    
    # applying Gaussian smoothing    
    if im_blur:
        mask = cv2.GaussianBlur(mask,(5,5),10)

    # applying morphological operations
    kernel = np.ones((3),np.uint8)
    mask_out = cv2.dilate(mask,kernel,iterations=2)
    mask_out = cv2.erode(mask_out,kernel,iterations=2)
    mask_out = cv2.morphologyEx(mask_out,cv2.MORPH_CLOSE, kernel)
    mask_out = cv2.morphologyEx(mask_out,cv2.MORPH_CLOSE, kernel)
    mask_out = cv2.morphologyEx(mask_out,cv2.MORPH_OPEN, kernel)

    return mask_out

def get_base(cv_img, mask, put_text=False):
    """
    Finds image base and bouding box
    cv_img: input image RGB
    mask: binary image mask
    """
    cv_output = cv_img.copy()
    
    # fiding mask contours
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    # contours parameters
    area = 0
    cont_out = []

    # fiding great area in the mask
    for c in contours:
        M = cv2.moments(c)        
        if (M["m00"] > area):
            area = M["m00"]
            cont_out = [c]
    
    contours = cont_out

    contours_poly = [None]*len(contours)
    boundRect = [None]*len(contours)

    for i, c in enumerate(contours):
        contours_poly[i] = cv2.approxPolyDP(c, 3, True)
        boundRect[i] = cv2.boundingRect(contours_poly[i])

    for i in range(len(contours)):
        # contour parameters
        x = boundRect[i][0]
        y = boundRect[i][1]
        w = boundRect[i][2]
        h = boundRect[i][3]

    high_corner_x = x
    high_corner_y = y
    low_corner_x = x+w
    low_corner_y = y+h

    # getting the center point of the base of the rectangle
    base_x = low_corner_x - (int(w/2))
    base_y = low_corner_y 
    base = [base_x,base_y]

    # drawning features
    cv2.rectangle(cv_output,
                (high_corner_x,high_corner_y),
                (low_corner_x,low_corner_y),
                (0,255,0),2)

    # drowning image output elements
    cv2.circle(cv_output, (base_x, base_y), 4, (255,0,0),-1)

    if put_text:
        font = cv2.FONT_HERSHEY_SIMPLEX
        bottomLeftCornerOfText = (base_x,base_y+10)
        fontScale = 0.5
        fontColor = (255,255,255)
        lineType = 1
        text = '('+str(base_x)+', '+str(base_y)+')'

        cv2.putText(cv_output,text, 
            bottomLeftCornerOfText, 
            font, 
            fontScale,
            fontColor,
            lineType)

    return base, cv_output