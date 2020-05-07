#!/usr/bin/env python
#coding=utf-8
import rospy
import colorsys
import os
import cv2
import numpy as np
import collections
import message_filters
from sensor_msgs.msg import Image
import darknet_ros_msgs.BoundingBoxes
import darknet_ros_msgs.BoundingBox
from testV_msgs import Signal
sig_sub = rospy.Publisher('/testV/isDanger',Signal,queue_size=3)    
def getColorList():
    dict = collections.defaultdict(list)
     
    # red
    lower_red = np.array([156, 43, 46])
    upper_red = np.array([180, 255, 255])
    color_list = []
    color_list.append(lower_red)
    color_list.append(upper_red)
    dict['red'] = color_list

    # red2
    lower_red = np.array([0, 43, 46])
    upper_red = np.array([10, 255, 255])
    color_list = []
    color_list.append(lower_red)
    color_list.append(upper_red)
    dict['red2'] = color_list

    # orange
    lower_orange = np.array([11, 43, 46])
    upper_orange = np.array([25, 255, 255])
    color_list = []
    color_list.append(lower_orange)
    color_list.append(upper_orange)
    dict['orange'] = color_list

    # yellow
    lower_yellow = np.array([26, 43, 46])
    upper_yellow = np.array([34, 255, 255])
    color_list = []
    color_list.append(lower_yellow)
    color_list.append(upper_yellow)
    dict['yellow'] = color_list

    # green
    lower_green = np.array([35, 43, 46])
    upper_green = np.array([77, 255, 255])
    color_list = []
    color_list.append(lower_green)
    color_list.append(upper_green)
    dict['green'] = color_list
    return dict

def get_color(frame):
    print('go in get_color')
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)#
    maxsum = -100
    color = None
    color_dict = getColorList()
    score = 0
    type = 'black'
    for d in color_dict:
        mask = cv2.inRange(hsv, color_dict[d][0], color_dict[d][1])
        binary = cv2.threshold(mask, 127, 255, cv2.THRESH_BINARY)[1]
        binary = cv2.dilate(binary, None, iterations=2)
        img, cnts, hiera = cv2.findContours(binary.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        sum = 0
        for c in cnts:
            sum += cv2.contourArea(c)

        if sum > maxsum:
            maxsum = sum
            color = d
        if sum > score:
            score = sum
            type = d
    return type
class detector():
    def __init__(self):
        rospy.init_node('listener', anonymous=True)
        bbox_sub = message_filters.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes)
        img_sub = message_filters.Subscriber('/rgb/depth/image_raw', Image)
        ts = message_filters.ApproximateTimeSynchronizer([img_sub, bbox_sub], 3, 0.1, allow_headerless=True)
        ts.registerCallback(self.callback) 
    def callback(self,Img,Bbox):
        type = None
        for d in Bbox.bounding_boxes:
            if d.Class=='traffic light' and d.probability>0.9:
                a=d.xmin
                b=d.xmax
                c=d.ymin
                d=d.ymax
                cropImg = img[a:b,c:d]
                type=get_color(cropImg)
                break
        if type!=None:
            if type == 'red' or rype == 'yellow':
                sig = Signal()
                sig.danClass = 'light'
                sig_sub.publish(sig)
if __name__ == '__main__':
    #detector()
    #rospy.spin()
    print('haha')