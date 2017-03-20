# -*- coding: utf-8 -*-
"""
Created on Thu Feb  9 12:22:06 2017

@author: computing
"""

#!/usr/bin/env python

import rospy
import cv2
import numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError


class image_converter:

    def __init__(self):

        cv2.namedWindow("Image window", 1)
        cv2.startWindowThread()
        self.bridge = CvBridge()       
        self.image_sub = rospy.Subscriber("/turtlebot/camera/rgb/image_raw",
                                          Image, self.callback)
        self.cmd_vel_pub = rospy.Publisher("/turtlebot/cmd_vel", Twist, queue_size=10)

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError, e:
            print e

        bgr_thresh = cv2.inRange(cv_image,
                                 numpy.array((0, 102, 0)),
                                 numpy.array((200, 255, 100)))

        hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        hsv_green = cv2.inRange(hsv_img,
                                 numpy.array((80, 100, 40)),
                                 numpy.array((150, 255, 255)))  #green
                                 
        hsv_blue = cv2.inRange(hsv_img,
                                 numpy.array((200, 100, 65)),
                                 numpy.array((260, 255, 255)))  #blue
                                 
        hsv_thresh = cv2.inRange(hsv_img,
                                 numpy.array((0, 90, 50)),
                                 numpy.array((15, 255, 255)))  #red
                                 
        hsv_thresh3 = cv2.inRange(hsv_img,
                                 numpy.array((0, 40, 40)),
                                 numpy.array((60, 100, 100))) #yellow
                            

        

       
        
        print '===='
        cv2.imshow("Image window", cv_image)
               
        h, w, d = cv_image.shape
        search_top = 3*h/4
        search_bot = 3*h/4 + 20
        hsv_thresh[0:search_top, 0:w] = 0
        hsv_thresh[search_bot:h, 0:w] = 0
        M = cv2.moments(hsv_thresh)
        
        if M['m00'] > 500:
          cx = int(M['m10']/M['m00'])                   
          # BEGIN CONTROL
          err = cx - w/2
          self.twist=Twist()
          self.twist.linear.x = 0.2
          self.twist.angular.z = -float(err) / 100
          #print "HERE:     "

          self.cmd_vel_pub.publish(self.twist)
          # END CONTROL

        cv2.waitKey(3)



rospy.init_node('image_converter', anonymous=True)
image_converter()
rospy.spin()
cv2.destroyAllWindows()