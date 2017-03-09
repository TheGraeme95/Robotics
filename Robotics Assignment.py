import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError


class Colour_Finder:
    def __init__(self):
        cv2.namedWindow("Robot Vision", 1)
        cv2.namedWindow("Colours", 1)
        cv2.startWindowThread()
        self.bridge = CvBridge() 
        self.laser_sub = rospy.Subscriber("/turtlebot/scan", LaserScan, self.lasercall)
        self.image_sub = rospy.Subscriber("/turtlebot/camera/rgb/image_raw", Image, self.callback)
        self.cmd_vel_pub = rospy.Publisher("/turtlebot/cmd_vel", Twist, queue_size=10)
        self.twistMessage = Twist()
        
        
        
    def lasercall(self, data):
        self.laser = data      
    
    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            
        except CvBridgeError, e:
            print e
        
        hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        #Colour Boundaries      
        redLower = np.array([0, 90, 50])
        redUpper = np.array([15, 255, 255])
        
        yellowLower = np.array([0, 40, 40])
        yellowUpper = np.array([60, 100, 100])
        
        blueLower = np.array([200, 100, 65])
        blueUpper = np.array([260, 255, 255])
        
        greenLower = np.array([80, 100, 40])
        greenUpper = np.array([150, 255, 255])
        
        masks = cv2.inRange(hsv_img, greenLower, greenUpper)
        output = cv2.bitwise_and(cv_image, cv_image, mask = masks)
        
        
        
        
        
        
        
        
        
        
        ranges = self.laser.ranges  
        centre = ((len(ranges)-1)/2)
        centreView = ranges[centre]         
        rightView = ranges[centre:centre+100]       
        leftView = ranges[centre-100:centre]
        
        
        
        
    
#        if np.nanmin(ranges) > 0.5:
#            self.twistMessage.angular.z = 0
#            self.twistMessage.linear.x = 0.5
#            
#            if np.nanmin(rightView) <= 0.5:
#                self.twistMessage.linear.x = 0
#                self.twistMessage.angular.z = -0.2
#                
#            elif np.nanmin(leftView) <= 0.5:
#                self.twistMessage.linear.x = 0
#                self.twistMessage.angular.z = 0.2
#            
#        elif np.nanmin(ranges) <= 0.5:
#            self.twistMessage.linear.x = 0
#            self.twistMessage.angular.z = 0.4
    
    
                
            
            
            
        
        
               
        
        
        self.cmd_vel_pub.publish(self.twistMessage)
        
        cv2.imshow("Robot Vision", cv_image)
        cv2.imshow("Colours", output)
        
        
    
rospy.init_node('Colour_Finder', anonymous=True)
Colour_Finder()
rospy.spin()
cv2.destroyAllWindows()
        
    