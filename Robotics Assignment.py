import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

class Colour_Finder:
    def __init__(self):
        cv2.namedWindow("Image window", 1)
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
        
        ranges = self.laser.ranges        
        if np.nanmin(ranges) > 0.5:
            self.twistMessage.linear.x = 0.3
            
        elif np.nanmin(ranges) <= 0.5:
            self.twistMessage.linear.x = 0
        
        
               
        
        
        self.cmd_vel_pub.publish(self.twistMessage)
        cv2.imshow("Image window", cv_image)
        
    
rospy.init_node('Colour_Finder', anonymous=True)
Colour_Finder()
rospy.spin()
cv2.destroyAllWindows()
        
    