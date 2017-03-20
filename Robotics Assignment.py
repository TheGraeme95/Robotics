import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseActionFeedback
#from geometry_msgs.msg import Pose, Point, Quarternion
from cv_bridge import CvBridge, CvBridgeError


class Colour_Finder:
    def __init__(self):
        cv2.namedWindow("Robot Vision", 1)
        cv2.namedWindow("Colours", 1)
        cv2.startWindowThread()
        self.bridge = CvBridge() 
        self.laser_sub = rospy.Subscriber("/turtlebot/scan", LaserScan, self.lasercall)
        self.image_sub = rospy.Subscriber("/turtlebot/camera/rgb/image_raw", Image, self.callback)
        self.position_sub = rospy.Subscriber("/turtlebot/amcl_pose", PoseWithCovarianceStamped, self.pose_callback)
        self.goal_status = rospy.Subscriber("/turtlebot/move_base/feedback", MoveBaseActionFeedback, self.statusCall)     
        self.cmd_vel_pub = rospy.Publisher("/turtlebot/cmd_vel", Twist, queue_size=10)
        self.goal_pub = rospy.Publisher("/turtlebot/move_base_simple/goal", PoseStamped)        
        self.twistMessage = Twist()
        self.laser = []
        self.foundRed = False
        self.foundYellow = False
        self.foundBlue = False
        self.foundGreen = False
        self.position1 = [1.25, -4.35,[0,0,0.1,0.46]]
        self.goal1 = False
        self.goal2 = False
        self.goal3 = False
        self.goal4 = False
        self.goal5 = False
        self.sentgoal1 = False
        self.isMoving = False
        self.hasGoal = False        
        
        
        self.twistMessage.linear.x = 0
        self.twistMessage.angular.x = 0
        
    def statusCall(self, data):
        if data.status.status == 1:             
            self.hasGoal = True
        else: 
            self.hasGoal = False
        
        
    def lasercall(self, data):        
        self.laser = data

    def pose_callback(self, data):        
        self.xPosition = data.pose.pose.position.x
        self.yPosition = data.pose.pose.position.y
        
    
    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            
        except CvBridgeError, e:
            print e
            
        h, w, d = cv_image.shape
        self.imageHeight = h
        self.imageWidth = w
            
        
        #Colour Slicing
        hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        #RED    
        redLower = np.array([0, 90, 50])
        redUpper = np.array([8, 255, 255])
        redMask = cv2.inRange(hsv_img, redLower, redUpper)
        Mred = cv2.moments(redMask)
        #YELLOW
        yellowLower = np.array([25, 90, 20])
        yellowUpper = np.array([45, 255, 255])
        yellowMask = cv2.inRange(hsv_img, yellowLower, yellowUpper)
        Myellow = cv2.moments(yellowMask)
        #BLUE
        blueLower = np.array([115, 70, 60])
        blueUpper = np.array([125, 255, 255])
        blueMask = cv2.inRange(hsv_img, blueLower, blueUpper)
        Mblue = cv2.moments(blueMask)
        #GREEN
        greenLower = np.array([60, 100, 40])
        greenUpper = np.array([80, 255, 255])
        greenMask = cv2.inRange(hsv_img, greenLower, greenUpper)
        Mgreen = cv2.moments(greenMask)
        
        #Image Properties
        h, w, d = cv_image.shape
        self.imageHeight = h
        self.imageWidth = w
        
        #Laserscan ranges
        ranges = self.laser.ranges       
        centre = ((len(ranges)-1)/2)           
        self.middleSlit = ranges[centre]
        
        
        if self.hasGoal == 0:
            self.chosenGoal1 = self.moveToPoint(self.position1[0], self.position1[1], self.position1[2])            
            print("Goal Sent!")
            self.sentgoal1 = True
            if self.sentgoal1 == True and self.hasGoal == True:
                self.isMoving = True
                self.sentgoal1 = True
                print("Goal Acknowledged")
            
        
        
        
        
        
        
        
        
        
#        if Mred['m00'] > 100 and self.foundRed == False:
#            if np.nanmin(self.middleSlit > 0.7):            
#                self.trackObject(Mred)  
#            else:
#                self.twistMessage.linear.x = 0
#                self.twistMessage.angular.z = 0
#                print('Found Red!')
#                self.foundRed = True
#                
#        if Myellow['m00'] > 100 and self.foundYellow == False:
#            if np.nanmin(self.middleSlit > 0.7):            
#                self.trackObject(Myellow)  
#            else:
#                self.twistMessage.linear.x = 0
#                self.twistMessage.angular.z = 0
#                print('Found Yellow!')
#                self.foundYellow = True
#                
#        if Mblue['m00'] > 100 and self.foundBlue == False:
#            if np.nanmin(self.middleSlit > 0.7):            
#                self.trackObject(Mblue)  
#            else:
#                self.twistMessage.linear.x = 0
#                self.twistMessage.angular.z = 0
#                print('Found Blue!')
#                self.foundBlue = True
#                
#        if Mgreen['m00'] > 100 and self.foundGreen == False:
#            if np.nanmin(self.middleSlit > 0.7):            
#                self.trackObject(Mgreen)  
#            else:
#                self.twistMessage.linear.x = 0
#                self.twistMessage.angular.z = 0
#                print('Found Green!')
#                self.foundGreen = True
                
        
        
        
        
        
        
        
        
        
        #self.goal_pub.publish(self.chosenGoal1)
        #self.cmd_vel_pub.publish(self.twistMessage)
        masks = redMask + yellowMask + blueMask + greenMask
        output = cv2.bitwise_and(cv_image, cv_image, mask = masks)        
        cv2.imshow("Robot Vision", cv_image)
        cv2.imshow("Colours", output)
        
        
   # def detectColour(self):
        
        
    def moveToPoint(self, x, y, orien):
        Goal = PoseStamped()
        Goal.header.frame_id = "/map"
        Goal.header.stamp = rospy.Time.now()
        Goal.pose.position.x = x
        Goal.pose.position.y = y
        Goal.pose.orientation.x = orien[0]
        Goal.pose.orientation.y = orien[1]
        Goal.pose.orientation.z = orien[2]
        Goal.pose.orientation.w = orien[3]
        self.goal_pub.publish(Goal)
        
        return Goal
        

        
    def trackObject(self, moments):
        cx = int(moments['m10']/moments['m00'])
        err = cx - self.imageWidth/2            
        self.twistMessage.linear.x = 1
        self.twistMessage.angular.z = -float(err) / 100           
        
        
        
        
        
    
    
                
            
            
            
        
        
               
        
        
        
        
        
    
rospy.init_node('Colour_Finder', anonymous=True)
Colour_Finder()
rospy.spin()
cv2.destroyAllWindows()
        
    