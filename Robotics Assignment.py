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
        rospy.sleep(2)
        cv2.namedWindow("Robot Vision", 1)
        cv2.namedWindow("Colours", 1)
        cv2.startWindowThread()
        self.bridge = CvBridge() 
        self.laser_sub = rospy.Subscriber("/turtlebot/scan", LaserScan, self.lasercall)
        self.image_sub = rospy.Subscriber("/turtlebot/camera/rgb/image_raw", Image, self.callback)
        self.position_sub = rospy.Subscriber("/turtlebot/amcl_pose", PoseWithCovarianceStamped, self.pose_callback)
        #self.goal_status = rospy.Subscriber("/turtlebot/move_base/feedback", MoveBaseActionFeedback, self.statusCall)     
        self.cmd_vel_pub = rospy.Publisher("/turtlebot/cmd_vel", Twist, queue_size=10)
        self.goal_pub = rospy.Publisher("/turtlebot/move_base_simple/goal", PoseStamped)        
        self.twistMessage = Twist()
        self.laser = []
        self.foundRed = False
        self.foundYellow = False
        self.foundBlue = False
        self.foundGreen = False
        self.position1 = [-0.1, -4.4,[0,0,0,1]]
        self.position2 = [-4.20, 1.70, [0,0,0.29,0.956]]
        self.position3 = [-3.83, 0.96, [0,0,-0.337,0.94]]
        self.position4 = [-1.19, 3.66, [0,0,0.079,0.996]]
        self.position5 = [-0.54, 2.02, [0,0,0.0166,0.999]]
        self.position6 = [2.67, 2.18, [0,0,-0.688,0.725]]
        self.currentGoal = self.position1
        self.goal1Achieved = False
        self.goal2Achieved = False
        self.goal3Achieved = False
        self.goal4Achieved = False
        self.goal5Achieved = False
        self.sentgoal1 = False
        self.isMoving = False
        self.hasGoal = False 
        self.foundColour = False        
        self.twistMessage.linear.x = 0
        self.twistMessage.angular.x = 0
        self.colourChecked = False
        rospy.sleep(2)
        
#    def statusCall(self, data):
#        if data.status.status == 1:             
#            self.hasGoal = True
#        else: 
#            self.hasGoal = False
        
        
    def lasercall(self, data):        
        self.laser = data

    def pose_callback(self, data):        
        self.xPosition = data.pose.pose.position.x
        self.yPosition = data.pose.pose.position.y
        self.zOrien = data.pose.pose.orientation.z
        self.wOrien = data.pose.pose.orientation.w
        
    
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
        self.Mred = cv2.moments(redMask)
        #YELLOW
        yellowLower = np.array([25, 90, 20])
        yellowUpper = np.array([45, 255, 255])
        yellowMask = cv2.inRange(hsv_img, yellowLower, yellowUpper)
        self.Myellow = cv2.moments(yellowMask)
        #BLUE
        blueLower = np.array([115, 70, 60])
        blueUpper = np.array([125, 255, 255])
        blueMask = cv2.inRange(hsv_img, blueLower, blueUpper)
        self.Mblue = cv2.moments(blueMask)
        #GREEN
        greenLower = np.array([60, 100, 40])
        greenUpper = np.array([80, 255, 255])
        greenMask = cv2.inRange(hsv_img, greenLower, greenUpper)
        self.Mgreen = cv2.moments(greenMask)
        
        #Image Properties
        h, w, d = cv_image.shape
        self.imageHeight = h
        self.imageWidth = w
        
        #Laserscan ranges
        ranges = self.laser.ranges       
        centre = ((len(ranges)-1)/2)           
        self.middleSlit = ranges[centre]        

        masks = redMask + yellowMask + blueMask + greenMask
        output = cv2.bitwise_and(cv_image, cv_image, mask = masks)        
        cv2.imshow("Robot Vision", cv_image)
        cv2.imshow("Colours", output)
        
        
    def detectColour(self):
        self.colourChecked = True
        print("Detecting")
        if self.Mred['m00'] > 50 and self.foundRed == False:
            print("red")
            if np.nanmin(self.middleSlit >= 0.9):            
                self.trackObject(self.Mred)                
            else:
                self.twistMessage.linear.x = 0
                self.twistMessage.angular.z = 0
                print('Found Red!')
                self.foundRed = True
                self.foundColour = True                
                
        elif self.Myellow['m00'] > 50 and self.foundYellow == False:
            print("yellow")
            if np.nanmin(self.middleSlit >= 0.7):            
                self.trackObject(self.Myellow)  
            else:
                self.twistMessage.linear.x = 0
                self.twistMessage.angular.z = 0
                print('Found Yellow!')
                self.foundYellow = True
                self.foundColour = True
                
        elif self.Mblue['m00'] > 50 and self.foundBlue == False:
            print("green")
            if np.nanmin(self.middleSlit >= 0.7):            
                self.trackObject(self.Mblue)  
            else:
                self.twistMessage.linear.x = 0
                self.twistMessage.angular.z = 0
                print('Found Blue!')
                self.foundBlue = True
                self.foundColour = True
                
        elif self.Mgreen['m00'] > 50 and self.foundGreen == False:
            print("green")
            if np.nanmin(self.middleSlit >= 0.7):            
                self.trackObject(self.Mgreen)                
            else:
                self.twistMessage.linear.x = 0
                self.twistMessage.angular.z = 0
                print('Found Green!')
                self.foundGreen = True
                self.foundColour = True
                
        else:
            self.foundColour = True
        
        
    def moveToPoint(self, x, y, orien):
        Goal = PoseStamped()
        Goal.header.frame_id = "/map"
        Goal.header.stamp = rospy.Time.now()
        Goal.pose.position.x = x
        Goal.pose.position.y = y
        Goal.pose.position.z = 0
        Goal.pose.orientation.x = orien[0]
        Goal.pose.orientation.y = orien[1]
        Goal.pose.orientation.z = orien[2]
        Goal.pose.orientation.w = orien[3]
        self.goal_pub.publish(Goal)         
    
    def trackObject(self, moments):
        cx = int(moments['m10']/moments['m00'])
        err = cx - self.imageWidth/2            
        self.twistMessage.linear.x = 0.7
        self.twistMessage.angular.z = -float(err) / 100
        self.cmd_vel_pub.publish(self.twistMessage)
        
        
if __name__ == '__main__':
    rospy.init_node('Colour_Finder', anonymous=True)
    Robot = Colour_Finder()
    goalSent = False
    
    #Going to Goal 1    
    
    while Robot.goal1Achieved == False:
        if goalSent == False:
            Robot.currentGoal = Robot.position1       
            Robot.moveToPoint(Robot.currentGoal[0], Robot.currentGoal[1], Robot.currentGoal[2])
            goalSent = True            
            print("Goal 1 sent!")
        if (Robot.currentGoal[0]-0.4) <= Robot.xPosition <= (Robot.currentGoal[0]+0.4) and (Robot.currentGoal[1]-0.4) <= Robot.yPosition <= (Robot.currentGoal[1]+0.4) and goalSent == True :
            print("Goal 1 Achieved!")
            Robot.goal1Achieved = True
            Robot.currentGoal = Robot.position2
            goalSent = False
            Robot.colourChecked = False
            Robot.foundColour = False
    
    #Checking for coloured objects at Goal 1
    
    if Robot.colourChecked == False:
        print("Finding Colour")
        while Robot.foundColour == False:            
            Robot.detectColour()
    print Robot.foundColour
    
    #Going to Goal 2    
    
    while Robot.goal2Achieved == False:
        if goalSent == False:
            Robot.moveToPoint(Robot.currentGoal[0], Robot.currentGoal[1], Robot.currentGoal[2])
            goalSent = True
            print("Goal 2 sent!")
        if (Robot.currentGoal[0]-0.4) <= Robot.xPosition <= (Robot.currentGoal[0]+0.4) and (Robot.currentGoal[1]-0.4) <= Robot.yPosition <= (Robot.currentGoal[1]+0.4) and goalSent == True:
            print("Goal 2 Achieved!")
            Robot.goal2Achieved = True
            Robot.currentGoal = Robot.position3
            goalSent = False
            Robot.colourChecked = False
            Robot.foundColour = False
            print Robot.foundColour
            
    #Checking for coloured objects at Goal 2
            
    if Robot.colourChecked == False:
        print("Finding Colour")
        while Robot.foundColour == False:
            Robot.detectColour()
    print Robot.foundColour
            
    #Move to Goal 3
            
    while Robot.goal3Achieved == False:
        if goalSent == False:
            Robot.moveToPoint(Robot.currentGoal[0], Robot.currentGoal[1], Robot.currentGoal[2])
            goalSent = True
            print("Goal 3 sent!")
        if (Robot.currentGoal[0]-0.4) <= Robot.xPosition <= (Robot.currentGoal[0]+0.4) and (Robot.currentGoal[1]-0.4) <= Robot.yPosition <= (Robot.currentGoal[1]+0.4) and goalSent == True:
            print("Goal 3 Achieved!")
            Robot.goal3Achieved = True
            Robot.currentGoal = Robot.position4
            goalSent = False
            Robot.colourChecked = False
            Robot.foundColour = False
            print Robot.foundColour
     
    #Checking for coloured objects at Goal 3     
     
    if Robot.colourChecked == False:
        print("Finding Colour")
        print Robot.foundColour
        while Robot.foundColour == False:
            Robot.detectColour()
        
    
    rospy.spin()
    
    cv2.destroyAllWindows()

    