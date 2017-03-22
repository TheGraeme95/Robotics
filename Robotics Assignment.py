import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped
from move_base_msgs.msg import MoveBaseActionResult
#from geometry_msgs.msg import Pose, Point, Quarternion
from cv_bridge import CvBridge, CvBridgeError


class Colour_Finder:
    def __init__(self):
        cv2.namedWindow("Robot Vision", 1)
        cv2.namedWindow("Colours", 1)
        cv2.startWindowThread()
        self.laser = LaserScan()       
        self.laser_sub = rospy.Subscriber("/turtlebot/scan", LaserScan, self.laser_callback, queue_size = 10)
        rospy.sleep(1)
        self.bridge = CvBridge()         
        self.image_sub = rospy.Subscriber("/turtlebot/camera/rgb/image_raw", Image, self.callback, queue_size = 10)        
        self.result_sub = rospy.Subscriber("/turtlebot/move_base/result", MoveBaseActionResult, self.resultCall, queue_size = 10)
        self.cmd_vel_pub = rospy.Publisher("/turtlebot/cmd_vel", Twist, queue_size=10)      
        self.goal_pub = rospy.Publisher("/turtlebot/move_base_simple/goal", PoseStamped, queue_size = 10)        
        self.twistMessage = Twist()        
        self.foundRed = False
        self.foundYellow = False
        self.foundBlue = False
        self.foundGreen = False
        self.position1 = [-0.1, -4.4,[0,0,0,1]]
        self.position2 = [-0.536, -1.63, [0,0,-1,0]]
        self.position3 = [-4.4, 0.36, [0,0,-0.074,0.997]]
        self.position4 = [-3.81, 1.89, [0,0,0.371,0.928]]
        self.position5 = [-1.7, -1.61, [0,0,0.0164,0.999]]
        self.position6 = [-0.826, 0.823, [0,0,0.713,0.701]]
        self.position7 = [-0.41, 4.08, [0,0,0.1,0.994]]
        self.position8 = [0.271, 2.06, [0,0,-0.0347,0.999]]
        self.position9 = [2.40, 1.875, [0,0,-0.583,0.812]]
        self.goalSent = False
        self.foundColour = False
        self.goalResult = False
        self.twistMessage.linear.x = 0
        self.twistMessage.angular.x = 0
        self.colourChecked = False
        self.goalAchieved = False
        self.goalCounter = 1
        rospy.sleep(3)
        
        
    def resultCall(self, data):
        self.goalResult = True  
      
        
    def laser_callback(self, data):        
        self.laser = data        
    
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
        yellowLower = np.array([25, 200, 100])
        yellowUpper = np.array([30, 255, 255])
        yellowMask = cv2.inRange(hsv_img, yellowLower, yellowUpper)
        self.Myellow = cv2.moments(yellowMask)
        #BLUE
        blueLower = np.array([115, 70, 60])
        blueUpper = np.array([125, 255, 255])
        blueMask = cv2.inRange(hsv_img, blueLower, blueUpper)
        self.Mblue = cv2.moments(blueMask)
        #GREEN
        greenLower = np.array([40, 200, 100])
        greenUpper = np.array([60, 255, 255])
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
        Robot.goalAchieved = False
        if self.Mred['m00'] > 50 and self.foundRed == False:            
            if np.nanmin(self.middleSlit >= 0.9):            
                self.trackObject(self.Mred)                
            else:
                self.twistMessage.linear.x = 0
                self.twistMessage.angular.z = 0
                print('Found Red!')
                self.foundRed = True
                self.foundColour = True                
                
        elif self.Myellow['m00'] > 50 and self.foundYellow == False:            
            if np.nanmin(self.middleSlit >= 0.9):            
                self.trackObject(self.Myellow)  
            else:
                self.twistMessage.linear.x = 0
                self.twistMessage.angular.z = 0
                print('Found Yellow!')
                self.foundYellow = True
                self.foundColour = True
                
        elif self.Mblue['m00'] > 50 and self.foundBlue == False:            
            if np.nanmin(self.middleSlit >= 0.9):            
                self.trackObject(self.Mblue)  
            else:
                self.twistMessage.linear.x = 0
                self.twistMessage.angular.z = 0
                print('Found Blue!')
                self.foundBlue = True
                self.foundColour = True
                
        elif self.Mgreen['m00'] > 50 and self.foundGreen == False:            
            if np.nanmin(self.middleSlit >= 0.9):            
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
        
    def goToGoal(self, position):        
        if self.goalSent == False:
            self.moveToPoint(position[0], position[1], position[2])
            self.goalSent = True
            print "Goal", self.goalCounter,"sent!"
        if self.goalResult == True and self.goalSent == True:
            print "Goal", self.goalCounter, "Achieved!"
            self.goalAchieved = True
            self.goalSent = False
            self.colourChecked = False
            self.foundColour = False
            self.goalCounter += 1
            
            
            
            
    def trackObject(self, moments):
        cx = int(moments['m10']/moments['m00'])
        err = cx - self.imageWidth/2            
        self.twistMessage.linear.x = 0.5
        self.twistMessage.angular.z = -float(err) / 100
        self.cmd_vel_pub.publish(self.twistMessage)
        
        
if __name__ == '__main__':
    rospy.init_node('Colour_Finder', anonymous=True)
    Robot = Colour_Finder()    
    
    
    #Sending the robot to first position
    
    Robot.goalResult = False
    while Robot.goalAchieved == False:
        Robot.goToGoal(Robot.position1)
        
    #Checking for colours after each goal
    #The robot only checks for colours once after each goal before moving to next goal.
        
    if Robot.colourChecked == False:
        print("Checking for Colour")
        while Robot.foundColour == False: #If it hasn't already found a colour then it will detect one.    
            Robot.detectColour() #Checks for colour atleast once. If it sees a new one then it will run to it.
    
    #Sending the robot to the second position
     
    Robot.goalResult = False    
    while Robot.goalAchieved == False:
        Robot.goToGoal(Robot.position2)
        
    if Robot.colourChecked == False:
        print("Checking for Colour")
        while Robot.foundColour == False:            
            Robot.detectColour()
            
     #Sending the robot to third position
            
    Robot.goalResult = False    
    while Robot.goalAchieved == False:
        Robot.goToGoal(Robot.position3)
        
    if Robot.colourChecked == False:
        print("Checking for Colour")
        while Robot.foundColour == False:            
            Robot.detectColour()
            
     #Sending the robot to fourth position
            
    Robot.goalResult = False    
    while Robot.goalAchieved == False:
        Robot.goToGoal(Robot.position4)
        
    if Robot.colourChecked == False:
        print("Checking for Colour")
        while Robot.foundColour == False:            
            Robot.detectColour()
            
     #Sending the robot to fifth position
            
    Robot.goalResult = False    
    while Robot.goalAchieved == False:
        Robot.goToGoal(Robot.position5)
        
    if Robot.colourChecked == False:
        print("Checking for Colour")
        while Robot.foundColour == False:            
            Robot.detectColour()
            
     #Sending the robot to sixth position
            
    Robot.goalResult = False    
    while Robot.goalAchieved == False:
        Robot.goToGoal(Robot.position6)
        
    if Robot.colourChecked == False:
        print("Checking for Colour")
        while Robot.foundColour == False:            
            Robot.detectColour()
            
     #Sending the robot to seventh position
            
    Robot.goalResult = False    
    while Robot.goalAchieved == False:
        Robot.goToGoal(Robot.position7)
        
    if Robot.colourChecked == False:
        print("Checking for Colour")
        while Robot.foundColour == False:            
            Robot.detectColour()
            
     #Sending the robot to eighth position
            
    Robot.goalResult = False    
    while Robot.goalAchieved == False:
        Robot.goToGoal(Robot.position8)
        
    if Robot.colourChecked == False:
        print("Checking for Colour")
        while Robot.foundColour == False:            
            Robot.detectColour()
            
    Robot.goalResult = False    
    while Robot.goalAchieved == False:
        Robot.goToGoal(Robot.position9)
        
    if Robot.colourChecked == False:
        print("Checking for Colour")
        while Robot.foundColour == False:            
            Robot.detectColour()
            
    print("Visited all goals")
    if Robot.foundBlue == True and Robot.foundGreen == True and Robot.foundRed == True and Robot.foundYellow == True:
        print "Found all colours!"
    rospy.spin()
    
    cv2.destroyAllWindows()

    