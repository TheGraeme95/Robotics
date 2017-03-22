import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped #Message type for twist messages and coordinate goals.
from move_base_msgs.msg import MoveBaseActionResult #Message type for goal results
from cv_bridge import CvBridge, CvBridgeError


class Colour_Finder:
    def __init__(self):
        cv2.namedWindow("Robot Vision", 1)
        cv2.namedWindow("Colours", 1)
        cv2.startWindowThread()
        self.laser = LaserScan()       
        self.laser_sub = rospy.Subscriber("/turtlebot/scan", LaserScan, self.laser_callback, queue_size = 10)
        rospy.sleep(1) #Gives the laserscan time to work
        self.bridge = CvBridge()
        #Subscriber for the camera
        self.image_sub = rospy.Subscriber("/turtlebot/camera/rgb/image_raw", Image, self.callback, queue_size = 10)        
        #Subscriber for confirming that the robot has reached the goal        
        self.result_sub = rospy.Subscriber("/turtlebot/move_base/result", MoveBaseActionResult, self.resultCall, queue_size = 10)
        #Publishes twist messages to the robot when it is tracking the objects        
        self.cmd_vel_pub = rospy.Publisher("/turtlebot/cmd_vel", Twist, queue_size=10)
        #Publisher for setting coordinate goals
        self.goal_pub = rospy.Publisher("/turtlebot/move_base_simple/goal", PoseStamped, queue_size = 10)        
        self.twistMessage = Twist()

        #Logical flags that stop the robot finding each colour more than once        
        
        self.foundRed = False
        self.foundYellow = False
        self.foundBlue = False
        self.foundGreen = False
        
        #Defining chosen position coordinates and quaternions for orientation
        
        self.position1 = [-0.1, -4.4,[0,0,0,1]]   
        self.position2 = [-0.536, -1.63, [0,0,-1,0]]
        self.position3 = [-4.4, 0.36, [0,0,-0.074,0.997]]
        self.position4 = [-3.81, 1.89, [0,0,0.371,0.928]]
        self.position5 = [-1.7, -1.61, [0,0,0.0164,0.999]]
        self.position6 = [-0.826, 0.823, [0,0,0.713,0.701]]
        self.position7 = [-0.41, 4.08, [0,0,0.1,0.994]]
        self.position8 = [0.271, 2.06, [0,0,-0.0347,0.999]]
        self.position9 = [2.40, 1.875, [0,0,-0.583,0.812]]
        self.position10 = [0.25, -4.49,[0,0,1,0.02]]
        
        #Logical flags for controlling the flow of the robot's behaviour.        
        #Goal flags.
        self.goalSent = False
        self.goalResult = False
        self.goalAchieved = False
        self.goalCounter = 1
        #Colour tracked and finding flags.
        self.foundColour = False
        self.colourChecked = False
        
        #Initialising robot's speed as 0
        self.twistMessage.linear.x = 0
        self.twistMessage.angular.x = 0
        
        #Sleep to give subcribers/publishers time to respond
        rospy.sleep(3)
        
    
    #When the robot reaches the goal, the result topic will be called confirming it has reached the goal
    def resultCall(self, data):
        self.goalResult = True
      
    #Set the data received from the laser scanner to a accessible variable. 
    def laser_callback(self, data):        
        self.laser = data        
    
    #Calback that handles any information provided from the camera feed
    #All image processing and colour slicing is performed in this callback.
    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            
        except CvBridgeError, e:
            print e
        
        #Image Properties
        h, w, d = cv_image.shape
        self.imageHeight = h
        self.imageWidth = w            
        
        #Colour Slicing
        #Each colour is segmented using an upper and lower bound containing HSV values. [Hue, Saturation, Value]
        #Each colour will be detected using the specified mask and the moments of each image are gained to find the centre of the object.
        #The moment variables are used to centre the vision of the robot on it and adjust speed accordingly.        
        hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        #Red    
        redLower = np.array([0, 90, 50])
        redUpper = np.array([8, 255, 255])
        redMask = cv2.inRange(hsv_img, redLower, redUpper)
        self.Mred = cv2.moments(redMask)
        #Yellow
        yellowLower = np.array([25, 200, 100])
        yellowUpper = np.array([30, 255, 255])
        yellowMask = cv2.inRange(hsv_img, yellowLower, yellowUpper)
        self.Myellow = cv2.moments(yellowMask)
        #Blue
        blueLower = np.array([115, 70, 60])
        blueUpper = np.array([125, 255, 255])
        blueMask = cv2.inRange(hsv_img, blueLower, blueUpper)
        self.Mblue = cv2.moments(blueMask)
        #Green
        greenLower = np.array([40, 200, 100])
        greenUpper = np.array([60, 255, 255])
        greenMask = cv2.inRange(hsv_img, greenLower, greenUpper)
        self.Mgreen = cv2.moments(greenMask)
        
        #Laserscan ranges
        #The centre value of the laserscanner is taken. 
        ranges = self.laser.ranges         
        centre = ((len(ranges)-1)/2)           
        self.middleSlit = ranges[centre] #Centre value
        
        #Displays the standard vision and masked vision of the robot.
        masks = redMask + yellowMask + blueMask + greenMask
        output = cv2.bitwise_and(cv_image, cv_image, mask = masks)        
        cv2.imshow("Robot Vision", cv_image)
        cv2.imshow("Colours", output)
        
      
    #After each goal the robot will check for a colour by running through this function.
    #It will always check only once before moving on the the next goal.
    #It also resets the goalAchieved flag so that the robot will peform the nexy goal.
    #If any of the moments of the image are above the specified threshold then it will call the tracking function.
    #The tracking function will be continuously called until the centre laserscan value is less than 0.9 indicating
    #that the robot has found the specific colour, checking that colour off the list so it won't find it again.
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
        
    #This function creates a PoseStamped message which is then published to the Move_base_simple/goal topic
    #It defines x and y coordinates along with 4 different quaternion values which specific the orientaion.
    #An array which contains these values is passed to this function.    
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
        
        
    
    #This function makes the robot move to the specified position. It calls the moveToPoint function to create and publish the PoseStamped message.
    #It also handles the logic to ensure that the goal is only sent once per position.
    #When the robot reaches the position it will flag that it has which will break it out of the while loop to then check for colours.
    def goToGoal(self, position):        
        if self.goalSent == False: #Ensures that the goal is only sent once for each position
            self.moveToPoint(position[0], position[1], position[2]) # Calls the goal setting function
            self.goalSent = True # Flags that the goal has been sent
            print "Goal", self.goalCounter,"sent!"
        if self.goalResult == True and self.goalSent == True: ## When the robot reaches the goal that has been sent it will exit the loop
            print "Goal", self.goalCounter, "Achieved!"
            self.goalAchieved = True ## Sets goal achieved to true to break the encompassing while loop
            self.goalSent = False ## Resets the goal sent flag so the next goal can be sent
            self.colourChecked = False ## Resets the state of whether the robot has checked for a colour after each goal
            self.foundColour = False
            self.goalCounter += 1 # adds to goal counter so messages display properly.         
            
            
    #Adjusts the velocity of the robot so that it will move towards the centre of the coloured objects.
    def trackObject(self, moments):
        cx = int(moments['m10']/moments['m00']) #Calculates the centroid of the coloured object
        err = cx - self.imageWidth/2 #Calculates how far off the centroid is from the middle of the image.      
        self.twistMessage.linear.x = 0.5
        self.twistMessage.angular.z = -float(err) / 100 #Adjusts the angular speed so that the centroid of the object is kept within the centre of the image.
        self.cmd_vel_pub.publish(self.twistMessage)
        

#########################################################################################################

#The main function is looped over sequentially while the callbacks are gathering data about the robot.
#Any specific movement instruction is performed in the main so that behvaiour can be controlled.       
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
            
    #Sending the robot to the ninth position
            
    Robot.goalResult = False    
    while Robot.goalAchieved == False:
        Robot.goToGoal(Robot.position9)
        
    if Robot.colourChecked == False:
        print("Checking for Colour")
        while Robot.foundColour == False:            
            Robot.detectColour()
            
    #Sending the robot to the tenth position
            
    Robot.goalResult = False    
    while Robot.goalAchieved == False:
        Robot.goToGoal(Robot.position10)
        
    if Robot.colourChecked == False:
        print("Checking for Colour")
        while Robot.foundColour == False:            
            Robot.detectColour()
    
    #Once all goals have been visited, the robot will say whether it has found all the colours.    
    print("Visited all goals")
    if Robot.foundBlue == True and Robot.foundGreen == True and Robot.foundRed == True and Robot.foundYellow == True:
        print "Found all colours!"
    rospy.spin()
    
    cv2.destroyAllWindows()

#############################################################################################################