# This code has been customised to run on a Turtlebot

# Imports all the bits that are needed by the program.
import rospy
import cv2
import numpy
import math
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

class behave:
    
    # Constructor for the behave class where vairables are set up for the class such as Publishers and Subscribers.
    def __init__(self):
        # Creates the window for the image to be displayed in.
        cv2.namedWindow("Image window", 1)
        cv2.startWindowThread()
        self.bridge = CvBridge()
        
        # Sets up the publisher where a Twist message shall be published from the node.
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # Sets up the subscribers to retreive data from the robots laserscanner and rgb camera.
        self.laser_sub = rospy.Subscriber("/scan", LaserScan, self.lasercallback)
        self.image_sub = rospy.Subscriber("/camera/rgb/image_color", Image, self.callback)
                     
    # Callback for the laser that sets the laser data to a vairable.                     
    def lasercallback(self, data):
        self.laser = data           
                    
    # Callback for camera that will move the robot based on its vision.
    def callback(self, data):
        try:
            # Reads in the image and sets the height, width and finds the brightness of the image.
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            height, width, channels = cv_image.shape
            cv_image = cv2.medianBlur(cv_image, 5)
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            mean = numpy.mean(hsv_image[:, :, 2])
            #print mean

            # Splits the image into left and right.
            left = hsv_image[0:height,0:width/2]
            right = hsv_image[0:height,width/2:width]
            height, width, channels = left.shape
            
        except CvBridgeError, e:
            print e

        # Default values are set.
        l_wheel = 0
        r_wheel = 0
        hit = False
        twist_msg = Twist()  

        # Laser ranges are read and split into left and right.
        ranges = self.laser.ranges
        middle = int(round((len(ranges)-1)/2))
        center = ranges[middle]
        leftRanges = ranges[middle-100:middle]
        rightRanges = ranges[middle:middle+100]

        # If the robot is too close to an obstical it will enter recovery until it is able to move.  
        if numpy.nanmin(ranges) < 0.8:
            print "Too Close"
            twist_msg.angular.z = 0.2

        # If the robot has room to move.
        else:
            # The left and right images are segmented based on colour.
            leftRange = numpy.zeros((height,width,1), numpy.uint8)
            rightRange = numpy.zeros((height,width,1), numpy.uint8)
            cv2.inRange(left, numpy.array([70, 50, 50]), numpy.array([100, 200, 200]), leftRange)
            cv2.inRange(right, numpy.array([70, 50, 50]), numpy.array([100, 200, 200]), rightRange)    

            # Set the wheel speed based on if green is seen in the left image.
            if cv2.countNonZero(leftRange) > 1000:
                # Based on the brightness of the image the robot will change the wheel that is moved changing the emotion.
                if mean > 30:
                    l_wheel = 1.0 # Love
                else:
                    r_wheel = 1.0 # Fear
                hit = True

            # Similar to the if statment but with the right image.
            if cv2.countNonZero(rightRange) > 1000:
                if mean > 30:
                    r_wheel = 1.0 # Love
                else:
                    l_wheel = 1.0 # Fear
                hit = True
            
            # Makes the robot approtch the target.
            if hit:
                # If both values are equal the target is centered.
                if r_wheel == l_wheel:
                    print "center"
                    if math.isnan(center):                        # Checks if center range is not NaN.
                        value = 1.0
                    else:
                        value = 1.0 - (1.0/center)                # Calculates wheel values for robot to approtch target.
                        if value <= 0:
                            value = 0
                    (v, a) = forward_kinematics(value, value)     # Calculates the velocity based on range to the target.
                    
                else:
                    (v, a) = forward_kinematics(l_wheel, r_wheel) # Calcualtes velocity based on wheel values
                    
                # Sets velocity to message
                twist_msg.linear.x = v
                twist_msg.angular.z = a
            
            # Makes the robot explore/
            else:
                # Makes the robot explore forwards when nothing is obstructing it.
                if numpy.nanmin(leftRanges) > 2.0 and numpy.nanmin(rightRanges) > 2.0:
                    print "explore forward"
                    twist_msg.linear.x = 0.5
                    twist_msg.angular.z = 0.0
                # Turns the robot left when something is on the right of the robot.
                elif numpy.nanmin(leftRanges) > 2.0 and numpy.nanmin(rightRanges) < 2.0:
                    print "explore left"
                    twist_msg.linear.x = 0.0
                    twist_msg.angular.z = 0.5
                # Turns the robot right when something is on the left.
                elif numpy.nanmin(leftRanges) < 2.0 and numpy.nanmin(rightRanges) > 2.0:
                    print "explore right"
                    twist_msg.linear.x = 0.0
                    twist_msg.angular.z = -0.5
                # If the robot is unsure about ranges will spin until sure.
                else:
                    print "explore recover"
                    twist_msg.linear.x = 0.0
                    twist_msg.angular.z = 0.5
            
        self.pub.publish(twist_msg)           # Publishes the the Twist Message to cmd_vel.
        cv2.imshow("Image window", cv_image)  # Displays cv_image to the image window.

# Fucnction for converting wheel speeds into a linear and angular velocity. (Hanhide, 2016)
def forward_kinematics(w_l, w_r):
    c_l = 1 * w_l
    c_r = 1 * w_r
    v = (c_l + c_r) / 2
    a = (c_l - c_r) / 1
    return (v, a)

rospy.init_node('node', anonymous=True)       # Initiates the Ros node that will publish topics from.
behave()                                  # Runs the class to control the robot.
rospy.spin()                              # Make the application run until exited.
cv2.destroyAllWindows()                   # Closes all windows opened by OpenCVx