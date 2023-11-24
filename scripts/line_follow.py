#!/usr/bin/env python3

# Import base ROS
import rospy

# Import OpenCV and NumPy
import cv2 as cv
import numpy as np

# Import ROS message information
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Import dynamic reconfigure 
from dynamic_reconfigure.server import Server
from l2_bot.cfg import LineFollowDynCfgConfig


ACTIVE_WINDOWS = []

##################################
# LineFollowNode class definition
##################################
class LineFollowNode():
    def __init__(self):
        """Line following node"""

        # Initialize dynamic reconfigure
        self.enable = 0
        self.gain = 0.0
        self.steer = 0.0
        
        self.speed_desired = 0.0 # @1m/s, GAIN = 3.5
        self.frame_skip = 1
        self.hue_low = 0
        self.hue_high = 0
        self.sat_low = 0
        self.sat_high = 0
        self.val_low = 0
        self.val_high = 0
        self.mask_top = 1
        self.mask_right = 1
        self.targ_line= 1
        self.test_length = 0
        self.test_time = 0
        self.track_width = 0
        self.wheel_radius = 0
        self.maxRPM = 0
        self.rpm2analog = 1

        # Initialize frame counter
        self.frame_count = 1
        
        # Initialize robot motion
        self.steer = 0.0

        '''# Open webcam
        self.br = CvBridge()
        self.vid = None
        self.ret = False
        self.pub_image = rospy.Publisher('webcam/image', Image, queue_size=1)'''
        # Define the image subscriber
        self.image_view = rospy.Subscriber('camera/image_raw', Image,
                                          self.camera_callback, queue_size=1)
        
        # Define cmd publisher
        self.pub_twist = rospy.Publisher('cmd_vel',
                                         Twist, queue_size=1)
        # Define communication publisher
        self.pub_arduino = rospy.Publisher('comms/arduino', Float64, queue_size=1)
                                                                        
        # Set up dynamics reconfigure
        self.srv = Server(LineFollowDynCfgConfig,
                          self.dyn_reconfig_callback)

        # Define ROS rate
        self.rate = rospy.Rate(20)  # Vehicle rate

        # Loop and publish commands to vehicle
        while not rospy.is_shutdown():
            '''self.vid = cv.VideoCapture(4)
            self.ret, self.frame = self.vid.read()
            self.frame = cv.cvtColor(self.frame, cv.COLOR_BGR2RGB)'''
            # Build message yaw and speed rate message and publish
            if self.enable:
                twist_msg.linear.x = self.speed_desired
                twist_msg.angular.z = self.steer
            else:
                twist_msg = Twist()
            self.maxRPM = (((self.test_length/(2*np.pi*self.wheel_radius))/self.test_time)*60)
            self.rpm2analog = 255/self.maxRPM
            '''if self.vid is not None:
                self.pub_image.publish(self.br.cv2_to_imgmsg(self.frame))'''
            self.pub_arduino.publish(self.rpm2analog)
            self.pub_twist.publish(twist_msg)
            
            # Sleep for time step
            self.rate.sleep()
            
        return

    ################################
    # Dynamic Reconfigure callback
    ################################
    def dyn_reconfig_callback(self, config, level):
        self.enable = config['enable']
        self.gain = config['gain']
        self.speed_desired = config['speed']
        self.frame_skip = config['frame_skip']

        # Add HSV hue, sat and val
        self.hue_low = config['hue_low']
        self.hue_high = config['hue_high']
        self.sat_low = config['sat_low']
        self.sat_high = config['sat_high']
        self.val_low = config['val_low']
        self.val_high = config['val_high']
        self.mask_top = config['mask_top']
        self.mask_right = config['mask_right']
        self.targ_line = config['targ_line']
        self.test_length = config['test_length']
        self.test_time = config['test_time']
        self.track_width = config['track_width']
        self.wheel_radius = config['wheel_radius']
        
        self.dyn_config = config
        
        return config

    #########################
    # Camera image callback
    #########################
    def camera_callback(self, rgb_msg):

        # Check frame counter
        if( self.frame_count % self.frame_skip != 0 ):
            self.frame_count += 1
            return
        self.frame_count = 1
        
        # Get the camera image and make a copy
        img = CvBridge().imgmsg_to_cv2(rgb_msg, "bgr8" )
        

        # Convert image to a HSV image and blur
        img_hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        img_blur = cv.medianBlur(img_hsv, 7)
        
        # Threshold HSV image to binary based on range
        img_hsv_thresh = cv.inRange(img_blur,
                                   (self.hue_low, 0, 0),
                                   (self.hue_high, 255, 255))
        thresh_copy = img_hsv_thresh
        self.display_image('HSV Image Threshold', thresh_copy, True)
        img_masked = img_hsv_thresh                        
        rows, cols = img_masked.shape[:2]
        start_point_top = (0,0)
        end_point_top = (cols,int(rows/self.mask_top))
        start_point_right = (int(cols/self.mask_right),0)
        end_point_right = (cols,rows)
        color = (0,0,0)
        mask_top = cv.rectangle(img_masked, start_point_top, end_point_top, color, -1)
        mask_right = cv.rectangle(img_masked, start_point_right, end_point_right, color, -1)
        img_masked = cv.bitwise_and(img_masked,mask_top)
        img_masked = cv.bitwise_and(img_masked,mask_right)

        self.display_image('Masked Image', img_masked, True)
        

        # Count total pixels and non-zero pixels
        total_pixels = rows*cols
        targ_line = cols/self.targ_line
        point1, point2 = (int(cols/self.targ_line), 0), (int(cols/self.targ_line), rows)
        cv.line(img, point1, point2, (0,200,255), 1)
        nnz_pixels = cv.countNonZero(img_hsv_thresh)
        
        ## Check if no lines are observed and spin
        ## Set speed and return!

        if nnz_pixels <= 3000:
            self.speed_desired = 0.0
        else:
            self.speed_desired = self.dyn_config['speed']
        self.contours, self.hierarchy = cv.findContours(img_hsv_thresh, cv.RETR_TREE,
                                               cv.CHAIN_APPROX_SIMPLE)
        cv.drawContours(img_masked, self.contours, -1, (0,0,255), 1)
        for i, contour in enumerate(self.contours):
            self.cnt = self.contours[i]
            M = cv.moments(self.cnt)
            CG_x = M['m10']/M['m00'] if M['m00'] > 0 else 0
            CG_y = M['m01']/M['m00'] if M['m00'] > 0 else 0
            CG = np.array([CG_x, CG_y])
            cv.circle(img, (int(CG[0]), int(CG[1])), 7, (0, 0, 255), -1)
            if CG_x < (int(cols/2)):
                # Calculate error based on the center of the image and the gain
                err = targ_line - CG[0]
                p_control = self.gain * err/targ_line 
                self.speed_desired = self.dyn_config['speed']
                self.steer = p_control
            else:
                pass
        self.display_image('Centroid', img, True)        
        return
    
    ########################
    # Arduino Comms Callback
    ########################
    def arduino_callback(self):
        self.maxRPM = (((self.test_length/(2*np.pi*self.wheel_radius))/self.test_time)*60)
        self.rpm2analog = 255/self.maxRPM
        self.pub_arduino.publish(self.rpm2analog)
        return
    ####################
    # Display an image
    ####################
    def display_image(self, title_str, img, disp_flag ):

        if( disp_flag ):
            # Display the given image
            cv.namedWindow(title_str, cv.WINDOW_NORMAL)
            cv.imshow(title_str, img)
            cv.waitKey(3)

            # Add window to active window list
            if not ( title_str in ACTIVE_WINDOWS ):
                ACTIVE_WINDOWS.append(title_str)
        else:
            if( title_str in ACTIVE_WINDOWS):
                cv.destroyWindow(title_str)
                ACTIVE_WINDOWS.remove(title_str)
        return
    


#################    
# Main function
#################
if __name__ == '__main__':
    
    # Initialize the node and name it.
    rospy.init_node('line_folllow_node')
    print("Line Follow node initialized")
    
    # Start node
    try:
        LineFollowNode()
    except rospy.ROSInterruptException:
        pass
