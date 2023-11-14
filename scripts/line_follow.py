#!/usr/bin/env python3

# Import base ROS
import rospy

# Import OpenCV and NumPy
import cv2 as cv
import numpy as np

# Import ROS message information
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Import dynamic reconfigure 
from dynamic_reconfigure.server import Server
from hw08_crist.cfg import LineFollowDynCfgConfig


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
        self.blur = 0

        # Initialize frame counter
        self.frame_count = 1
        
        # Initialize robot motion
        self.steer = 0.0

        # Define the image subscriber
        self.image_view = rospy.Subscriber('camera/image_raw', Image,
                                          self.camera_callback, queue_size=1)
        
        # Define publisher
        self.pub_twist = rospy.Publisher('cmd_vel',
                                         Twist, queue_size=1)
                                         
        # Set up dynamics reconfigure
        self.srv = Server(LineFollowDynCfgConfig,
                          self.dyn_reconfig_callback)

        # Define ROS rate
        self.rate = rospy.Rate(20)  # Vehicle rate

        # Loop and publish commands to vehicle
        while not rospy.is_shutdown():

            # Build message yaw and speed rate message and publish
            if self.enable:
                twist_msg.linear.x = self.speed_desired
                twist_msg.angular.z = self.steer
            else:
                twist_msg = Twist()
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
        self.blur = config['blur']
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
        # print('Orig', img.shape)

        # Convert image to a HSV image and blur
        img_hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        img_hsv = cv.medianBlur(img_hsv, self.blur)
        
        # Threshold HSV image to binary based on range
        img_hsv_thres = cv.inRange(img_hsv,
                                   (self.hue_low, 0, 0),
                                   (self.hue_high, 255, 255))
                                   
        rows, cols = img_hsv_thres.shape
        # print('Thres Shape = ', img_hsv_thres.shape)
        self.display_image('HSV Image Threshold', img_hsv_thres, True)

        # Count total pixels and non-zero pixels
        total_pixels = rows*cols
        targ_line = np.array([cols/5, rows/2])
        point1, point2 = (int(cols/5), 0), (int(cols/5), rows)
        cv.line(img, point1, point2, (0,200,255), 1)
        nnz_pixels = cv.countNonZero(img_hsv_thres)
        # rospy.loginfo('Total / NNZ: %5d / %5d' % (total_pixels, nnz_pixels))
        # 3000 non-zero pixels is sufficient
        
        
        ##
        ## Check if no lines are observed and spin
        ## Set speed and return!
        ##
        #
        if nnz_pixels <= 3000:
            self.speed_desired = 0.0
        else:
            self.speed_desired = self.dyn_config['speed']
        self.contours, self.hierarchy = cv.findContours(img_hsv_thres, cv.RETR_TREE,
                                               cv.CHAIN_APPROX_SIMPLE)
        cv.drawContours(img, self.contours, -1, (255,255,0), 1)
        for i, contour in enumerate(self.contours):
            self.cnt = self.contours[i]
            M = cv.moments(self.cnt)
            CG_x = M['m10']/M['m00'] if M['m00'] > 0 else 0
            CG_y = M['m01']/M['m00'] if M['m00'] > 0 else 0
            CG = np.array([CG_x, CG_y])
            cv.circle(img, (int(CG[0]), int(targ_line[1])), 7, (0, 0, 255), -1)
            if CG_x < (int(cols/2)):
                # Calculate error based on the center of the image and the gain
                err = targ_line[0] - CG[0]
                p_control = self.gain * err/targ_line[0] 
                self.speed_desired = self.dyn_config['speed']
                self.steer = p_control
            else:
                pass
                #self.speed_desired = 0.0
                #self.steer = self.steer - 1
        # Creating moment to find centroid of the white pixels
        '''M = cv.moments(img_hsv_thres)
        CG_x = M['m10']/M['m00'] if M['m00'] > 0 else 0
        CG_y = M['m01']/M['m00'] if M['m00'] > 0 else 0
        CG = np.array([CG_x, CG_y])
        
        if CG_y > (int(rows/3)):
            # Calculate error based on the center of the image and the gain
            err = targ_line[0] - CG[0]
            p_control = self.gain * err/targ_line[0]
        else:
            p_control = p_control -1'''
        # Display centroid image
        
        
        
        self.display_image('Centroid', img, True)

        

        ##############################
        # Use for debugging purposes: 
        ##############################
        
        # print('m00  ', M['m00'])
        # print('m01  ', M['m01'])
        # print('m10  ', M['m10'])
        
        # print('m10/m00  ', M['m10']/M['m00'])   # X Centroid
        # print('m01/m00  ', M['m01']/M['m00'])   # Y Centroid
        # print('Center = ', cols/2) 
        # print('Error = ', err)
        
        ##############################
                
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
