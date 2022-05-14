#!/usr/bin/env python3
#zone colour detection
#made using com3528 wiki
# Imports
##########################
import os
import numpy as np  # Numerical Analysis library
import cv2  # Computer Vision library

import rospy  # ROS Python interface
from sensor_msgs.msg import CompressedImage  # ROS CompressedImage message
from sensor_msgs.msg import JointState  # ROS joints state message
from cv_bridge import CvBridge, CvBridgeError  # ROS -> OpenCV converter
from geometry_msgs.msg import TwistStamped  # ROS cmd_vel (velocity control) message


class MiRoClient:
     
    """
    Script settings below
    """
    TICK = 0.02  # This is the update interval for the main control loop in secs

    def callback_caml(self, ros_image):  # Left camera
        self.callback_cam(ros_image, 0)

    def callback_camr(self, ros_image):  # Right camera
        self.callback_cam(ros_image, 1)

    def callback_cam(self, ros_image, index):
        """
        Callback function executed upon image arrival
        """
        # Silently(-ish) handle corrupted JPEG frames
        try:
            # Convert compressed ROS image to raw CV image
            image = self.image_converter.compressed_imgmsg_to_cv2(ros_image, "rgb8")
            # Convert from OpenCV's default BGR to RGB
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            # Store image as class attribute for further use
            self.input_camera[index] = image
            # Get image dimensions
            self.frame_height, self.frame_width, channels = image.shape
            self.x_centre = self.frame_width / 2.0
            self.y_centre = self.frame_height / 2.0
            # Raise the flag: A new frame is available for processing
            self.new_frame[index] = True
        except CvBridgeError as e:
            # Ignore corrupted frames
            pass

    def detect_colour(self, frame, index):
        """
        Image processing operations, fine-tuned to detect a small,
        toy blue ball in a given frame.
        """
        if frame is None:  # Sanity check
            return

        # Flag this frame as processed, so that it's not reused in case of lag
        self.new_frame[index] = False
        # Get image in HSV (hue, saturation, value) colour format
        im_hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)

        # Specify target colours
        # e.g. Blue (Note: BGR)
        # Convert this colour to HSV colour model
        #first version
        #1
        blue_rgb = np.uint8([[[255, 0, 0]]])
        blue_hsv = cv2.cvtColor(blue_rgb, cv2.COLOR_RGB2HSV)
        #2
        green_rgb = np.uint8([[[0, 255, 0]]])
        green_hsv = cv2.cvtColor(green_rgb, cv2.COLOR_RGB2HSV)
        #3
        orange_rgb = np.uint8([[[0, 84, 192]]])
        orange_hsv = cv2.cvtColor(orange_rgb, cv2.COLOR_RGB2HSV)

        rgb_colours = [blue_hsv, green_hsv, orange_hsv]

        # Extract colour boundaries for masking image
        # Get the hue value from the numpy array containing target colours
        hsv_end_table = np.empty([3,2])
        Data_type = object
        hsv_np_array = np.array(hsv_end_table, dtype=Data_type)
        colour_no = 0
        for colour in rgb_colours:
            target_hue = colour[0, 0][0]
            hsv_np_array[colour_no][0] = np.array([target_hue - 20, 70, 70])
            hsv_np_array[colour_no][1] = np.array([target_hue + 20, 255, 255])

        # Generate the mask based on the desired hue range
        # Extract colour boundaries for masking image
        # Get the hue value from the numpy array containing target colour

        #version 1
        #blue_mask = cv2.inRange(im_hsv, blue_hsv_lo_end, blue_hsv_hi_end)
        #green_mask = cv2.inRange(im_hsv, green_hsv_lo_end, green_hsv_hi_end)
        #orange_mask = cv2.inRange(im_hsv, orange_hsv_lo_end, orange_hsv_hi_end)

        #version 2
        #blue_mask = cv2.inRange(im_hsv, (95,106,50), (130,255,255))
        #green_mask = cv2.inRange(im_hsv, (45,100,20), (65,150,204))
        #orange_mask = cv2.inRange(im_hsv, (10,100,20), (25,200,255))

        #version 3
        orange_mask = cv2.inRange(im_hsv, (95,100,20), (130,150,204))
        green_mask = cv2.inRange(im_hsv, (45,100,20), (65,150,204))
        blue_mask = cv2.inRange(im_hsv, (10,100,20), (25,150,204))

        # Check if a given colour is present
        if cv2.countNonZero(blue_mask) > 0:
            print('blue is present!')

        if cv2.countNonZero(green_mask) > 0:
            print('green is present!')

        if cv2.countNonZero(orange_mask) > 0:
            print('orange is present!')


    def __init__(self):
        # Initialise a new ROS node to communicate with MiRo
        rospy.init_node("colour_detection", anonymous=True)
        # Give it some time to make sure everything is initialised
        rospy.sleep(2.0)
        # Initialise CV Bridge
        self.image_converter = CvBridge()
        # Individual robot name acts as ROS topic prefix
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
        # Create two new subscribers to recieve camera images with attached callbacks
        self.sub_caml = rospy.Subscriber(
            topic_base_name + "/sensors/caml/compressed",
            CompressedImage,
            self.callback_caml,
            queue_size=1,
            tcp_nodelay=True,
        )
        self.sub_camr = rospy.Subscriber(
            topic_base_name + "/sensors/camr/compressed",
            CompressedImage,
            self.callback_camr,
            queue_size=1,
            tcp_nodelay=True,
        )
        # Create handle to store images
        self.input_camera = [None, None]
        # New frame notification
        self.new_frame = [False, False]
        # Set the default frame width (gets updated on reciecing an image)
        self.frame_width = 640
        # Action selector to reduce duplicate printing to the terminal
        self.just_switched = True
        # Bookmark
        self.bookmark = 0

    def loop(self):
        """
        Main control loop
        """
        print("loop starts")
        while not rospy.core.is_shutdown():

            for index in range(2):  # For each camera (0 = left, 1 = right)
                # Skip if there's no new image, in case the network is choking
                if not self.new_frame[index]:
                    continue
                image = self.input_camera[index]
                # Run the detect ball procedure
                self.detect_colour(image, index)

            # Yield
            rospy.sleep(self.TICK)


# This condition fires when the script is called directly
if __name__ == "__main__":
    main = MiRoClient()  # Instantiate class
    main.loop()  # Run the main control loop