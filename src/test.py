#!/usr/bin/env python3
"""
This script makes MiRo look for a blue ball and kick it

The code was tested for Python 2 and 3
For Python 2 you might need to change the shebang line to
#!/usr/bin/env python
"""
# Imports
##########################
import os
from math import radians  # This is used to reset the head pose
import numpy as np  # Numerical Analysis library
import cv2  # Computer Vision library
import math

import rospy  # ROS Python interface
from sensor_msgs.msg import CompressedImage  # ROS CompressedImage message
from sensor_msgs.msg import JointState  # ROS joints state message
from nav_msgs.msg import Odometry 
from cv_bridge import CvBridge, CvBridgeError  # ROS -> OpenCV converter
from geometry_msgs.msg import TwistStamped  # ROS cmd_vel (velocity control) message
from tf.transformations import euler_from_quaternion


import miro2 as miro  # Import MiRo Developer Kit library

try:  # For convenience, import this util separately
    from miro2.lib import wheel_speed2cmd_vel  # Python 3
except ImportError:
    from miro2.utils import wheel_speed2cmd_vel  # Python 2
##########################


class MiRoClient(object):
    """
    Script settings below
    """
    def reset_head_pose(self):
        """
        Reset MiRo head to default position, to avoid having to deal with tilted frames
        """
        self.kin_joints = JointState()  # Prepare the empty message
        self.kin_joints.name = ["tilt", "lift", "yaw", "pitch"]
        self.kin_joints.position = [0.0, radians(34.0), 0.0, 0.0]
        t = 0
        while not rospy.core.is_shutdown():  # Check ROS is running
            # Publish state to neck servos for 1 sec
            self.pub_kin.publish(self.kin_joints)
            rospy.sleep(1)
            t += 1
            if t > 1:
                break

    def drive(self, speed_l=0.1, speed_r=0.1):  # (m/sec, m/sec)
        """
        Wrapper to simplify driving MiRo by converting wheel speeds to cmd_vel
        """
        # Prepare an empty velocity command message
        msg_cmd_vel = TwistStamped()

        # Desired wheel speed (m/sec)
        wheel_speed = [speed_l, speed_r]

        # Convert wheel speed to command velocity (m/sec, Rad/sec)
        (dr, dtheta) = wheel_speed2cmd_vel(wheel_speed)

        # Update the message with the desired speed
        msg_cmd_vel.twist.linear.x = dr
        msg_cmd_vel.twist.angular.z = dtheta

        # Publish message to control/cmd_vel topic
        self.vel_pub.publish(msg_cmd_vel)

   
#moving from starting position
    def move(self):
        rospy.sleep(0.1)  # Print once
        print("MiRo is looking for the ball...")
        self.drive(-2, 2)
        rospy.sleep(0.2)
        print ("moving")

    def scan_callback(self,scan_data):
        #print(scan_data.pose.pose.position)

        orientation = scan_data.pose.pose.orientation
        ranges = scan_data.pose.pose.position

        (roll, pitch, yaw) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w], 'sxyz')
        print(yaw)

        self.dead_left90 = ranges[90]
        self.dead_right90 = ranges[-90]
        self.dead_front = range[0]

        left_arc45_55 = ranges[30:40]
        left_arc55_60 = ranges[40:50]
        left_arc45_60 = np.array(left_arc55_60[::-1] + left_arc45_55[::-1])
        self.left_arc =left_arc45_60.min()

        
        right_arc45_55 = ranges[-40:-30]
        right_arc55_60 = ranges[-50:-40]
        right_arc45_60 = np.array(right_arc55_60[::-1] + right_arc45_55[::-1])
        self.right_arc = right_arc45_60.min()
        


#90 turn right
    def right90(self):
        if self.dead_right90 < 0.5:
            self.right_status = True
            print ("tilted towards right")
        else:
            self.right_status = False
            print ("straight")

#90 turn left
    def left90(self):
        if self.dead_left90 < 0.5:
            self.left_status = True
            print ("tilted towards left")
        else:
            self.left_status = False
            print ("straight")
    
#stay forward
    def front(self):
        if self.dead_front < 0.45:
            self.front_status = True
            print ("started titlting")
        else:
            self.front_status = False
            print ("straight")


    def turn_right90(self):
        print ("turn  right 90")
        self.drive(0.0,-0.62)
        rospy.sleep(2.5)
        print ("adjust to the right")
        while self.dead_left90 > 0.35:
            self.drive(0.0,0.3)
        self.drive(0.0,0.0)
        print ("stop")
        rospy.sleep(2)

    def turn_left90(self):
        print ("turn left 90")
        self.drive(0.0,0.62)
        rospy.sleep(2.5)
        print ("adjust to the left")
        while self.dead_right90 > 0.35:
            self.drive(0.0,0.3)
        self.drive(0.0,0.0)
        print ("stop")
        rospy.sleep(2)

    def avoid_junction(self):
        if self.left_arc < 0.2:
            self.drive(0.0,-0.3)
            print ("adjust turn right")
        elif self.right_arc < 0.2:
            self.drive(0.0, 0.3)
            print ("adjust turn right")
        else:
            self.drive(0.2,0.0)

    def __init__(self):
        # Initialise a new ROS node to communicate with MiRo
        rospy.init_node("test", anonymous=True)
        # Give it some time to make sure everything is initialised
        rospy.sleep(2.0)
    
        # Individual robot name acts as ROS topic prefix
        topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")
    
        
        # Create a new publisher to send velocity commands to the robot
        self.vel_pub = rospy.Publisher(
            topic_base_name + "/control/cmd_vel", TwistStamped, queue_size=0
        )
        # Create a new publisher to move the robot head
        self.pub_kin = rospy.Publisher(
            topic_base_name + "/control/kinematic_joints", JointState, queue_size=0
        )

         # Create a new publisher to move the robot head
        self.pub_odom = rospy.Publisher(
            topic_base_name + "/sensors/odom", JointState, queue_size=0
        )

        self.sub_odom = rospy.Subscriber(
            topic_base_name +  "/sensors/odom",
            Odometry,
            self.scan_callback
        )
       
        # Move the head to default pose
        #self.reset_head_pose()
        #current angle

        self.dead_left90 = 0
        self.dead_right90 = 0
        self.dead_front = 0
        self.right_status = False
        self.left_status = False
        self.front_status = True

    def loop(self):
        """
        Main control loop
        """
        self.ctrl_c = False
        #self.move()
        while not rospy.core.is_shutdown():
            self.move()
            
            self.left90()
            self.right90()
            self.front()
            if self.left_status == True and self.front_status == True:
                print ("1")
                self.drive(0.0,0.0)
                print ("stop")
                rospy.sleep(0.1)
                self.turn_right90()

            else: 
                self.avoid_wall()



# This condition fires when the script is called directly
if __name__ == "__main__":
    main = MiRoClient()
    try:
        main.loop()
    except rospy.ROSInterruptException:
        pass
    #main = MiRoClient()  # Instantiate class
    #main.loop()  # Run the main control loop
