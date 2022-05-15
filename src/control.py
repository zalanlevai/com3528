#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, TwistStamped, Vector3
from sensor_msgs.msg import JointState, Range

import miro2.constants as MIRO
from miro2.lib.miro_utils import wheel_speed2cmd_vel

def vel(linear = 0, angular = 0):
    return TwistStamped(
        twist = Twist(
            linear = Vector3(x = linear, y = 0, z = 0),
            angular = Vector3(x = 0, y = 0, z = angular)
        )
    )

def miro_vel(linear = 0, angular = 0):
    return vel(linear * MIRO.WHEEL_MAX_SPEED_M_PER_S, angular * MIRO.WHEEL_MAX_ANG_SPEED_RAD_PER_S)

def drive(speed_l = 0, speed_r = 0):
    (dr, dtheta) = wheel_speed2cmd_vel((speed_l, speed_r))
    return vel(linear = dr, angular = dtheta)

def joints_pos(tilt = MIRO.TILT_RAD_CALIB, lift = MIRO.LIFT_RAD_CALIB, yaw = MIRO.YAW_RAD_CALIB, pitch = MIRO.PITCH_RAD_CALIB):
    return JointState(
        name = ["tilt", "lift", "yaw", "pitch"],
        position = [tilt, lift, yaw, pitch]
    )

def wait(duration):
    rospy.rostime.wallsleep(duration.to_sec())

def move_for(duration, pub, vel):
    t_0 = rospy.get_rostime()
    while not rospy.is_shutdown():
        t = rospy.get_rostime()
        if t - t_0 > duration: break
        pub.publish(vel)

def enter_pose(duration, pub, joints):
    pub.publish(joints)
    # wait for joints to reach target state
    wait(duration)

def read_range(msg):
    is_valid = msg.range < msg.max_range and msg.range > msg.min_range
    range = max(min(msg.range, msg.max_range), msg.min_range)
    return range, is_valid

def main():
    rospy.init_node("control")

    pub_vel = rospy.Publisher("ctrl_vel", TwistStamped, queue_size = 1)
    pub_joints = rospy.Publisher("ctrl_joints", JointState, queue_size = 1)

    while not rospy.is_shutdown():
        move_for(rospy.Duration(secs = 2), pub_vel, miro_vel(linear = 1))

        enter_pose(rospy.Duration(secs = 1), pub_joints, joints_pos(yaw = MIRO.YAW_RAD_MIN, lift = MIRO.LIFT_RAD_MAX))
        range, is_valid = read_range(rospy.wait_for_message("sonar", Range))
        rospy.loginfo("right: {}".format("invalid" if not is_valid else "{} m".format(range)))

        if not is_valid:
            move_for(rospy.Duration(secs = 0.5), pub_vel, miro_vel(linear = -0.2, angular = 0.1))

        enter_pose(rospy.Duration(secs = 1), pub_joints, joints_pos(yaw = MIRO.YAW_RAD_MAX, lift = MIRO.LIFT_RAD_MAX))
        range, is_valid = read_range(rospy.wait_for_message("sonar", Range))
        rospy.loginfo("left: {}".format("invalid" if not is_valid else "{} m".format(range)))

        if not is_valid:
            move_for(rospy.Duration(secs = 0.5), pub_vel, miro_vel(linear = -0.2, angular = -0.1))

        enter_pose(rospy.Duration(secs = 1), pub_joints, joints_pos(yaw = 0))
        range, is_valid = read_range(rospy.wait_for_message("sonar", Range))
        rospy.loginfo("ahead: {}".format("invalid" if not is_valid else "{} m".format(range)))

        if range < 0.1:
            rospy.loginfo("too close, panic")
            move_for(rospy.Duration(secs = 2), pub_vel, miro_vel(linear = -1, angular = -1))

    rospy.spin()

    pub_vel.unregister()
    pub_joints.unregister()

if __name__ == "__main__":
    main()
