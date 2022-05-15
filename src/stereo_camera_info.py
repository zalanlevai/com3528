#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CameraInfo, Image

from stereo_camera import left_camera_matrix, left_distortion, right_camera_matrix, right_distortion, size, R1, R2, P1, P2

def sync_publish(pub, camera_info):
    def fn(msg):
        camera_info.header = msg.header
        pub.publish(camera_info)
    return fn

def main():
    rospy.init_node("stereo_camera_info")

    pub_l = rospy.Publisher("left/camera_info", CameraInfo, queue_size = 1)
    camera_info_l = CameraInfo(
        height = size[1],
        width = size[0],
        distortion_model = "plumb_bob",
        D = left_distortion.tolist(),
        K = left_camera_matrix.flatten().tolist(),
        R = R1.flatten().tolist(),
        P = P1.flatten().tolist(),
    )
    sub_l = rospy.Subscriber("left/image_raw", Image, sync_publish(pub_l, camera_info_l))

    pub_r = rospy.Publisher("right/camera_info", CameraInfo, queue_size = 1)
    camera_info_r = CameraInfo(
        height = size[1],
        width = size[0],
        distortion_model = "plumb_bob",
        D = right_distortion.tolist(),
        K = right_camera_matrix.flatten().tolist(),
        R = R2.flatten().tolist(),
        P = P2.flatten().tolist(),
    )
    sub_r = rospy.Subscriber("right/image_raw", Image, sync_publish(pub_r, camera_info_r))

    rospy.spin()

    sub_l.unregister()
    sub_r.unregister()
    pub_l.unregister()
    pub_r.unregister()

if __name__ == "__main__":
    main()
