#!/usr/bin/env python3

import ctypes
from math import inf
import struct

import cv2
import numpy as np

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Header

from stereo_camera import Q, left_map1, left_map2, right_map1, right_map2

def stereo_depth(frame_l, frame_r):
    rect_l = cv2.remap(frame_l, left_map1, left_map2, cv2.INTER_LINEAR)
    rect_r = cv2.remap(frame_r, right_map1, right_map2, cv2.INTER_LINEAR)

    img_l = cv2.cvtColor(rect_l, cv2.COLOR_BGR2GRAY)
    img_r = cv2.cvtColor(rect_r, cv2.COLOR_BGR2GRAY)

    stereo = cv2.StereoBM_create(numDisparities = 16, blockSize = 5)
    disparity = stereo.compute(img_l, img_r)

    disparity_norm = cv2.normalize(disparity, disparity, alpha = 5, beta = 255, norm_type = cv2.NORM_MINMAX, dtype = cv2.CV_8U)

    projection = cv2.reprojectImageTo3D(disparity.astype(np.float32) / 16.0, Q)

    return disparity_norm, projection

cv_bridge = CvBridge()

def opencv_frame(msg):
    try:
        return cv_bridge.imgmsg_to_cv2(msg, desired_encoding = "bgr8")
    except CvBridgeError as error:
        rospy.logerr(error)

def main():
    rospy.init_node("stereo_camera_depth")

    pub = rospy.Publisher("depth", PointCloud2, queue_size = 1)

    secondary_img_msg = None
    def handle_secondary_image(msg):
        nonlocal secondary_img_msg
        secondary_img_msg = msg

    def handle_primary_image(msg):
        while True:
            if rospy.is_shutdown(): return
            if secondary_img_msg is None: return
            # FIXME: if msg.header.seq != secondary_img_msg.header.seq: continue
            break

        frame_l = opencv_frame(msg)
        frame_r = opencv_frame(secondary_img_msg)

        _disparity, projection = stereo_depth(frame_l, frame_r)

        point_struct = struct.Struct("<fff")
        buff = ctypes.create_string_buffer(point_struct.size * projection.shape[0] * projection.shape[1])

        offset = 0
        for v in range(projection.shape[0]):
            for u in range(projection.shape[1]):
                x, y, z = projection[v, u]
                if z == 10000.0 or z == inf: continue

                point_struct.pack_into(buff, offset, *(x, y, z))
                offset += point_struct.size

        pub.publish(PointCloud2(
            header = Header(
                stamp = rospy.Time.now(),
                frame_id = "head_caml_optical"
            ),
            height = projection.shape[0],
            width = projection.shape[1],
            fields = [
                PointField(name = "x", offset = 0, datatype = PointField.FLOAT32, count = 1),
                PointField(name = "y", offset = 4, datatype = PointField.FLOAT32, count = 1),
                PointField(name = "z", offset = 8, datatype = PointField.FLOAT32, count = 1)
            ],
            is_bigendian = False,
            is_dense = False,
            point_step = 12,
            row_step = 12 * projection.shape[1],
            data = buff.raw
        ))

    sub_l = rospy.Subscriber("left/image_raw", Image, handle_primary_image, queue_size = 1)
    sub_r = rospy.Subscriber("right/image_raw", Image, handle_secondary_image, queue_size = 1)

    rospy.spin()

    sub_l.unregister()
    sub_r.unregister()
    pub.unregister()

if __name__ == "__main__":
    main()
