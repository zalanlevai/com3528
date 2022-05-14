#!/usr/bin/python3
#
#dile showing the view of miro camera
#	made using miro MDK example files


import rospy
from sensor_msgs.msg import CompressedImage

import time
import sys
import os
import numpy as np

import cv2
from cv_bridge import CvBridge, CvBridgeError

class client:

	def callback_cam(self, ros_image, index):

		# silently (ish) handle corrupted JPEG frames
		try:

			image = self.image_converter.compressed_imgmsg_to_cv2(ros_image, "rgb8")
            # Convert from OpenCV's default BGR to RGB
			image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            # Store image as class attribute for further use
			self.input_camera[index] = image
            # Get image dimensions
			self.frame_height, self.frame_width, channels = image.shape
			self.x_centre = self.frame_width / 2.0
			self.y_centre = self.frame_height / 2.0


		except CvBridgeError as e:

			# swallow error, silently
			#print(e)
			pass

	def callback_caml(self, ros_image):

		self.callback_cam(ros_image, 0)

	def callback_camr(self, ros_image):

		self.callback_cam(ros_image, 1)


	def loop(self):

		# state
		channels_to_process = [0, 1]
		if 1 == 1:
			channels_to_process = [2]
		outfile = [None, None, None]
		cam_names = ['left', 'right', 'stitched']

		# loop
		while not rospy.core.is_shutdown():

			## stitching ##
			if not self.input_camera[0] is None and not self.input_camera[1] is None:
					images = [self.input_camera[0], self.input_camera[1]]
					self.input_camera[2] = cv2.hconcat(images)
					self.input_camera[0] = None
					self.input_camera[1] = None

			# for each channel to process
			for index in channels_to_process:

				# get image
				image = self.input_camera[index]

				# if present
				if not image is None:

					# handle
					self.input_camera[index] = None

					# show
					cv2.imshow("client_video: " + cam_names[index], image)
					cv2.waitKey(1)

			# state
			time.sleep(0.02)

		# for each camera
		for index in range(len(outfile)):

			# if open, release
			if not outfile[index] is None:
				outfile[index].release()

	def __init__(self):

		# state
		self.input_camera = [None, None, None]

		# ROS -> OpenCV converter
		self.image_converter = CvBridge()

		# robot name
		topic_base_name = "/" + os.getenv("MIRO_ROBOT_NAME")

		# subscribe
		self.sub_caml = rospy.Subscriber(topic_base_name + "/sensors/caml/compressed",
					CompressedImage, self.callback_caml, queue_size=1, tcp_nodelay=True)
		self.sub_camr = rospy.Subscriber(topic_base_name + "/sensors/camr/compressed",
					CompressedImage, self.callback_camr, queue_size=1, tcp_nodelay=True)

		# report
		print ("recording from 2 cameras, press CTRL+C to halt...")

if __name__ == "__main__":

	rospy.init_node("client_video", anonymous=True)
	main = client()
	main.loop()
