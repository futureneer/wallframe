#!/usr/bin/env python

import os
import sys
import re
import rospy
import numpy
import cv
from PIL import Image
from PySide import QtGui, QtCore

### ROS/PySide Functions ###

def create_app(app_name, app_class, anonymous = True):	
	rospy.init_node(name, anonymous)
	
	app = QtGui.QApplication(sys.argv)
	
	app_canvas = app_class(app_name, app)

	return app, app_canvas

def log_info(msg):
	rospy.loginfo(msg)
	pass

def log_warn(msg):
	rospy.logwarn(msg)
	pass

def log_err(msg, *args):
	rospy.logerr(msg)
	pass

def create_signal():
	signal = QtCore.Signal()
	return signal

### Image Functions ###

default_img_file_formats = ['.jpg', '.png', '.jpeg', '.bmp']

def add_file_format(format):
	default_img_file_formats.append(format)
	pass

def find_image_files(path, formats = None):
	if formats == None:
		formats = default_img_file_formats

	files = os.listdir(path)

	file_regex = ''
	for ext in formats:
		file_regex += '.*' + ext + '|'

	file_locs = []
	for img in files:
		if re.match(file_regex, img):
			file_locs.append(path + img)

	return file_locs

def load_image_files(image_files):
	images = []
	for img in image_files:
		tmp = Image.open(img)
		images.append(tmp)

	return images

def to_string(image):
	return image.tostring('raw', 'RGBX', 0, -1)

### Video Functions ###

def load_video(path):
	return cv.CaptureFromFile(path)

def get_next_frame(video):
	image = cv.QueryFrame(video)
	cv.Flip(image, None, 0)
	# cv.CvtColor(image, image, cv.CV_BGR2RGB)
	image_arr = ipl2tex(image)

	return image_arr

def get_video_fps(video):
	return int(cv.GetCaptureProperty(video, cv.CV_CAP_PROP_FPS))

def ipl2tex(image):
	depth2dtype = { 
		cv.IPL_DEPTH_8U: 	'uint8', 
		cv.IPL_DEPTH_8S: 	'int8', 
		cv.IPL_DEPTH_16U: 	'uint16', 
		cv.IPL_DEPTH_16S: 	'int16', 
		cv.IPL_DEPTH_32S: 	'int32', 
		cv.IPL_DEPTH_32F: 	'float32', 
		cv.IPL_DEPTH_64F: 	'float64', 
	} 
	arr_dtype = image.depth
	tex = numpy.fromstring(
		image.tostring(),
		dtype = depth2dtype[arr_dtype],
		count = image.width * image.height * image.nChannels
		)
	tex.shape = (image.height, image.width, image.nChannels)
	return tex

