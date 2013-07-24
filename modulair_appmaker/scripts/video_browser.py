#!/usr/bin/env python

import cv
import numpy
import rospy

from PySide.QtCore import QTimer
from PySide.QtOpenGL import QGLWidget
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
import Quad

class GLWidget(QGLWidget):

	def __init__(self, width, height, parent=None):
		QGLWidget.__init__(self, parent)

		self.video_width = width
		self.video_height = height

		self.capture = cv.CaptureFromFile("/home/kel/modulair/modulair_appmaker/scripts/vids/random.avi")
		cv.SetCaptureProperty(self.capture, cv.CV_CAP_PROP_FRAME_WIDTH, self.video_width)
		cv.SetCaptureProperty(self.capture, cv.CV_CAP_PROP_FRAME_HEIGHT, self.video_height)
		self.fps = int(cv.GetCaptureProperty(self.capture, cv.CV_CAP_PROP_FPS))

		self.quad = Quad.Rectangle()
		self.timer = QTimer()
		self.timer.timeout.connect(self.send_next_frame)
		self.timer.start(self.fps)
		pass

	def initializeGL(self):
		glClearColor(0.0, 0.0, 0.0, 0.0)
		pass

	def paintGL(self):
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
		glEnable(GL_TEXTURE_2D)
		glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)

		glMatrixMode(GL_PROJECTION)
		glLoadIdentity()
		gluOrtho2D(0, self.video_width, 0, self.video_height)

		glMatrixMode(GL_MODELVIEW)
		glLoadIdentity()

		glBegin(GL_QUADS)
		glTexCoord2f(0.0, 0.0)
		glVertex2f(0.0, 0.0)
		glTexCoord2f(1.0, 0.0)
		glVertex2f(self.video_width, 0.0)
		glTexCoord2f(1.0, 1.0)
		glVertex2f(self.video_width, self.video_height)
		glTexCoord2f(0.0, 1.0)
		glVertex2f(0.0, self.video_height)
		glEnd()

		glFlush()
		pass

	def resizeGL(self, width, height):
		if height == 0:
			height = 1

		glViewport(0, 0, width, height)
		glMatrixMode(GL_PROJECTION)
		glLoadIdentity()
		if width <= height:
			glOrtho(-1.0, 1.0, -1.0 * height / width, 1.0 * height / width, -1.0, 1.0)
		else:
			glOrtho(-1.0 * width / height, 1.0 * width / height, -1.0, 1.0, -1.0, 1.0)
		glMatrixMode(GL_MODELVIEW)
		glLoadIdentity()
		pass

	def send_next_frame(self):
		image = cv.QueryFrame(self.capture)
		if image == None:
			# Stop the video
			# self.timer.stop()
			# rospy.logwarn("VIDEO FINISHED")
			
			# Put the video on replay:
			self.capture = cv.CaptureFromFile("/home/kel/modulair/modulair_appmaker/scripts/vids/random.avi")
			return
		image_size = cv.GetSize(image)
		cv.Flip(image, None, 0)
		# cv.CvtColor(image, image, cv.CV_BGR2RGB)

		image_arr = self.cv2array(image)

		glTexImage2D(
			GL_TEXTURE_2D, 
			0, 
			GL_RGB, 
			image_size[0], 
			image_size[1], 
			0, 
			GL_BGR, 
			GL_UNSIGNED_BYTE, 
			image_arr
		)
		
		self.updateGL()
		pass

	def cv2array(self, im):
		depth2dtype = { 
			cv.IPL_DEPTH_8U: 'uint8', 
			cv.IPL_DEPTH_8S: 'int8', 
			cv.IPL_DEPTH_16U: 'uint16', 
			cv.IPL_DEPTH_16S: 'int16', 
			cv.IPL_DEPTH_32S: 'int32', 
			cv.IPL_DEPTH_32F: 'float32', 
			cv.IPL_DEPTH_64F: 'float64', 
		}

		arrdtype = im.depth
		a = numpy.fromstring(
			im.tostring(),
			dtype = depth2dtype[im.depth],
			count = im.width * im.height * im.nChannels)
		a.shape = (im.height, im.width, im.nChannels)
		return a 
		

	def clean_up(self):
		self.timer.stop()
		pass