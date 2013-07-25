#!/usr/bin/env python

### PySide Imports ###
from PySide import QtOpenGL
from PySide.QtCore import QTimer
### OpenGL Imports ###
from OpenGL.GL import *
from OpenGL.GLU import *

class GLWidget(QtOpenGL.QGLWidget):

	def __init__(self, delay = 0, fullscreen = True, parent = None):
		QtOpenGL.QGLWidget.__init__(self, parent)

		self.timer = QTimer()
		self.timer.timeout.connect(self.idle)
		self.timer.start(delay)
		self.fullscreen = fullscreen
		pass

	def initializeGL(self):
		glEnable(GL_TEXTURE_2D)
		glClearColor(0.0, 0.0, 0.0, 0.0)
		glClearDepth(1.0)
		glDepthFunc(GL_LESS)
		glShadeModel(GL_SMOOTH)
		glEnable(GL_DEPTH_TEST)     
		glEnable(GL_CULL_FACE)
		self.initGL()
		pass

	def paintGL(self):
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
		self.paint()
		pass

	def resizeGL(self, width, height):
		self.resize(width, height)
		pass

	def initGL(self):
		pass

	def paint(self):
		pass

	def idle(self):
		pass

	def update_widget(self):
		self.updateGL()
		pass

	def resize(self, width, height):
		if height == 0:
			height = 1

		glViewport(0, 0, width, height)
		glMatrixMode(GL_PROJECTION)
		glLoadIdentity()

		if self.fullscreen:
			if width <= height:
				glOrtho(-1.0, 1.0, -1.0 * height / width, 1.0 * height / width, -1.0, 1.0)
			else:
				glOrtho(-1.0 * height / width, 1.0 * height / width, -1.0, 1.0, -1.0, 1.0)
		else:
			gluPerspective(45.0, float(width) / float(height), 0.1, 100.0)

		glEnable(GL_DEPTH_TEST)
		glMatrixMode(GL_MODELVIEW)
		pass

	def clean_up(self):
		self.timer.stop()
		pass