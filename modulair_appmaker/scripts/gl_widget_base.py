#!/usr/bin/env python

### PySide Imports ###
from PySide import QtOpenGL
from PySide.QtCore import QTimer
### OpenGL Imports ###
from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *

class GLWidget(QtOpenGL.QGLWidget):

	def __init__(self, delay = 0, depth = False, parent = None):
		QtOpenGL.QGLWidget.__init__(self, parent)

		self.timer = QTimer()
		self.timer.timeout.connect(self.idle)
		self.timer.start(delay)
		self.depth = depth

		self.viewPortWidth = 0
		self.viewPortHeight = 0
		self.aspect = 0
		pass

	def initializeGL(self):
		glEnable(GL_TEXTURE_2D)
		glClearColor(0.7, 0.7, 0.7, 0.7)
		glClearDepth(1.0)
		glDepthFunc(GL_LESS)
		# glShadeModel(GL_SMOOTH)
		# glEnable(GL_DEPTH_TEST)
		# # glFrontFace(GL_CW)    
		# glCullFace(GL_BACK) 
		glEnable(GL_CULL_FACE)
		glEnable(GL_LIGHTING)
		# glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE)
		# glEnable(GL_NORMALIZE)
		glEnable(GL_LIGHT0)
		self.initGL()
		pass

	def paintGL(self):
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
		self.paint()
		pass

	def resizeGL(self, width, height):
		# if height == 0:
		# 	height = 1

		# glViewport(0,0,width,height)
		# self.viewPortWidth = width
		# self.viewPortHeight = height

		# self.aspect = self.viewPortWidth/self.viewPortHeight
		# glMatrixMode(GL_PROJECTION)
		# glLoadIdentity()  

		# self.resize(width, height)
		
		# glMatrixMode(GL_MODELVIEW) 
		# glLoadIdentity()
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
		pass

	def clean_up(self):
		self.timer.stop()
		pass

	def enableGlut(self):
		glutInit()
		pass