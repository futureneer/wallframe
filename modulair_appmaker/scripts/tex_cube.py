#!/usr/bin/env python 

### Python Imports ###
import math
### PySide Imports ###
from PySide.QtCore import QTimer
from PySide.QtOpenGL import QGLWidget 
### PyOpenGL Imports ###
from OpenGL.GL import *
from OpenGL.GLU import *
### Utility Imports ###
from PIL import Image

class TCube(QGLWidget):

	def __init__(self):
		QGLWidget.__init__(self, None)

		self.x_rot = 0
		self.y_rot = 0
		self.z_rot = 0
		self.image_x = 0
		self.image_y = 0
		self.texture = 0

		self.timer = QTimer()
		self.timer.timeout.connect(self.updateGL)
		self.timer.start(0)
		pass

	def load_texture(self):
		self.image = Image.open("reef.jpg")
		self.image.show()
		self.image_x = self.image.size[0]
		self.image_y = self.image.size[1]
		self.image = self.image.tostring("raw", "RGBX", 0, -1)

		glGenTextures(1, self.texture)
		glBindTexture(GL_TEXTURE_2D, self.texture)

		glPixelStorei(GL_UNPACK_ALIGNMENT, 1)
		glTexImage2D(GL_TEXTURE_2D, 0, 3, self.image_x, self.image_y, 0, GL_RGBA, GL_UNSIGNED_BYTE, self.image)
		glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP)
		glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP)
		glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT)
		glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT)
		glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST)
		glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)
		glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL)
		pass

	def initializeGL(self):
		self.load_texture()
		glEnable(GL_TEXTURE_2D)
		glClearColor(0.0, 0.0, 0.0, 0.0)
		glClearDepth(1.0)
		glDepthFunc(GL_LESS)
		glShadeModel(GL_SMOOTH)
		glEnable(GL_DEPTH_TEST)
		glEnable(GL_CULL_FACE)
		pass

	def paintGL(self):

		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
		glLoadIdentity()
		glTranslatef(0.0, 0.0, -5.0)

		glRotatef(self.x_rot, 1.0, 0.0, 0.0)
		glRotatef(self.y_rot, 0.0, 1.0, 0.0)
		glRotatef(self.z_rot, 0.0, 0.0, 1.0)

		glBindTexture(GL_TEXTURE_2D, self.texture)

		glBegin(GL_QUADS)

		# Front Face
		glTexCoord2f(0.0, 0.0)
		glVertex3f(-1.0, -1.0, 1.0)
		glTexCoord2f(1.0, 0.0)
		glVertex3f(1.0, -1.0, 1.0)
		glTexCoord2f(1.0, 1.0)
		glVertex3f(1.0, 1.0, 1.0)
		glTexCoord2f(0.0, 1.0)
		glVertex3f(-1.0, 1.0, 1.0)

		# Back Face
		glTexCoord2f(1.0, 0.0)
		glVertex3f(-1.0, -1.0, -1.0)
		glTexCoord2f(1.0, 1.0)
		glVertex3f(-1.0, 1.0, -1.0)
		glTexCoord2f(0.0, 1.0)
		glVertex3f(1.0, 1.0, -1.0)
		glTexCoord2f(0.0, 0.0)
		glVertex3f(1.0, -1.0, -1.0)

		# Top Face
		glTexCoord2f(0.0, 1.0)
		glVertex3f(-1.0, 1.0, -1.0)
		glTexCoord2f(0.0, 0.0)
		glVertex3f(-1.0, 1.0, 1.0)
		glTexCoord2f(1.0, 0.0)
		glVertex3f(1.0, 1.0, 1.0)
		glTexCoord2f(1.0, 1.0)
		glVertex3f(1.0, 1.0, -1.0)

		# Bottom Face
		glTexCoord2f(1.0, 1.0)
		glVertex3f(-1.0, -1.0, -1.0)
		glTexCoord2f(0.0, 1.0)
		glVertex3f(1.0, -1.0, -1.0)
		glTexCoord2f(0.0, 0.0)
		glVertex3f(1.0, -1.0, 1.0)
		glTexCoord2f(1.0, 0.0)
		glVertex3f(-1.0, -1.0, 1.0)

		# Right Face
		glTexCoord2f(1.0, 0.0)
		glVertex3f(1.0, -1.0, -1.0)
		glTexCoord2f(1.0, 1.0)
		glVertex3f(1.0, 1.0, -1.0)
		glTexCoord2f(0.0, 1.0)
		glVertex3f(1.0, 1.0, 1.0)
		glTexCoord2f(0.0, 0.0)
		glVertex3f(1.0, -1.0, 1.0)

		# Left Face
		glTexCoord2f(0.0, 0.0)
		glVertex3f(-1.0, -1.0, -1.0)
		glTexCoord2f(1.0, 0.0)
		glVertex3f(-1.0, -1.0, 1.0)
		glTexCoord2f(1.0, 1.0)
		glVertex3f(-1.0, 1.0, 1.0)
		glTexCoord2f(0.0, 1.0)
		glVertex3f(-1.0, 1.0, -1.0)

		glEnd()

		self.x_rot += 0.2
		self.y_rot += 0.2
		self.z_rot += 0.2
		pass

	def resizeGL(self, width, height):
		if height == 0:
			height = 1

		glViewport(0, 0, width, height)
		glMatrixMode(GL_PROJECTION)
		glLoadIdentity()
		gluPerspective(45.0, float(width) / float(height), 0.1, 100.0)
		glMatrixMode(GL_MODELVIEW)

	def clean_up(self):
		self.timer.stop()
		pass