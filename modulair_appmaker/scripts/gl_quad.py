#!/usr/bin/env python

### OpenGL Imports ###
from OpenGL.GL import *
from OpenGL.GLU import *

from gl_utils import *
import threading

class Rectangle(object):

	def __init__(self, x, y, width, height, image = None, img_type = None):
		super(Rectangle, self).__init__()

		self.x_ = x
		self.y_ = y
		self.width_ = width
		self.height_ = height

		self.texture = 0
		self.texture_needed = False

		if image != None and img_type != None:
			self.load_texture(image, img_type)

		self.obj = self.make_commandlist()
		pass

	def make_commandlist(self):
		gen_list = glGenLists(1)
		glNewList(gen_list, GL_COMPILE)

		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
		glEnable(GL_TEXTURE_2D)
		glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)

		glMatrixMode(GL_PROJECTION)
		glLoadIdentity()
		gluOrtho2D(0, self.width_, 0, self.height_)

		glMatrixMode(GL_MODELVIEW)
		glLoadIdentity()

		if self.texture_needed:
			glBindTexture(GL_TEXTURE_2D, self.texture)

		glBegin(GL_QUADS)

		if self.texture_needed:
			glTexCoord2f(0.0, 0.0)
		glVertex2f(0.0, 0.0)

		if self.texture_needed:
			glTexCoord2f(1.0, 0.0)
		glVertex2f(self.width_, 0.0)

		if self.texture_needed:
			glTexCoord2f(1.0, 1.0)
		glVertex2f(self.width_, self.height_)

		if self.texture_needed:
			glTexCoord2f(0.0, 1.0)
		glVertex2f(0.0, self.height_)

		glEnd()
		glEndList()

		return gen_list

	def load_texture(self, image, type):
		if type == 'PIL':
			self.set_image_texture(image)
		elif type == 'IPL':
			self.set_frame_texture(image)
		pass

	def set_image_texture(self, image):
		self.texture_needed = True
		img_x = image.size[0]
		img_y = image.size[1]

		raw_image = to_string(image)
		glGenTextures(1, self.texture)
		glBindTexture(GL_TEXTURE_2D, self.texture)

		glPixelStorei(GL_UNPACK_ALIGNMENT, 1)
		glTexImage2D(GL_TEXTURE_2D, 0, 3, img_x, img_y, 0, GL_RGBA, GL_UNSIGNED_BYTE, raw_image)
		self.update()
		pass

	def set_frame_texture(self, frame):
		self.texture_needed = True
		glGenTextures(1, self.texture)
		glBindTexture(GL_TEXTURE_2D, self.texture)
		
		glPixelStorei(GL_UNPACK_ALIGNMENT, 1)
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, frame.shape[1], frame.shape[0], 0, GL_BGR, GL_UNSIGNED_BYTE, frame)
		glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)
		self.update()
		pass

	def update(self):
		self.obj = self.make_commandlist()
		pass

	def draw(self):
		glLoadIdentity()
		glCallList(self.obj)
		pass