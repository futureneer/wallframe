#!/usr/bin/env python
import sys
import os
import re
import math

from PySide import QtCore, QtGui, QtOpenGL
from PySide.QtCore import QTimer
from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
from Image import *
import rospy

TYPES = set(['Cube','Triangle','Rectangle'])

# class Cube(Quad):	
# 	def __init__(self, *args):
# 		super(Cube, self).__init__()
# 		if len(args) !=4:
# 			print("3 Input arguments(L,W,H) required, only " + str(len(args)) + " given")
# 			sys.exit(0)				
	

class Object(object):	
	def __init__(self):
		super(Object, self).__init__()
		pass

	def makeCommandList(self):
		pass
		
	def initializeQuad(self, *args):
		genList = glGenLists(1)
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
		glLoadIdentity()
		glTranslate(self.cord[0], self.cord[1], -5.0)

	def loadTexture(self, fileName):
		pass

	def draw(self):
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
		glLoadIdentity()
		glCallList(self.obj)

	def update(self):
		self.object = self.makeCommandList()
		self.draw()
		pass

class Triangle(object):
	def __init__(self):
		super(Triangle, self).__init__()
		self.cord = []
		self.dim = []
		self.textureNeeded = 0
		self.texture = 0
		self.obj = 0		
		pass

	def makeCommandList(self):
		genList = glGenLists(1)
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
		glLoadIdentity()
		glTranslate(self.cord[0], self.cord[1], -3)

		glBegin(GL_TRIANGLES)

		if self.textureNeeded:
			glTexCoord2f(0.0, 0.0)
		glVertex3f(-1.0, -1.0, 1.0)

		if self.textureNeeded:
			glTexCoord2f(1.0, 0.0)
		glVertex3f(1.0, -1.0, 1.0)

		if self.textureNeeded:
			glTexCoord2f(1.0, 1.0)
		glVertex3f(1.0, 1.0, 1.0)

		glEnd()

		glEndList()

		return genList

	def initializeQuad(self, *args):
		if len(args) != 4 :
			print("\nERROR :: 4 Input arguments(x,y,l,w) required, only " + str(len(args)) + " given")
			sys.exit(0)
		self.cord = [args[0],args[1]]		
		self.dim = [args[2],args[3]]
		self.obj = self.makeCommandList()		
		pass

	def loadTexture(self,fileName):
		pass

	def draw(self):
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
		glLoadIdentity()
		glCallList(self.obj)
		pass

	def update(self):
		self.object = self.makeCommandList()
		self.draw()
		pass

class Rectangle(object):
	def __init__(self):
		super(Rectangle, self).__init__()		
		self.cord = []
		self.dim = []
		self.textureNeeded = 0
		self.textureIndex = 0
		self.textures = []
		self.obj = 0		
		pass

	def makeCommandList(self):
		genList = glGenLists(1)	
		glNewList(genList,GL_COMPILE)

		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
		glLoadIdentity()
		glTranslate(self.cord[0]-0.5, self.cord[1], -3)

		if self.textureNeeded:
			glBindTexture(GL_TEXTURE_2D, self.textures[self.textureIndex])

		glBegin(GL_QUADS)

		if self.textureNeeded:
			glTexCoord2f(0.0, 0.0)
		glVertex3f(-1.0, -1.0, 1.0)

		if self.textureNeeded:
			glTexCoord2f(1.0, 0.0)
		glVertex3f(2, -1.0, 1.0)

		if self.textureNeeded:
			glTexCoord2f(1.0, 1.0)
		glVertex3f(2, 1.0, 1.0)

		if self.textureNeeded:
			glTexCoord2f(0.0, 1.0)
		glVertex3f(-1.0, 1.0, 1.0)

		glEnd()
		glEndList()

		return genList		

	def intializeQuad(self,*args):		
		if len(args) != 4 :
			print("\nERROR :: 4 Input arguments(x,y,l,w) required, only " + str(len(args)) + " given")
			sys.exit(0)
		self.cord = [args[0],args[1]]		
		self.dim = [args[2],args[3]]
		self.obj = self.makeCommandList()		
		pass

	# def loadTextures(self, file_path):
	# 	self.textureNeeded = 1
	# 	self.image = open(fileName)
	# 	self.ix = self.image.size[0]
	# 	self.iy = self.image.size[1]
	# 	self.image = self.image.tostring("raw", "RGBX", 0, -1)

	# 	# Create Texture
	# 	glGenTextures(1, self.texture)
	# 	glBindTexture(GL_TEXTURE_2D, self.texture)
	# 	glPixelStorei(GL_UNPACK_ALIGNMENT, 1)
	# 	glTexImage2D(GL_TEXTURE_2D, 0, 3 , self.ix, self.iy, 0, GL_RGBA, GL_UNSIGNED_BYTE, self.image)
	# 	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)				
	# 	pass

	def loadTextures(self, images, raw_images):
		self.textureNeeded = 1
		for tex in range(0, len(images)):
			img_x = images[tex].size[0]
			img_y = images[tex].size[1]			
			self.textures.append(glGenTextures(1))
			glBindTexture(GL_TEXTURE_2D, self.textures[tex])

			glPixelStorei(GL_UNPACK_ALIGNMENT, 1)
			glTexImage2D(GL_TEXTURE_2D, 0, 3, img_x, img_y, 0, GL_RGBA, GL_UNSIGNED_BYTE, raw_images[tex])
			glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)
		pass

	def draw(self):
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
		glLoadIdentity()
		glCallList(self.obj)
		pass

	def update(self):	
		self.obj = self.makeCommandList()
		self.draw()
		pass

	def set_texture_index(self, index):
		# rospy.logwarn("NEW INDEX: " + str(index))
		self.textureIndex = index
		pass

	def get_texture_index(self):
		return self.textureIndex
		pass