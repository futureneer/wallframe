#!/usr/bin/env python

import struct
import rospy

from PySide.QtCore import QTimer
from PySide.QtOpenGL import QGLWidget
from OpenGL.GL import *
from OpenGL.GLU import *

class GLWidget(QGLWidget):

	def __init__(self, parent=None):
		QGLWidget.__init__(self, parent)
		self.model1 = Loader()
		self.model1.load_stl("/home/kel/modulair/modulair_appmaker/scripts/models/cube.stl")
		# self.init_shading()
		pass

	def initializeGL(self):
		glShadeModel(GL_SMOOTH)
		glClearColor(1.0, 1.0, 1.0, 1.0)
		glClearDepth(1.0)
		glEnable(GL_DEPTH_TEST)
		glShadeModel(GL_SMOOTH) 
		glDepthFunc(GL_LEQUAL)
		glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST)


		glEnable(GL_COLOR_MATERIAL)

		glEnable(GL_LIGHTING)
		glEnable(GL_LIGHT0)   
		glLight(GL_LIGHT0, GL_POSITION,  (0, 1, 1, 0))

		glMatrixMode(GL_MODELVIEW)
		pass

	def paintGL(self):
		glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)
		glLoadIdentity()

		glTranslatef(0.0, -13.0, -50.0)
		self.model1.draw()
		pass

	def resizeGL(self, width, height):
		if height==0:
			height=1
		glViewport(0, 0, width, height)
		glMatrixMode(GL_PROJECTION)
		glLoadIdentity()
		gluPerspective(45, 1.0*width/height, 0.1, 100.0)
		# #gluLookAt(0.0,0.0,45.0,0,0,0,0,40.0,0)
		# if width <= height:
		# 	glOrtho(-1,1,-1*height/width,1*height/width,-1,1)        
		# else:
		# 	glOrtho(-1*height/width,1*height/width,-1,1,-1,1)
		glMatrixMode(GL_MODELVIEW)
		glLoadIdentity()
		pass

	def init_shading(self):
		glShadeModel(GL_SMOOTH)
		glClearColor(0.0, 0.0, 0.0, 0.0)
		glClearDepth(1.0)
		glEnable(GL_DEPTH_TEST)
		glShadeModel(GL_SMOOTH) 
		glDepthFunc(GL_LEQUAL)
		glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST)

		glEnable(GL_COLOR_MATERIAL)
		glEnable(GL_LIGHTING)
		glEnable(GL_LIGHT0)   
		glLight(GL_LIGHT0, GL_POSITION,  (0, 1, 1, 0))      
		glMatrixMode(GL_MODELVIEW)
		pass

class Point:

	def __init__(self, p, c = (1, 0, 0)):
		self.point_size = 0.5
		self.color = c
		self.x = p[0]
		self.y = p[1]
		self.z = p[2]
		pass

	def glvertex(self):
		glVertex3f(self.x, self.y, self.z)
		pass

class Triangle:

	points = None
	normal = None

	def __init__(self, p1, p2, p3, n = None):
		self.points = Point(p1),Point(p2),Point(p3)

		self.normal = Point(self.calculate_normal(self.points[0], self.points[1], self.points[2]))
		pass

	def calculate_normal(self, p1, p2, p3):
		a = self.calculate_vector(p3, p2)
		b = self.calculate_vector(p3, p1)

		return self.cross_product(a, b)

	def calculate_vector(self, p1, p2):
		return -p1.x + p2.x, -p1.y + p2.y, -p1.z + p2.z

	def cross_product(self, p1, p2):
		return (p1[1]*p2[2]-p2[1]*p1[2]) , (p1[2]*p2[0])-(p2[0]*p1[0]) , (p1[0]*p2[1])-(p2[0]*p1[1])

class Loader:

	model = []

	def get_triangles(self):
		if self.model:
			for face in self.model:
				yield face
		pass

	def draw(self):
		glBegin(GL_TRIANGLES)
		for tri in self.get_triangles():
			glNormal3f(tri.normal.x, tri.normal.y, tri.normal.z)
			glVertex3f(tri.points[0].x, tri.points[0].y, tri.points[0].z)
			glVertex3f(tri.points[1].x, tri.points[1].y, tri.points[1].z)
			glVertex3f(tri.points[2].x, tri.points[2].y, tri.points[2].z)
		glEnd()
		pass

	def load_stl(self, filename):
		fp = open(filename, 'rb')
		h = fp.read(80)
		type = h[0:5]
		fp.close()

		if type == 'solid':
			print 'reading text file ' + str(filename)
			self.load_text_stl(filename)
		else:
			print 'reading binary stl file ' + str(filename)
			self.load_binary_stl(filename)
		pass

	def load_text_stl(self, filename):
		rospy.logwarn('loading text')
		fp = open(filename, 'r')

		for line in fp.readlines():
			words = line.split()
			if len(words) > 0:
				if words[0] == 'solid':
					self.name = words[1]

				if words[0] == 'facet':
					center = [0.0, 0.0, 0.0]
					triangle = []
					normal = (eval(words[2]),eval(words[3]),eval(words[4]))

				if words[0] == 'vertex':
					triangle.append((eval(words[1]),eval(words[2]),eval(words[3])))

				if words[0] == 'endloop':
					if len(triangle) == 3:
						self.model.append(Triangle(triangle[0], triangle[1], triangle[2], normal))
		fp.close()
		pass

	def load_binary_stl(self, filename):
		rospy.logwarn('loading binary')
		fp = open(filename, 'rb')
		h = fp.read(80)

		l = struct.unpack('I', fp.read(4))[0]
		count = 0
		while True:
			try:
				p = fp.read(12)
				if len(p) == 12:
					n = struct.unpack('f',p[0:4])[0],struct.unpack('f',p[4:8])[0],struct.unpack('f',p[8:12])[0]

				p = fp.read(12)
				if len(p) == 12:
					p1 = struct.unpack('f',p[0:4])[0],struct.unpack('f',p[4:8])[0],struct.unpack('f',p[8:12])[0]

				p = fp.read(12)
				if len(p) == 12:
					p2 = struct.unpack('f',p[0:4])[0],struct.unpack('f',p[4:8])[0],struct.unpack('f',p[8:12])[0]

				p = fp.read(12)
				if len(p) == 12:
					p3 = struct.unpack('f',p[0:4])[0],struct.unpack('f',p[4:8])[0],struct.unpack('f',p[8:12])[0]

				new_tri = (n, p1, p2, p3)

				if len(new_tri) == 4:
					tri = Triangle(p1, p2, p3, n)
					self.model.append(tri)
				count += 1
				fp.read(2)

				if len(p) == 0:
					break
			except EOFError:
				break
		fp.close()
		pass