#!/usr/bin/env python

import math
import numpy
import rospy
import time

from OpenGL.GL import *
from OpenGL.GLU import *
from PySide.QtOpenGL import QGLWidget
from PySide.QtCore import QTimer

import pyassimp
from pyassimp.postprocess import *
from pyassimp.helper import *

class GLWidget(QGLWidget):

	def __init__(self, width, height, parent=None):
		QGLWidget.__init__(self, parent)
		rospy.logwarn("CLASS INIT")

		self.wall_width = width
		self.wall_height = height

		self.scene = None

		self.using_fixed_cam = False
		self.current_cam_index = 0

		self.autofit = True
		self.postprocess = None

		self.angle = 0.0

		self.prev_time = 0
		self.prev_fps_time = 0
		self.frames = 0

		self.timer = QTimer()
		self.timer.timeout.connect(self.updateGL)
		# self.timer.start(0)
		pass

	def initializeGL(self):
		rospy.logwarn("GL INIT")

		self.load_model('/home/kel/modulair/modulair_appmaker/scripts/models/wuson.ply')
		# glClearColor(0.1, 0.1, 0.1, 1.0)
		glClearColor(0.5, 0.5, 0.5, 0.5)

		glEnable(GL_LIGHTING)

		glEnable(GL_CULL_FACE)
		glEnable(GL_DEPTH_TEST)

		glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE)
		glEnable(GL_NORMALIZE)
		glEnable(GL_LIGHT0)

		glMatrixMode(GL_PROJECTION)
		glLoadIdentity()
		gluPerspective(35.0, self.wall_width/float(self.wall_height), 0.1, 100.0)
		glMatrixMode(GL_MODELVIEW)
		self.set_default_camera()

		if self.autofit:
			self.fit_scene()

		glPushMatrix()
		pass

	def paintGL(self):
		rospy.logwarn("GL PAINT")
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

		glRotatef(self.angle, 0.0, 1.0, 0.0)
		self.recursive_render(self.scene.rootnode)

		self.do_motion()
		pass

	# def resizeGL(self, width, height):
	# 	rospy.logwarn('W: ' + str(width) + 'H: ' + str(height))
	# 	rospy.logwarn("GL RESIZE")
	# 	glMatrixMode(GL_PROJECTION)
	# 	glLoadIdentity()
	# 	gluPerspective(35.0, width/float(height), 0.1, 100.0)
	# 	glMatrixMode(GL_MODELVIEW)
	# 	self.set_default_camera()

	# 	if self.autofit and self.scene:
	# 		self.fit_scene()

	# 	glPushMatrix()
	# 	pass

	def set_default_camera(self):
		rospy.logwarn("DEF CAM")
		if not self.using_fixed_cam:
			glLoadIdentity()

			gluLookAt(0.,0.,3.,
						0.,0.,-5.,
						0.,1.,0.)
		pass
	
	def prepare_gl_buffers(self, mesh):
		rospy.logwarn("BUFFERS")
		mesh.gl = {}

		mesh.gl["vertices"] = glGenBuffers(1)
		glBindBuffer(GL_ARRAY_BUFFER, mesh.gl["vertices"])
		glBufferData(GL_ARRAY_BUFFER, mesh.vertices, GL_STATIC_DRAW)

		mesh.gl["normals"] = glGenBuffers(1)
		glBindBuffer(GL_ARRAY_BUFFER, mesh.gl["normals"])
		glBufferData(GL_ARRAY_BUFFER, mesh.normals, GL_STATIC_DRAW)


		mesh.gl["triangles"] = glGenBuffers(1)
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mesh.gl["triangles"])
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, mesh.faces, GL_STATIC_DRAW)

		glBindBuffer(GL_ARRAY_BUFFER,0)
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,0)
		pass

	def load_model(self, path):
		rospy.logwarn("MODEL LOAD")

		if self.postprocess:
			self.scene = pyassimp.load(path, postprocess)
		else:
			self.scene = pyassimp.load(path)

		scene = self.scene
		self.bb_min, self.bb_max = get_bounding_box(self.scene)

		self.scene_center = [(a + b) / 2. for a, b in zip(self.bb_min, self.bb_max)]

		for index, mesh in enumerate(scene.meshes):
			self.prepare_gl_buffers(mesh)

		pyassimp.release(scene)
		pass

	def fit_scene(self, restore = False):
		rospy.logwarn("FIT")
		x_max = self.bb_max[0] - self.bb_min[0]
		y_max = self.bb_max[1] - self.bb_min[1]
		tmp = max(x_max, y_max)
		z_max = self.bb_max[2] - self.bb_min[2]
		tmp = max(z_max, tmp)

		if not restore:
			tmp = 1. / tmp

		glScalef(tmp, tmp, tmp)

		direction = -1 if not restore else 1
		glTranslatef( direction * self.scene_center[0], 
						direction * self.scene_center[1], 
						direction * self.scene_center[2] )

		return x_max, y_max, z_max

	def apply_material(self, mat):
		rospy.logwarn("MATERIAL")
		if not hasattr(mat, "gl_mat"):
			diffuse = numpy.array(mat.properties.get("diffuse", [0.8, 0.8, 0.8, 1.0]))
			specular = numpy.array(mat.properties.get("specular", [0., 0., 0., 1.0]))
			ambient = numpy.array(mat.properties.get("ambient", [0.2, 0.2, 0.2, 1.0]))
			emissive = numpy.array(mat.properties.get("emissive", [0., 0., 0., 1.0]))
			shininess = min(mat.properties.get("shininess", 1.0), 128)
			wireframe = mat.properties.get("wireframe", 0)
			twosided = mat.properties.get("twosided", 1)

			setattr(mat, "gl_mat", glGenLists(1))
			glNewList(mat.gl_mat, GL_COMPILE)

			glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diffuse)
			glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specular)
			glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, ambient)
			glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, emissive)
			glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, shininess)
			glPolygonMode(GL_FRONT_AND_BACK, GL_LINE if wireframe else GL_FILL)
			glDisable(GL_CULL_FACE) if twosided else glEnable(GL_CULL_FACE)

			glEndList()

		glCallList(mat.gl_mat)
		pass

	def do_motion(self):
		gl_time = time.clock()

		self.angle = (gl_time - self.prev_time) * 0.1

		self.prev_time = gl_time

		self.frames += 1
		if gl_time - self.prev_fps_time >= 1000:
			current_fps = self.frames * 1000 / (gl_time - self.prev_fps_time)
			self.frames = 0
			self.prev_fps_time = gl_time

		# self.updateGL()
		pass

	def recursive_render(self, node):
		rospy.logwarn("RECURSIVE")
		glPushMatrix()
		m = node.transformation.transpose()
		glMultMatrixf(m)

		for mesh in node.meshes:
			self.apply_material(mesh.material)

			glBindBuffer(GL_ARRAY_BUFFER, mesh.gl["vertices"])
			glEnableClientState(GL_VERTEX_ARRAY)
			glVertexPointer(3, GL_FLOAT, 0, None)

			glBindBuffer(GL_ARRAY_BUFFER, mesh.gl["normals"])
			glEnableClientState(GL_NORMAL_ARRAY)
			glNormalPointer(GL_FLOAT, 0, None)

			glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, mesh.gl["triangles"])
			glDrawElements(GL_TRIANGLES,len(mesh.faces) * 3, GL_UNSIGNED_INT, None)

			glDisableClientState(GL_VERTEX_ARRAY)
			glDisableClientState(GL_NORMAL_ARRAY)

			glBindBuffer(GL_ARRAY_BUFFER, 0)
			glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0)

		for child in node.children:
			self.recursive_render(child)

		glPopMatrix()
		pass

	# def on_key_press(self, key, x, y):
	# 	pass

	# def render(self, filename=None, autofit = True, postprocess = None):
	# 	rospy.logwarn("RENDER")
	# 	self.autofit = autofit
	# 	self.postprocess = postprocess
	# 	self.load_model(filename)
	# 	pass

	def clean_up(self):
		self.timer.stop()
		pass

