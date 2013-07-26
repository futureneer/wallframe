#!/usr/bin/env python

import numpy
from OpenGL.GL import *
from OpenGL.GLU import *

import pyassimp
from pyassimp.postprocess import *
from pyassimp.helper import *

class Model(object):

	def __init__(self, parent = None):
		super(Model, self).__init__()

		self.scene = None
		self.angle = 1.0
		pass

	def rotate(self, width, height):
		# glMatrixMode(GL_PROJECTION)
		# glLoadIdentity()
		# gluPerspective(35.0, width/float(height), 0.1, 100.0)
		# glMatrixMode(GL_MODELVIEW)

		glRotatef(self.angle*1.7, 0.0, 1.0, 0.0)
		pass

	def load_model(self,path):		
		self.scene = pyassimp.load(path)
		scene = self.scene
		self.bb_min, self.bb_max = get_bounding_box(self.scene)

		self.scene_center = [(a + b) / 2. for a, b in zip(self.bb_min, self.bb_max)]


		for index, mesh in enumerate(scene.meshes):						
			self.prepare_gl_buffers(mesh)

		pyassimp.release(scene)
		self.fit_scene()
		pass

	def prepare_gl_buffers(self,mesh):
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

	def fit_scene(self):
		x_max = self.bb_max[0] - self.bb_min[0]
		y_max = self.bb_max[1] - self.bb_min[1]
		tmp = max(x_max, y_max)
		z_max = self.bb_max[2] - self.bb_min[2]
		tmp = max(z_max, tmp)

		tmp = 0.5 / tmp

		glScalef(tmp, tmp, tmp)	

		return x_max, y_max, z_max

	def render(self, child = None):
		if child == None:
			node = self.scene.rootnode
		else:
			node = child
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
			self.render(child)

		glPopMatrix()
		pass

	def apply_material(self, mat):
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