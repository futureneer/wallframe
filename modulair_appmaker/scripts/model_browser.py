#!/usr/bin/env python

from gl_widget_base import GLWidget
from gl_quad import Rectangle
from gl_model import Model
from gl_utils import *

class ModelBrowser(GLWidget):

	def __init__(self, path, width, height, parent = None):
		super(ModelBrowser, self).__init__(depth = True, parent = parent)
		self.path_ = path
		self.width_ = width
		self.height_ = height
		pass

	def initGL(self):
		self.model = Model()
		self.model.load_model(self.path_)
		pass

	def paint(self):
		self.model.render()
		pass

	def idle(self):
		self.model.rotate(self.width_, self.height_)
		self.update_widget()
		pass