#!/usr/bin/env python

from gl_widget_base import GLWidget
from gl_quad import Rectangle
from gl_utils import *

class ImageBrowser(GLWidget):

	def __init__(self, path, width, height, parent = None):
		super(ImageBrowser, self).__init__(parent = parent)

		self.path_ = path
		self.width_ = width
		self.height_ = height

		self.images = load_image_files(find_image_files(self.path_))
		self.num_imgages = len(self.images)
		self.img_indx = 0
		pass

	def initGL(self):
		self.quad = Rectangle(0, 0, self.width_, self.height_, self.images[self.img_indx], 'PIL')

	def paint(self):
		self.quad.draw()
		pass

	def next_image(self):
		self.img_indx = (self.img_indx + 1) % self.num_imgages
		self.quad.load_texture(self.images[self.img_indx], 'PIL')
		self.update_widget()
		pass

	def prev_image(self):
		self.img_indx = (self.img_indx - 1) % self.num_imgages
		self.quad.load_texture(self.images[self.img_indx], 'PIL')
		self.update_widget()
		pass