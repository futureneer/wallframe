#!/usr/bin/env python

from gl_widget_base import GLWidget
from gl_quad import Rectangle
from gl_utils import *

class VideoBrowser(GLWidget):

	def __init__(self, path, width, height, parent = None):
		self.path_ = path
		self.video_width_ = width
		self.video_height_ = height

		self.video = load_video(path)
		self.fps = get_video_fps(self.video)

		log_warn('FPS: ' + str(self.fps))

		super(VideoBrowser, self).__init__(delay = self.fps, parent = parent)
		pass

	def initGL(self):
		self.quad = Rectangle(0, 0, self.video_width_, self.video_height_, get_next_frame(self.video), 'IPL')
		pass

	def paint(self):
		self.quad.draw()
		pass

	def idle(self):
		frame = get_next_frame(self.video)
		if frame == None:
			self.clean_up()
			return
		self.quad.load_texture(frame, 'IPL')
		self.update_widget()
		pass