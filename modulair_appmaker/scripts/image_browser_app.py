#!/usr/bin/env python

import roslib; roslib.load_manifest('modulair_appmaker')
from gl_wall_canvas import GLWallCanvas
from image_browser import ImageBrowser
from gl_utils import *

class ImageBrowserApp(GLWallCanvas):

	signal_next_image = create_signal()
	signal_prev_image = create_signal()

	def __init__(self, name):
		super(ImageBrowserApp, self).__init__(name)

		path = '/home/kel/modulair/modulair_appmaker/scripts/pics/'	

		self.image_browser_widget = ImageBrowser(path, self.default_width, self.default_height)
		self.use_default_layout(self.image_browser_widget)

		self.signal_next_image.connect(self.image_browser_widget.next_image)
		self.signal_prev_image.connect(self.image_browser_widget.prev_image)
		pass

	def user_event_cb(self, msg):
		self.current_user_event_ = msg

		if msg.message == 'left_hand_on_head':
			self.signal_prev_image.emit()
		elif msg.message == 'right_hand_on_head':
			self.signal_next_image.emit()
		pass

if __name__ == '__main__':
	app_canvas = ImageBrowserApp('image_browser')
	log_warn('image_browser: Started')
	app_canvas.app.exec_()
	log_warn('image_browser: Finished')