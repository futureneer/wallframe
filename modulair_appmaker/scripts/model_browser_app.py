#!/usr/bin/env python

import roslib; roslib.load_manifest('modulair_appmaker')
from gl_wall_canvas import GLWallCanvas
from model_browser import ModelBrowser
from gl_utils import *

class ModelBrowserApp(GLWallCanvas):
	def __init__(self, name):
		super(ModelBrowserApp, self).__init__(name)
		
		path = '/home/kel/modulair/modulair_appmaker/scripts/models/Collada/cow.x'
		
		self.model_browser_widget = ModelBrowser(path, self.default_width, self.default_height)
		self.use_default_layout(self.model_browser_widget)
		pass

if __name__ == '__main__':
	app_canvas = ModelBrowserApp('model_browser')
	log_warn('model_browser: Started')
	app_canvas.app.exec_()
	log_warn('model_browser: Finished')