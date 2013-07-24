#!/usr/bin/env python
import sys
### ROS Imports ###
import roslib; roslib.load_manifest('modulair_appmaker')
import rospy
### PySide Imports ###
import PySide
from PySide import QtCore
from PySide import QtGui
### Modulair Core Imports ###
import modulair_core
from modulair_core import ModulairAppWidget
### Browser Imports ###
from model_browser_3 import GLWidget

class Tester(ModulairAppWidget):

	def __init__(self, name, app):

		super(Tester, self).__init__(name, app)

		# name = '/home/kel/modulair/modulair_appmaker/scripts/models/cube.stl'

		self.glWidget = GLWidget(int(self.width_), int(self.height_ * self.height_perc_))
		# self.glWidget.updateGL()
		# self.glWidget.render(name)
		mainLayout = QtGui.QGridLayout()
		mainLayout.addWidget(self.glWidget)
		self.setLayout(mainLayout)
		pass

	# def render(self, filename):
	# 	self.glWidget.load_model(filename)
	# 	pass

	def clean_up(self):
		self.glWidget.clean_up()

if __name__ == '__main__':
	rospy.init_node("model browser", anonymous = True)
	app = QtGui.QApplication(sys.argv)
	app_widget = Tester("model browser", app)
	rospy.logwarn("model browser: Started")
	app.exec_()
	rospy.logwarn("model browser: Finished")