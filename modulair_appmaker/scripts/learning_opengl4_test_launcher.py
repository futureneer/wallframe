#!/usr/bin/env python
import sys
### ROS Imports ###
import roslib; roslib.load_manifest('modulair_appmaker')
import rospy
### PySide Imports ###
import PySide
from PySide import QtCore
from PySide import QtGui
#### Modulair Core Imports ###
import modulair_core
from modulair_core import ModulairAppWidget
import learning_opengl4_5
from learning_opengl4_5 import Rot_Cube

class Tester(ModulairAppWidget):
  def __init__(self, name, app):
    super(Tester, self).__init__(name, app)
    self.glWidget = Rot_Cube()
    mainLayout = QtGui.QGridLayout()
    mainLayout.addWidget(self.glWidget)
    self.setLayout(mainLayout)
    pass

  def clean_up(self):
    rospy.logwarn(self.name_ + ": App Widget Cleaning up")
    self.glWidget.clean_up()

if __name__ == '__main__':
  rospy.init_node("learning_opengl4_tester", anonymous=True)
  app = QtGui.QApplication(sys.argv)
  app_widget = Tester("learning_opengl4_tester", app)
  rospy.logwarn("learning_opengl4_tester: Started")
  app.exec_()
  rospy.logwarn("learning_opengl4_tester: Finished")