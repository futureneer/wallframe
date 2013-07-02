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
import learning_opengl4_4
from learning_opengl4_4 import Cube

class Tester(ModulairAppWidget):
  def __init__(self, name, app):
    super(Tester, self).__init__(name, app)
    self.glWidget = Cube()
    mainLayout = QtGui.QGridLayout()
    mainLayout.addWidget(self.glWidget)
    self.setLayout(mainLayout)
    pass

  def mouseMoveEvent(self, event):
    self.glWidget.updateGL()

if __name__ == '__main__':
  rospy.init_node("learning_opengl4_tester", anonymous=True)
  app = QtGui.QApplication(sys.argv)
  app_widget = Tester("learning_opengl4_tester", app)
  rospy.logwarn("learning_opengl4_tester: Started")
  app.exec_()
  rospy.logwarn("learning_opengl4_tester: Finished")