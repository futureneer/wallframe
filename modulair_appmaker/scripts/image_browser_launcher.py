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
### Browser Imports ###
from sample import GLWidget

class Tester(ModulairAppWidget):

  signal_next_image = QtCore.Signal()
  signal_prev_image = QtCore.Signal()

  def __init__(self, name, app):

    super(Tester, self).__init__(name, app)

    self.glWidget = GLWidget()
    mainLayout = QtGui.QGridLayout()
    mainLayout.addWidget(self.glWidget)
    self.setLayout(mainLayout)

    self.signal_next_image.connect(self.glWidget.next_image)
    self.signal_prev_image.connect(self.glWidget.prev_image)
    pass

  def user_state_cb(self, msg):
    self.current_users_ = msg.users


    if len(self.current_users_) > 0:
      self.current_users_ = msg.users

      if self.current_users_[0].left_hand_on_head:
        self.signal_prev_image.emit()
      elif self.current_users_[0].right_hand_on_head:
        self.signal_next_image.emit()
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