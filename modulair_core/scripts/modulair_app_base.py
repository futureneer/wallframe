#!/usr/bin/env python
import roslib; roslib.load_manifest('modulair_core')
import rospy, sys
import PySide
from PySide.QtGui import QWidget, QApplication
from PySide.QtCore import QTimer
from PySide import QtCore


from geometry_msgs.msg import Transform
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool
from std_msgs.msg import String

from modulair_msgs.msg import ModulairUser
from modulair_msgs.msg import ModulairUserArray
from modulair_msgs.msg import ModulairUserEvent
from modulair_msgs.msg import TrackerUser
from modulair_msgs.msg import TrackerUserArray as tracker_msg

import modulair_core
from modulair_core.srv import *

class ContainerWidget(QWidget):
  def __init__(self,name, app):
    super(ContainerWidget,self).__init__()
    self.name_ = name
    self.ok_timer_ = QTimer(self)
    self.app_ = app
    # Qt Connections
    self.connect(self.ok_timer_, QtCore.SIGNAL("timeout()"), self.check_ok)
    self.ok_timer_.start(15)
    # Publishers and Subscribers

  # @QtCore.Slot(bool)
  def check_ok(self):
    if rospy.is_shutdown():
      self.app_.exit()


class ModulairAppBase():
  def __init__(self):
    # Member variables
    self.current_users_ = []
    self.users_ = {}
    self.num_users_ = 0
    self.focused_user_id_ = -1
    # ROS Init
    rospy.init_node('python_app',anonymous=True)
    # ROS Subscribers
    self.user_state_sub_ = rospy.Subscriber("/modulair/users/state", ModulairUserArray, self.user_state_cb)
    self.user_event_sub_ = rospy.Subscriber("/modulair/users/events", ModulairUserEvent, self.user_event_cb)
    
    if rospy.has_param("/modulair/core/params/x"):
      self.x_ = rospy.get_param("/modulair/core/params/x")
    else:
      rospy.logerr("ModulairAppBase: parameter [x] not found on server")

    if rospy.has_param("/modulair/core/params/y"):
      self.y_ = rospy.get_param("/modulair/core/params/y")
    else:
      rospy.logerr("ModulairAppBase: parameter [y] not found on server")

    if rospy.has_param("/modulair/core/params/width"):
      self.width_ = rospy.get_param("/modulair/core/params/width")
    else:
      rospy.logerr("ModulairAppBase: parameter [width] not found on server")

    if rospy.has_param("/modulair/core/params/height"):
      self.height_ = rospy.get_param("/modulair/core/params/height")
    else:
      rospy.logerr("ModulairAppBase: parameter [height] not found on server")

    ### Qt App and Widget ###
    app = QApplication(sys.argv)

    self.container_ = ContainerWidget('container',app)
    self.container_.setWindowFlags(QtCore.Qt.FramelessWindowHint)
    self.container_.resize(self.width_, self.height_)
    self.container_.move(self.x_,self.y_)
    self.container_.setWindowTitle('Simple')
    self.container_.show()

    # Running
    rospy.logwarn("ModulairPythonApp: Started")  
    # sys.exit(app.exec_())
    app.exec_()
    # Quitting
    self.clean_up()
    rospy.logwarn("ModulairPythonApp: Finished")
  
  def clean_up(self):
    rospy.logwarn("ModulairPythonApp: Cleaning up")  
    pass    

  def user_state_cb(self,msg):
    self.current_users_ = msg.users
    pass

  def user_event_cb(self,msg):
    pass

# MAIN
if __name__ == '__main__':
  m = ModulairAppBase()