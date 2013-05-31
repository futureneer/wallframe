#!/usr/bin/env python
import roslib; roslib.load_manifest('modulair_core')
import rospy
### PySide ###
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

class ModulairAppWidget(QWidget):
  def __init__(self,name, app):
    super(ModulairAppWidget,self).__init__()
    # Member variables    
    self.name_ = name
    self.app_ = app
    self.ok_timer_ = QTimer(self)
    self.current_users_ = []
    self.users_ = {}
    self.num_users_ = 0
    self.height_perc_ = 1
    self.focused_user_id_ = -1
    rospy.logwarn(self.name_ + ": App Widget Starting")
    # ROS Subscribers
    self.user_state_sub_ = rospy.Subscriber("/modulair/users/state", ModulairUserArray, self.user_state_cb)
    self.user_event_sub_ = rospy.Subscriber("/modulair/users/events", ModulairUserEvent, self.user_event_cb)
    
    # App parameters
    if rospy.has_param("/modulair/core/params/x"):
      self.x_ = rospy.get_param("/modulair/core/params/x")
    else:
      rospy.logerr(self.name_ + ": parameter [x] not found on server")

    if rospy.has_param("/modulair/core/params/y"):
      self.y_ = rospy.get_param("/modulair/core/params/y")
    else:
      rospy.logerr(self.name_ + ": parameter [y] not found on server")

    if rospy.has_param("/modulair/core/params/width"):
      self.width_ = rospy.get_param("/modulair/core/params/width")
    else:
      rospy.logerr(self.name_ + ": parameter [width] not found on server")

    if rospy.has_param("/modulair/core/params/height"):
      self.height_ = rospy.get_param("/modulair/core/params/height")
    else:
      rospy.logerr(self.name_ + ": parameter [height] not found on server")

    if rospy.has_param("/modulair/app/params/height_percentage"):
      self.height_perc_ = rospy.get_param("/modulair/app/params/height_percentage")
    else:
      rospy.logerr(self.name_ + ": parameter [height_percentage] not found on server")
    rospy.logwarn(self.name_ + ": height percentage set to " + str(self.height_perc_))

    # Set base app widget size and hints based on parameters
    self.setWindowFlags(QtCore.Qt.FramelessWindowHint)
    self.resize(self.width_, int(self.height_*self.height_perc_) )
    self.move(self.x_,self.y_)
    self.setWindowTitle(self.name_)
    self.show()

    # Set up ros_ok watchdog timer to handle termination and ctrl-c
    self.connect(self.ok_timer_, QtCore.SIGNAL("timeout()"), self.check_ok)
    self.ok_timer_.start(15)

    rospy.logwarn(self.name_ + ": App Widget Set Up Successfully")

  # @QtCore.Slot(bool)
  def check_ok(self):
    if rospy.is_shutdown():
      self.clean_up()
      self.app_.exit()

  def clean_up(self):
    rospy.logwarn(self.name_ + ": App Widget Cleaning up")  
    pass    

  def user_state_cb(self,msg):
    self.current_users_ = msg.users
    pass

  def user_event_cb(self,msg):
    pass