#!/usr/bin/env python
################################################################################
# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Johns Hopkins University
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# * Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above
# copyright notice, this list of conditions and the following
# disclaimer in the documentation and/or other materials provided
# with the distribution.
# * Neither the name of the Johns Hopkins University nor the names of its
# contributors may be used to endorse or promote products derived
# from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
################################################################################
__author__ = "Kelleher Guerin"
__email__ = "futureneer@gmail.com"
__copyright__ = "2013, The Johns Hopkins University"
__license__ = "BSD"
################################################################################

import roslib; roslib.load_manifest('wallframe_core')
import rospy
### PySide ###
import PySide
from PySide.QtGui import QWidget, QApplication
from PySide.QtCore import QTimer
from PySide import QtCore
# ROS Msgs
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool
from std_msgs.msg import String
# Wallframe Msgs
from wallframe_msgs.msg import WallframeUser
from wallframe_msgs.msg import WallframeUserArray
from wallframe_msgs.msg import WallframeUserEvent
from wallframe_msgs.msg import TrackerUser
from wallframe_msgs.msg import TrackerUserArray as tracker_msg
# Wallframe Core
import wallframe_core
from wallframe_core.srv import *


"""
Base class widget for a python app in wallframe
"""
class WallframeAppWidget(QWidget):
  """
  In init, all of the necessary ROS interfaces are created, including publishers and subscribers for getting user data from the wallframe user manager, as well as getting the window size and other information from the parameter server.  Also, this function creates a basic qt window that will be the parent of anything else in the app.
  """
  def __init__(self,name, app):
    super(WallframeAppWidget,self).__init__()

    # Member variables    
    self.name_ = name
    self.app_ = app
    self.ok_timer_ = QTimer(self)
    self.current_users_ = []
    self.current_user_event_ = WallframeUserEvent()
    self.users_ = {}
    self.num_users_ = 0
    self.height_perc_ = 1
    self.focused_user_id_ = -1
    rospy.logwarn(self.name_ + ": App Widget Starting")

    # ROS Subscribers
    self.user_state_sub_ = rospy.Subscriber("/wallframe/users/state", WallframeUserArray, self.user_state_cb)
    self.user_event_sub_ = rospy.Subscriber("/wallframe/users/events", WallframeUserEvent, self.user_event_cb)
    
    # App parameters
    if rospy.has_param("/wallframe/core/params/x"):
      self.x_ = rospy.get_param("/wallframe/core/params/x")
    else:
      rospy.logerr(self.name_ + ": parameter [x] not found on server")

    if rospy.has_param("/wallframe/core/params/y"):
      self.y_ = rospy.get_param("/wallframe/core/params/y")
    else:
      rospy.logerr(self.name_ + ": parameter [y] not found on server")

    if rospy.has_param("/wallframe/core/params/width"):
      self.width_ = rospy.get_param("/wallframe/core/params/width")
    else:
      rospy.logerr(self.name_ + ": parameter [width] not found on server")

    if rospy.has_param("/wallframe/core/params/height"):
      self.height_ = rospy.get_param("/wallframe/core/params/height")
    else:
      rospy.logerr(self.name_ + ": parameter [height] not found on server")

    if rospy.has_param("/wallframe/app/params/height_percentage"):
      self.height_perc_ = rospy.get_param("/wallframe/app/params/height_percentage")
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

  """
  Check to see if ROS has recieved a TERM signal, and if so, stop the Qt thread
  """
  # @QtCore.Slot(bool)
  def check_ok(self):
    if rospy.is_shutdown():
      self.clean_up()
      self.app_.exit()

  """
  Clean up or delete things, such as ros parameters  
  """
  def clean_up(self):
    rospy.logwarn(self.name_ + ": App Widget Cleaning up")
    pass    

  """
  Signal functions here that you want to run on a user state callback
  """
  def user_state_cb(self,msg):
    self.current_users_ = msg.users
    pass
  """
  Signal functions here that you want to run on an event callback
  """
  def user_event_cb(self,msg):
    self.current_user_event_ = msg
    pass