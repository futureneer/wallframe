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

#
# Author: Kelleher Guerin, futureneer@gmail.com, Johns Hopkins University
#

import roslib; roslib.load_manifest('modulair_core')
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
# Modulair Msgs
from modulair_msgs.msg import ModulairUser
from modulair_msgs.msg import ModulairUserArray
from modulair_msgs.msg import ModulairUserEvent
from modulair_msgs.msg import TrackerUser
from modulair_msgs.msg import TrackerUserArray as tracker_msg
# Modulair Core
import modulair_core
from modulair_core.srv import *

################################################################################
class ModulairAppWidget(QWidget):
  def __init__(self,name, app):
    super(ModulairAppWidget,self).__init__()
    # Member variables    
    self.name_ = name
    self.app_ = app
    self.ok_timer_ = QTimer(self)
    self.current_users_ = []
    self.current_user_event_ = ModulairUserEvent()
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
    # Clean up or delete things, such as ros parameters  
    pass    

  def user_state_cb(self,msg):
    self.current_users_ = msg.users
    # Signal functions here that you want to run on a user state callback
    pass

  def user_event_cb(self,msg):
    self.current_user_event_ = msg
    # Signal functions here that you want to run on an event callback
    pass