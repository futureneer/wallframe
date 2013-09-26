#!/usr/bin/env python
#####################################################################
# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Johns Hopkins University
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# # Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.
# # Redistributions in binary form must reproduce the above
# copyright notice, this list of conditions and the following
# disclaimer in the documentation and/or other materials provided
# with the distribution.
# # Neither the name of the Johns Hopkins University nor the names of its
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
#####################################################################

###
# Author: Kelleher Guerin, futureneer@gmail.com, Johns Hopkins University
###

'''
Create User Bar at bottom of screen, that is able to update position dynamically.
'''

import sys, rospy, random

from std_msgs.msg import String
from PySide.QtGui import QWidget, QApplication
from PySide.QtCore import QSize, Qt

from wallframe_app_user_bar_tab import WallframeAppUserBarTab 

class WallframeAppUserBar(QWidget):
  def __init__(self):
    self.qapp_ = QApplication(sys.argv)
    QWidget.__init__(self)
    
    # Keep a dictionary of users_.
    self.users_ = {}
    # Keep a dictionary of user popups_.
#    self.popups_ = {}
    # List of users_ (probably via RosParam)
    self.userlist_ = ["user1", "user2", "user3"] 
    
    # Generate boxes
    for user in self.userlist_:
      self.users_[user] = WallframeAppUserBarTab(user, self)
      self.users_[user].update_position(random.randint(0, 1366), 0)
    # Generate Popups
#    self.popup_root = QWidget()
#    for user in self.userlist_:
#      self.popups_[user] = WallframeAppUserBarPopUp(self.popup_root)
    
    
    # Size restrictions
    self.width_ = 1366 # These values probably should be passed in via rosparams
    self.height_ = 25
    self.setFixedSize(QSize(self.width_, self.height_))
    # Set Initial Position
    self.setGeometry(0, 300, self.width_, self.height_)
    # Frameless Window
    self.setWindowFlags(Qt.FramelessWindowHint)
        
    # Set Background color to Red
    p = self.palette()
    p.setColor(self.backgroundRole(), Qt.red)
    self.setPalette(p)

    # Setup ROS and subscribers
    # TODO: Do we need to keep track of these subs?
    rospy.init_node('userbar')
    for user in self.userlist_:
      rospy.Subscriber(user, String, self.update, user)
    
  def update(self, data, user):
    # Call back function for subscribers to update position
    newx = int(data.data)
    self.users_[user].update_position(newx, 0)
    
  def run(self):
    self.show()
    self.qapp_.exec_()
    
def test():
  bar = WallframeAppUserBar()
  bar.run()
  
if __name__ == "__main__":
  test()  