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
UserBar Tabs
'''

from PySide.QtGui import QWidget, QLabel
from PySide.QtCore import Qt, QSize

class WallframeAppUserBarTab(QLabel):
  def __init__(self, text, parent):
    super(WallframeAppUserBarTab, self).__init__(text, parent)
    self.width_ = 50
    self.height_ = 30
    self.setFixedSize(QSize(self.width_, self.height_))
  def update_position(self, x, y):
    self.move(x,y)
    
#class WallframeAppUserBarPopUp(QWidget):
#  def __init__(self, parent):
#    super(WallframeAppUserBarPopUp, self).__init__(parent)
#    self.width_ = 50
#    self.height_ = 90
#    self.setFixedSize(QSize(self.width_, self.height_))
#    self.tab.setWindowFlags(Qt.FramelessWindowHint)
#    self.tabx = 200
#    self.taby = 300
#    self.tab.move(self.tabx, self.taby)