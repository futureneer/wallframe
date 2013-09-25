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

"""
  wallframe_app_menu_button
  This button is the widget to be used for all the application entries.
  
  @author: Andy Tien
"""

from PySide.QtGui import QWidget, QPixmap, QLabel, QVBoxLayout

class WallframeAppButton(QWidget):
  def __init__(self, app_tag, app_image_path, app_description_path):
    QWidget.__init__(self)
    #App tag
    self.app_tag_ = QLabel(app_tag)
    #App image
    app_image = QPixmap()
    app_image.load(app_image_path)
    self.app_image_ = QLabel()
    self.app_image_.setPixmap(app_image)
    #App description
    try:
      f = open(app_description_path, 'r')  
      self.app_description_ = f.read()
      f.close()
    except:
      print "Error opening description. Quitting."
    
    self.setToolTip(self.app_description_)
    #Layout the child widgets
    self.child_layout_ = QVBoxLayout()
    self.child_layout_.addWidget(self.app_image_)
    self.child_layout_.addWidget(self.app_tag_)
    
    self.setLayout(self.child_layout_)