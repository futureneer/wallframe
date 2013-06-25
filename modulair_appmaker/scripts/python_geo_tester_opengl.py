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
# Author: Cesar Hernandez, z3tlin59@gmail.com, Johns Hopkins University
#

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
import python_quad_opengl
from python_quad_opengl import GLQuad

class PythonGeoTester():
  def __init__(self):
    rospy.init_node('python_geo_tester', anonymous=True)
    app = QtGui.QApplication(sys.argv)
    app_widget = PythonAppWidget("PythonGeoTester", app)
    # Running
    rospy.logwarn("PythonGeoTester: Started")
    app.exec_()
    # Done
    rospy.logwarn("PythonGeoTester: Finished")

class PythonAppWidget(ModulairAppWidget):
  
  signal_update_rotation = QtCore.Signal()
  
  def __init__(self, name, app):
    super(PythonAppWidget, self).__init__(name, app)
    self.glWidget = GLQuad(200, 200, 100, 100, None)
    mainLayout = QtGui.QGridLayout()
    mainLayout.addWidget(self.glWidget)
    self.setLayout(mainLayout)
    pass

if __name__ == '__main__':
  app = PythonGeoTester()