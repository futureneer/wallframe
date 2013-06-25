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
import rospy
### PySide Imports ###
import PySide
from PySide import QtCore
from PySide import QtGui
from PySide.QtOpenGL import QGLWidget
from PySide.QtGui import QApplication

try:
  from OpenGL import GL
except ImportError:
  app = QtGui.QApplication(sys.argv)
  QtGui.QMessageBox.critical(None, "OpenGL GLQuad",
    "PyOpenGL must be installed to run this example.",
    QtGui.QMessageBox.Ok | QtGui.QMessageBox.Default, 
    QtGui.QMessageBox.NoButton)
  sys.exit(1)

class GLQuad(QGLWidget):

  def __init__(self, length=400, width=400, x=0, y=0, parent=None):
    QGLWidget.__init__(self, parent)
    self.length_ = length
    self.width_ = width
    self.x_ = x
    self.y_ = y
    self.Teal = QtGui.QColor.fromCmykF(0.902, 0.008, 0, 0.337)
    self.Red = QtGui.QColor.fromCmykF(0, 0.667, 0.625, 0.247)

  def initializeGL(self):
    self.qglClearColor(self.Teal)
    self.object = self.makeObject(self.length_, self.width_, self.x_, self.y_)
    GL.glShadeModel(GL.GL_FLAT)
    GL.glEnable(GL.GL_DEPTH_TEST)
    GL.glEnable(GL.GL_CULL_FACE)

  def paintGL(self):
    GL.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT)
    GL.glLoadIdentity()
    GL.glTranslated(0.0, 0.0, -10.0)
    GL.glCallList(self.object)

  def resizeGL(self, width, height):
    side = min(width, height)
    # GL.glViewport((width - side) / 2, (height - side) / 2, side, side)
    GL.glViewport(0, 0, width, height)
    GL.glMatrixMode(GL.GL_PROJECTION)
    GL.glLoadIdentity()
    GL.glOrtho(-0.5, +0.5, +0.5, -0.5, 4.0, 15.0)
    GL.glMatrixMode(GL.GL_MODELVIEW)

  def makeObject(self, length, width, x, y):
    genList = GL.glGenLists(1)
    GL.glNewList(genList, GL.GL_COMPILE)
    GL.glBegin(GL.GL_QUADS)
    
    x1 = -0.29
    y1 = -0.15

    x2 = +0.29
    y2 = -0.15

    x3 = +0.29
    y3 = +0.18

    x4 = -0.29
    y4 = +0.18

    self.quad(x1, y1, x2, y2, x3, y3, x4, y4)
  
    # factor = float(1000)
    # l_half = (length / 2) / factor
    # w_half = (width / 2) / factor
    # self.quad(-l_half, -w_half, l_half, -w_half, l_half, w_half, -l_half, w_half)
    
    GL.glEnd()
    GL.glEndList()

    return genList

  def quad(self, x1, y1, x2, y2, x3, y3, x4, y4):
    self.qglColor(self.Red)

    GL.glVertex3d(x4, y4, +0.05)
    GL.glVertex3d(x3, y3, +0.05)
    GL.glVertex3d(x2, y2, +0.05)
    GL.glVertex3d(x1, y1, +0.05)

  def normalizeAngle(self, angle):
    while angle < 0:
      angle += 360 * 16

    while angle > 360 * 16:
      angle -= 360 * 16

    return angle