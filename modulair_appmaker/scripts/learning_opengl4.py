#!/usr/bin/env python
import sys
import rospy
import PySide
from PySide import QtCore
from PySide import QtGui
from PySide.QtOpenGL import QGLWidget
from PySide.QtGui import QApplication

try:
  from OpenGL import GL
  print 'OpenGL Version:',GL.glGetString(GL.GL_VERSION)
except ImportError:
  app = QApplication(sys.argv)
  QtGui.QMessageBox.critical(None, "Learning OpenGl 4",
    "PyOpenGL must be installed to run this example.",
    QtGui.QMessageBox.Ok | QtGui.QMessageBox.Default, 
    QtGui.QMessageBox.NoButton)
  sys.exit(1)

class BaseTemplate(QGLWidget):
  def __init__(self):
    QGLWidget.__init__(self, None)
    pass

  def initializeGL(self):
    GL.glClearColor(0.0, 0.0, 0.0, 0.0)
    pass

  def paintGL(self):
    GL.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT)
    pass

  def resizeGL(self, length, width):
    GL.glViewport(0, 0, length, width)
    pass
