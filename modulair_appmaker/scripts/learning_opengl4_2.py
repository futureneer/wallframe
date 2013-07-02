#!/usr/bin/env python
import sys
### ROS Imports ###
import rospy
### PySide Imports ###
import PySide
from PySide import QtCore
from PySide import QtGui
from PySide.QtOpenGL import QGLWidget
from PySide.QtGui import QApplication
### OpenGL Imports ###
from OpenGL.GL import *
from OpenGL.arrays import vbo
from OpenGL.GL import shaders
import numpy


class Rectangle(QGLWidget):

  def __init__(self):
    QGLWidget.__init__(self, None)
    pass

  def initializeGL(self):
    self.create_shader()
    self.create_vbo()
    glClearColor(0.0, 0.0, 0.0, 0.0)
    pass

  def paintGL(self):
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    try:
      self.vbo.bind()
      try:
        glEnableClientState(GL_VERTEX_ARRAY)
        glEnableClientState(GL_COLOR_ARRAY)
        glVertexPointer(3, GL_FLOAT, 24, self.vbo)
        glColorPointer(3, GL_FLOAT, 24, self.vbo + 12)
        glPointSize(50)
        glPolygonMode(GL_FRONT_AND_BACK, GL_POINT)
        glDrawArrays(GL_TRIANGLE_STRIP, 0, 4)
      finally:
        self.vbo.unbind()
        glDisableClientState(GL_VERTEX_ARRAY)
        glDisableClientState(GL_COLOR_ARRAY)
    finally:
      glUseProgram(0)
    pass

  def resizeGL(self, length, width):
    glViewport(0, 0, length, width)
    pass

  def create_vbo(self):
    self.vbo = vbo.VBO(
      numpy.array([
        [-0.8, 0.8, 0.0, 1.0, 0.0, 0.0],
        [0.8, 0.8, 0.0, 0.0, 1.0, 0.0],
        [-0.8, -0.8, 0.0, 0.0, 0.0, 1.0],
        [0.8, -0.8, 0.0, 1.0, 1.0, 1.0]
        ], dtype='f')
      )
    pass

  def create_shader(self):
    vertex_shader = shaders.compileShader("""
    #version 400
  
    layout(location = 0) in vec4 in_Position;
    layout(location = 1) in vec4 in_Color;
    out vec4 ex_Color;

    void main(void)
    {
      gl_Position = in_Position;
      ex_Color = in_Color;
    }
    """, GL_VERTEX_SHADER)

    fragmnet_shader = shaders.compileShader("""
    #version 400

    in vec4 ex_Color;
    out vec4 out_Color;

    void main(void)
    {
      out_Color = ex_Color;
    }
    """, GL_FRAGMENT_SHADER)

    self.shader = shaders.compileProgram(vertex_shader, fragmnet_shader)
    glUseProgram(self.shader)
    pass