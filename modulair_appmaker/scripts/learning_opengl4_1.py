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


class Triangle(QGLWidget):

  # vertex_shader = """
  # #version 400
  
  # layout(location = 0) in vec4 in_Position;
  # layout(location = 1) in vec4 in_Color;

  # void main(void)
  # {
  #   gl_Position = in_Position;
  #   ex_Color = in_Color;
  # }
  # """

  # fragmnet_shader = """
  # #version 400

  # in vec4 ex_Color;
  # out vec4 out_Color;

  # void main(void)
  # {
  #   out_Color = ex_Color;
  # }
  # """

  def __init__(self):
    QGLWidget.__init__(self, None)
    pass

  def initializeGL(self):
    self.create_shader()
    self.create_vbo()
    glUseProgram(self.shader)
    glClearColor(0.0, 0.0, 0.0, 0.0)
    # try:
    #   self.vbo.bind()
    #   try:
    #     glEnableClientState(GL_VERTEX_ARRAY)
    #     glEnableClientState(GL_COLOR_ARRAY)
    #     glVertexPointer(3, GL_FLOAT, 24, self.vbo)
    #     glColorPointer(3, GL_FLOAT, 24, self.vbo + 12)
    #     glDrawArrays(GL_TRIANGLES, 0, 3)
    #   finally:
    #     self.vbo.unbind()
    #     glDisableClientState(GL_VERTEX_ARRAY)
    #     glDisableClientState(GL_COLOR_ARRAY)
    # finally:
    #   glUseProgram(0)

    # rospy.logwarn("DOING SHIT") 
    pass

  def paintGL(self):
    # rospy.logwarn("DOING SHIT")
    # glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    try:
      self.vbo.bind()
      try:
        glEnableClientState(GL_VERTEX_ARRAY)
        glEnableClientState(GL_COLOR_ARRAY)
        glVertexPointer(3, GL_FLOAT, 24, self.vbo)
        glColorPointer(3, GL_FLOAT, 24, self.vbo + 12)
        glDrawArrays(GL_TRIANGLES, 0, 3)
      finally:
        self.vbo.unbind()
        glDisableClientState(GL_VERTEX_ARRAY)
        glDisableClientState(GL_COLOR_ARRAY)
    finally:
      glUseProgram(0)
    pass

  def resizeGL(self, length, width):
    rospy.logwarn("DOING SHIT11")
    glViewport(0, 0, length, width)
    pass

  def create_vbo(self):
    # vertices = numpy.array([-0.8, -0.8, 0.0, 1.0,
    #             0.0, 0.8, 0.0, 1.0,
    #             0.8, -0.8, 0.0, 1.0], 'f')

    # colors = numpy.array([1.0, 0.0, 0.0, 1.0,
    #           0.0, 1.0, 0.0, 1.0,
    #           0.0, 0.0, 1.0, 1.0], 'f')

    # check_error_value = glGetError()

    # self.VaoId = glGenVertexArrays(1)
    # glBindVertexArray(self.VaoId)

    # self.VboId = glGenBuffers(1)
    # glBindBuffer(GL_ARRAY_BUFFER, self.VboId)
    # glBufferData(GL_ARRAY_BUFFER, sys.getsizeof(vertices), vertices, GL_STATIC_DRAW)
    # glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 0, 0)
    # glEnableVertexAttribArray(0)

    # self.ColorBufferId = glGenBuffers(1)
    # glBindBuffer(GL_ARRAY_BUFFER, self.ColorBufferId)
    # glBufferData(GL_ARRAY_BUFFER, sys.getsizeof(colors), colors, GL_STATIC_DRAW)
    # glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 0, 0)
    # glEnableVertexAttribArray(1)

    # check_error_value = glGetError()
    # if check_error_value != GL_NO_ERROR:
    #   rospy.logerr(self.name_ + "Could not create VBO")
    #   sys.exit(1)
    self.vbo = vbo.VBO(
      numpy.array([
        [-0.8, -0.8, 0.0, 1.0, 0.0, 0.0],
        [0.0, 0.8, 0.0, 0.0, 1.0, 0.0],
        [0.8, -0.8, 0.0, 0.0, 0.0, 1.0]
        ], 'f')
      )
    pass

  def create_shader(self):
    # check_error_value = glGetError()

    # self.VertexShaderId = glCreateShader(GL_VERTEX_SHADER)
    # glShaderSource(self.VertexShaderId, self.vertex_shader)
    # glCompileShader(self.VertexShaderId)

    # self.FragmentShaderId = glCreateShader(GL_FRAGMENT_SHADER)
    # glShaderSource(self.FragmentShaderId, self.fragmnet_shader)
    # glCompileShader(self.FragmentShaderId)

    # self.ProgramId = glCreateProgram()
    # glAttachShader(self.ProgramId, self.VertexShaderId)
    # glAttachShader(self.ProgramId, self.FragmentShaderId)
    # glLinkProgram(self.ProgramId)
    # glUseProgram(self.ProgramId)

    # check_error_value = glGetError()
    # if check_error_value != GL_NO_ERROR:
    #   rospy.logerr(self.name_ + "Could not create shader")
    #   sys.exit(1)

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
    pass