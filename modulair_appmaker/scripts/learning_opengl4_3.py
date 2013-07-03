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


class Cross(QGLWidget):

  def __init__(self):
    QGLWidget.__init__(self, None)
    pass

  def initializeGL(self):
    # self.create_shader()
    self.create_vbo()
    self.create_shader()
    
    glClearColor(0.0, 0.0, 0.0, 0.0)
    pass

  def paintGL(self):    
    rospy.logwarn("DONE")
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    try:
      self.vertices.bind()
      self.indices.bind()
      try:
        glEnableClientState(GL_VERTEX_ARRAY)
        glEnableClientState(GL_COLOR_ARRAY)
        glVertexPointer(3, GL_FLOAT, 24, self.vertices)
        glColorPointer(3, GL_FLOAT, 24, self.vertices + 12)
        # glPointSize(50)
        # glPolygonMode(GL_FRONT_AND_BACK, GL_POINT)
        glDrawElements(GL_TRIANGLES, 48, GL_UNSIGNED_BYTE, self.indices)
      finally:
        self.vertices.unbind()
        self.indices.unbind()
        glDisableClientState(GL_VERTEX_ARRAY)
        glDisableClientState(GL_COLOR_ARRAY)
    finally:
      glUseProgram(0)
    pass

  def resizeGL(self, length, width):
    glViewport(0, 0, length, width)
    pass

  def create_vbo(self):
    self.vertices = vbo.VBO(
      numpy.array([
        [0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        # top
        [-0.2, 0.8, 0.0, 0.0, 1.0, 0.0],
        [0.2, 0.8, 0.0, 0.0, 0.0, 1.0],
        [0.0, 0.8, 0.0, 0.0, 1.0, 1.0],
        [0.0, 1.0, 0.0, 1.0, 0.0, 0.0],
        # bottom
        [-0.2, -0.8, 0.0, 0.0, 0.0, 1.0],
        [0.2, -0.8, 0.0, 0.0, 1.0, 0.0],
        [0.0, -0.8, 0.0, 0.0, 1.0, 1.0],
        [0.0, -1.0, 0.0, 1.0, 0.0, 0.0],
        # left
        [-0.8, -0.2, 0.0, 0.0, 1.0, 0.0],
        [-0.8, 0.2, 0.0, 0.0, 0.0, 1.0],
        [-0.8, 0.0, 0.0, 0.0, 1.0, 1.0],
        [-1.0, 0.0, 0.0, 1.0, 0.0, 0.0],
        # right
        [0.8, -0.2, 0.0, 0.0, 0.0, 1.0],
        [0.8, 0.2, 0.0, 0.0, 1.0, 0.0],
        [0.8, 0.0, 0.0, 0.0, 1.0, 1.0],
        [1.0, 0.0, 0.0, 1.0, 0.0, 0.0],
        ], dtype=numpy.float32),
      usage = GL_STATIC_DRAW,
      target = GL_ARRAY_BUFFER
    )

    self.indices = vbo.VBO(
      numpy.array([
      # top
      0, 1, 3,
      0, 3, 2,
      3, 1, 4,
      3, 4, 2,
      # bottom
      0, 5, 7,
      0, 7, 6,
      7, 5, 8,
      7, 8, 6,
      # left
      0, 9, 11,
      0, 11, 10,
      11, 9, 12,
      11, 12, 10,
      # right
      0, 13, 15,
      0, 15, 14,
      15, 13, 16,
      15, 16, 14
      ], dtype=numpy.int8),
      usage = GL_STATIC_DRAW,
      target = GL_ELEMENT_ARRAY_BUFFER,
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