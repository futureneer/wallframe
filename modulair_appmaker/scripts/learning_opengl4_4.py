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
import math
import time

class Tools3D:

  IDENTITY_MATRIX = [1, 0, 0, 0,
                    0, 1, 0, 0,
                    0, 0, 1, 0,
                    0, 0, 0, 1]

  def __init__(self):
    pass

  @staticmethod
  def get_identity():
    return list(Tools3D.IDENTITY_MATRIX)

  @staticmethod
  def multiply_matrices(matrix_one, matrix_two):
    out = Tools3D.get_identity()

    row = row_offset = column = 0
    while row < 4:
      column = 0
      while column < 4:
        out[row_offset + column] = ((matrix_one[row_offset + 0] * matrix_two[column + 0]) +
                                    (matrix_one[row_offset + 1] * matrix_two[column + 4]) +
                                    (matrix_one[row_offset + 2] * matrix_two[column + 8]) +
                                    (matrix_one[row_offset + 3] * matrix_two[column + 12]))
        column +=1
      row += 1
      row_offset = row * 4

    return out

  @staticmethod
  def scale_matrix(matrix, x, y, z):
    scale = Tools3D.get_identity()

    scale[0] = x
    scale[5] = y
    scale[10] = z

    return Tools3D.multiply_matrices(matrix, scale) 

  @staticmethod
  def translate_matrix(matrix, x, y, z):
    translation = Tools3D.get_identity()

    translation[12] = x
    translation[13] = y
    translation[14] = z

    return Tools3D.multiply_matrices(matrix, translation)

  @staticmethod
  def rotate_about_x(matrix, angle):
    rotation = Tools3D.get_identity()

    sine = math.sin(angle)
    cosine = math.cos(angle)

    rotation[5] = cosine
    rotation[6] = -sine
    rotation[9] = sine
    rotation[10] = cosine

    return Tools3D.multiply_matrices(matrix, rotation)

  @staticmethod
  def rotate_about_y(matrix, angle):
    rotation = Tools3D.get_identity()

    sine = math.sin(angle)
    cosine = math.cos(angle)

    rotation[0] = cosine
    rotation[8] = sine
    rotation[2] = -sine
    rotation[10] = cosine
    
    return Tools3D.multiply_matrices(matrix, rotation)

  @staticmethod
  def rotate_about_z(matrix, angle):
    rotation = Tools3D.get_identity()

    sine = math.sin(angle)
    cosine = math.cosine(angle)

    rotation[0] = cosine
    rotation[1] = -sine
    rotation[4] = sine
    rotation[5] = cosine
    
    return Tools3D.multiply_matrices(matrix, rotation)

  @staticmethod
  def create_projection_matrix(fovy, aspect_ratio, near_plane, far_plane):
    out = [0, 0, 0, 0,
          0, 0, 0, 0,
          0, 0, 0, 0,
          0, 0, 0, 0]

    y_scale = Tools3D.cotangent(math.radians(int(fovy / 2)))
    x_scale = y_scale / aspect_ratio
    frustom_length = far_plane - near_plane

    out[0] = x_scale
    out[5] = y_scale
    out[10] = -int((far_plane + near_plane) / frustom_length)
    out[11] = -1
    out[14] = -int((2 * near_plane * far_plane) / frustom_length)
    
    return out

  @staticmethod
  def cotangent(angle):
    return float(1.0 / math.tan(angle))

class Cube(QGLWidget):

  def __init__(self):
    QGLWidget.__init__(self, None)
    self.now = 0
    self.last_time = 0
    self.cube_rotation = 0
    pass

  def initializeGL(self):
    glClearColor(0.0, 0.0, 0.0, 0.0)

    glEnable(GL_DEPTH_TEST)
    glDepthFunc(GL_LESS)

    glEnable(GL_CULL_FACE)
    glCullFace(GL_BACK)
    glFrontFace(GL_CCW)

    self.model_matrix = Tools3D.get_identity()
    # rospy.logwarn("INIT: Model Matrix: "+str(self.model_matrix))
    self.projection_matrix = Tools3D.get_identity()
    # rospy.logwarn("INIT: Projection Matrix: "+str(self.projection_matrix))
    self.view_matrix = Tools3D.get_identity()
    # rospy.logwarn("INIT: View Matrix: "+str(self.view_matrix))
    self.view_matrix = Tools3D.translate_matrix(self.view_matrix, 0, 0, -2)
    # rospy.logwarn("INIT: View Matrix: "+str(self.view_matrix))

    self.create_cube()
    pass

  def paintGL(self):
    rospy.logwarn("PAINTGL")
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    self.draw_cube()
    pass

  def resizeGL(self, length, width):
    glViewport(0, 0, length, width)
    # rospy.logwarn("RESIZE: Projection Matrix: "+str(self.projection_matrix))
    self.projection_matrix = Tools3D.create_projection_matrix(60, length / width, 1.0, 100.0)
    # rospy.logwarn("RESIZE: Projection Matrix: "+str(self.projection_matrix))
    glUseProgram(self.shader)
    glUniformMatrix4fv(self.projection_matrix_uni_loc, 1, GL_FALSE, self.projection_matrix)
    glUseProgram(0)
    pass

  # def closeEvent(self, event):
  #   glDetachShader(self.shader, self.vertex_shader)
  #   glDetachShader(slef.shader, self.fragment_shader)
  #   glDeleteShader(self.vertex_shader)
  #   glDeleteShader(self.fragment_shader)
  #   glDeleteProgram(self.shader)
  #   self.indices.delete()
  #   self.verties.delete()
  #   event.accept()

  def create_cube(self):
    self.verties = vbo.VBO(
      numpy.array([
        [-0.5, -0.5, 0.5, 1,   0, 0, 1, 1],
        [-0.5, 0.5, 0.5, 1,    1, 0, 0, 1],
        [0.5, 0.5, 0.5, 1,     0, 1, 0, 1],
        [0.5, -0.5, 0.5, 1,    1, 1, 0, 1],
        [-0.5, -0.5, -0.5, 1,  1, 1, 1, 1],
        [-0.5, 0.5, -0.5, 1,   1, 0, 0, 1],
        [0.5, 0.5, -0.5, 1,    1, 0, 1, 1],
        [0.5, -0.5, -0.5, 1,   0, 0, 1, 1]
        ], dtype = numpy.float32),
      usage = GL_STATIC_DRAW,
      target = GL_ARRAY_BUFFER
    )

    self.indices = vbo.VBO(
      numpy.array([
        0, 2, 1,  0, 3, 2,
        4, 3, 0,  4, 7, 3,
        4, 1, 5,  4, 0, 1,
        3, 6, 2,  3, 7, 6,
        1, 6, 5,  1, 2, 6,
        7, 5, 6,  7, 4, 5
        ], dtype=numpy.int8),
      usage = GL_STATIC_DRAW,
      target = GL_ELEMENT_ARRAY_BUFFER
    )

    self.vertex_shader = shaders.compileShader("""
      #version 400

      layout(location = 0) in vec4 in_Position;
      layout(location = 1) in vec4 in_Color;
      out vec4 ex_Color;

      uniform mat4 model_matrix;
      uniform mat4 view_matrix;
      uniform mat4 projection_matrix;

      void main(void)
      {
        gl_Position = (projection_matrix * view_matrix * model_matrix) * in_Position;
        ex_Color = in_Color;
      }
      """, GL_VERTEX_SHADER)

    self.fragment_shader = shaders.compileShader("""
      #version 400

      in vec4 ex_Color;
      out vec4 out_Color;

      void main(void)
      {
        out_Color = ex_Color;
      }
      """, GL_FRAGMENT_SHADER)

    self.shader = shaders.compileProgram(self.vertex_shader, self.fragment_shader)
    # glUseProgram(self.shader)

    self.model_matrix_uni_loc = glGetUniformLocation(self.shader, "model_matrix")
    self.view_matrix_uni_loc = glGetUniformLocation(self.shader, "view_matrix")
    self.projection_matrix_uni_loc = glGetUniformLocation(self.shader, "projection_matrix")
    pass

  def destroy_cube(self):
    pass

  def draw_cube(self):
    self.now = time.clock()
    # rospy.logwarn("TIME: "+str(self.now))

    if self.last_time == 0:
      self.last_time = self.now

    self.cube_rotation += 45.0 * float(self.now - self.last_time)
    # rospy.logwarn("ROT: "+str(self.cube_rotation))
    self.cube_angle = math.radians(self.cube_rotation)
    # rospy.logwarn("ANGLE: "+str(self.cube_angle))
    self.last_time = self.now

    self.model_matrix = Tools3D.get_identity()
    # rospy.logwarn("PAINT: Model Matrix i: "+str(self.model_matrix))
    self.model_matrix = Tools3D.rotate_about_y(self.model_matrix, self.cube_angle)
    # rospy.logwarn("PAINT: Model Matrix y: "+str(self.model_matrix))
    self.model_matrix = Tools3D.rotate_about_x(self.model_matrix, self.cube_angle)
    # rospy.logwarn("PAINT: Model Matrix x: "+str(self.model_matrix))

    glUseProgram(self.shader)

    glUniformMatrix4fv(self.model_matrix_uni_loc, 1, GL_FALSE, self.model_matrix)
    glUniformMatrix4fv(self.view_matrix_uni_loc, 1, GL_FALSE, self.view_matrix)

    try:
      self.verties.bind()
      self.indices.bind()
      try:
        glEnableClientState(GL_VERTEX_ARRAY)
        glEnableClientState(GL_COLOR_ARRAY)
        glVertexPointer(4, GL_FLOAT, 32, self.verties)
        glColorPointer(4, GL_FLOAT, 32, self.verties + 16)
        glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_BYTE, self.indices)
      finally:
        self.verties.unbind()
        self.indices.unbind()
        glDisableClientState(GL_VERTEX_ARRAY)
        glDisableClientState(GL_COLOR_ARRAY)
        pass
    finally:
      glUseProgram(0)   
    pass
