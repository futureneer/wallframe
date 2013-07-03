#!/usr/bin/env python
import sys
import math
import numpy
import time
### ROS Imports ###
import rospy
### PySide Imports ###
from PySide.QtCore import QTimer
from PySide.QtOpenGL import QGLWidget
### PyOpenGL Imports ###
from OpenGL.GL import *
from OpenGL.GL import shaders
### Transformations Import ###
from transformations import Tools3D

class Rot_Cube(QGLWidget):

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
    self.projection_matrix = Tools3D.get_identity()
    self.view_matrix = Tools3D.get_identity()
    self.view_matrix = Tools3D.translate_matrix(self.view_matrix, 0, 0, -2)

    self.create_cube()

    self.timer = QTimer()
    self.timer.timeout.connect(self.updateGL)
    self.timer.start(0)
    pass

  def paintGL(self):
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    self.draw_cube()
    pass

  def resizeGL(self, width, height):
    glViewport(0, 0, width, height)
    self.projection_matrix = Tools3D.create_projection_matrix(60, width / height, 1.0, 100.0)
    glUseProgram(self.shader_program)
    glProgramUniformMatrix4fv(self.shader_program, self.projection_m_uniloc, 1, GL_FALSE, self.projection_matrix)
    glUseProgram(0)
    pass

  def create_cube(self):
    self.vertices = numpy.array([
      [-0.5, -0.5, 0.5, 1,   0, 0, 1, 1],
      [-0.5, 0.5, 0.5, 1,    1, 0, 0, 1],
      [0.5, 0.5, 0.5, 1,     0, 1, 0, 1],
      [0.5, -0.5, 0.5, 1,    1, 1, 0, 1],
      [-0.5, -0.5, -0.5, 1,  1, 1, 1, 1],
      [-0.5, 0.5, -0.5, 1,   1, 0, 0, 1],
      [0.5, 0.5, -0.5, 1,    1, 0, 1, 1],
      [0.5, -0.5, -0.5, 1,   0, 0, 1, 1]
    ], dtype = numpy.float32)

    self.indices = numpy.array([
      0, 2, 1,  0, 3, 2,
      4, 3, 0,  4, 7, 3,
      4, 1, 5,  4, 0, 1,
      3, 6, 2,  3, 7, 6,
      1, 6, 5,  1, 2, 6,
      7, 5, 6,  7, 4, 5
    ], dtype=numpy.int8)

    vertex_shader_src = """
      #version 400

      in vec4 in_Position;
      in vec4 in_Color;
      out vec4 ex_Color;

      uniform mat4 model_matrix;
      uniform mat4 view_matrix;
      uniform mat4 projection_matrix;

      void main(void)
      {
        gl_Position = (projection_matrix * view_matrix * model_matrix) * in_Position;
        ex_Color = in_Color;
        //ex_Color = vec4(0.0, 1.0, 0.0, 1.0);
        //ex_Color = in_Position;
      }
      """
    
    fragment_shader_src = """
      #version 400

      in vec4 ex_Color;
      out vec4 out_Color;

      void main(void)
      {
        out_Color = ex_Color;
      }
      """

    self.vertex_shader = glCreateShader(GL_VERTEX_SHADER)
    glShaderSource(self.vertex_shader, vertex_shader_src)
    glCompileShader(self.vertex_shader)

    self.fragment_shader = glCreateShader(GL_FRAGMENT_SHADER)
    glShaderSource(self.fragment_shader, fragment_shader_src)
    glCompileShader(self.fragment_shader)

    self.shader_program = glCreateProgram()
    glAttachShader(self.shader_program, self.vertex_shader)
    glAttachShader(self.shader_program, self.fragment_shader)
    glLinkProgram(self.shader_program)
# -------------------------------------------------------------------------
    # vertex_shader_src = shaders.compileShader("""
    #   #version 400

    #   in vec4 in_Position;
    #   in vec4 in_Color;
    #   out vec4 ex_Color;

    #   uniform mat4 model_matrix;
    #   uniform mat4 view_matrix;
    #   uniform mat4 projection_matrix;

    #   void main(void)
    #   {
    #     gl_Position = (projection_matrix * view_matrix * model_matrix) * in_Position;
    #     ex_Color = in_Color;
    #     //ex_Color = vec4(0.0, 1.0, 0.0, 1.0);
    #     //ex_Color = in_Position;
    #   }
    #   """, GL_VERTEX_SHADER)
    
    # fragment_shader_src = shaders.compileShader("""
    #   #version 400

    #   in vec4 ex_Color;
    #   out vec4 out_Color;

    #   void main(void)
    #   {
    #     out_Color = ex_Color;
    #   }
    #   """, GL_FRAGMENT_SHADER)

    # self.shader_program = shaders.compileProgram(vertex_shader_src, fragment_shader_src)
# -------------------------------------------------------------------------

    self.model_m_uniloc = glGetUniformLocation(self.shader_program, "model_matrix")
    self.view_m_uniloc = glGetUniformLocation(self.shader_program, "view_matrix")
    self.projection_m_uniloc = glGetUniformLocation(self.shader_program, "projection_matrix")

    self.position_attloc = glGetAttribLocation(self.shader_program, "in_Position")
    self.color_attloc = glGetAttribLocation(self.shader_program, "in_Color")

    rospy.logwarn("POS: "+str(self.position_attloc))
    rospy.logwarn("COL: "+str(self.color_attloc))

    self.vao = glGenVertexArrays(1)
    glBindVertexArray(self.vao)

    glEnableVertexAttribArray(self.position_attloc)
    glEnableVertexAttribArray(self.color_attloc)

    self.vertices_vbo = glGenBuffers(1)
    glBindBuffer(GL_ARRAY_BUFFER, self.vertices_vbo)
    glBufferData(GL_ARRAY_BUFFER, self.vertices, GL_STATIC_DRAW)
    glVertexAttribPointer(self.position_attloc, 4, GL_FLOAT, GL_FALSE, 32, None)
    glVertexAttribPointer(self.color_attloc, 4, GL_FLOAT, GL_FALSE, 32, None)


    self.indices_vbo = glGenBuffers(1)   
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, self.indices_vbo)
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, self.indices, GL_STATIC_DRAW)

    glBindVertexArray(0)
    pass

  def draw_cube(self):
    self.now = time.clock()

    if self.last_time == 0:
      self.last_time = self.now

    self.cube_rotation += 45.0 * float(self.now - self.last_time)
    self.cube_angle = math.radians(self.cube_rotation)
    self.last_time = self.now

    self.model_matrix = Tools3D.get_identity()
    self.model_matrix = Tools3D.rotate_about_y(self.model_matrix, self.cube_angle)
    self.model_matrix = Tools3D.rotate_about_x(self.model_matrix, self.cube_angle)

    glUseProgram(self.shader_program)

    glProgramUniformMatrix4fv(self.shader_program, self.model_m_uniloc, 1, GL_FALSE, self.model_matrix)
    glProgramUniformMatrix4fv(self.shader_program, self.view_m_uniloc, 1, GL_FALSE, self.view_matrix)

    glBindVertexArray(self.vao)

    glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_BYTE, None)

    glBindVertexArray(0)
    glUseProgram(0)
    pass

  def clean_up(self):
    self.timer.stop()
    # glDetachShader(self.shader_program, self.vertex_shader)
    # glDetachShader(self.shader_program, self.fragment_shader)
    # glDeleteShader(self.vertex_shader)
    # glDeleteShader(self.fragment_shader)
    # glDeleteProgram(self.shader_program)

    # glDeleteBuffers
    pass
