#!/usr/bin/env python

### Python Imports ###
import math
import time
### PySide Imports ###
from PySide.QtCore import QTimer
from PySide.QtOpenGL import QGLWidget
### PyOpenGL Imports ###
from OpenGL.GL import *
### Utility Imports ###
from gl_tools import MatrixTools, GLTools

class Cube(QGLWidget):

	def __init__(self):
		QGLWidget.__init__(self, None)

		self.now = 0
		self.last_time = 0
		self.cube_rotation = 0

	def initializeGL(self):
		print GLTools.get_opengl_info()

		glClearColor(0.0, 0.0, 0.0, 0.0)
		glEnable(GL_DEPTH_TEST)
		glDepthFunc(GL_LESS)

		glEnable(GL_CULL_FACE)
		glCullFace(GL_BACK)
		glFrontFace(GL_CCW)

		self.model_matrix = MatrixTools.get_identity()
		MatrixTools.print_matrix(self.model_matrix, 'MODEL')
		
		self.projection_matrix = MatrixTools.get_identity()
		MatrixTools.print_matrix(self.projection_matrix, 'PROJECTION')
		
		self.view_matrix = MatrixTools.get_identity()
		MatrixTools.print_matrix(self.view_matrix, 'VIEW')

		MatrixTools.translate_matrix(self.view_matrix, 0, 0, -2)
		MatrixTools.print_matrix(self.view_matrix, 'VIEW')

		self.create_cube()

		self.timer = QTimer()
		self.timer.timeout.connect(self.updateGL)
		self.timer.start(0)
		pass

	def resizeGL(self, width, height):
		glViewport(0, 0, width, height)
		MatrixTools.get_projection_matrix(self.projection_matrix, 60, width / height, 1.0, 100.0)
		glUseProgram(self.shader_program)
		glProgramUniformMatrix4fv(self.shader_program, self.projection_m_uniloc, 1, GL_FALSE, self.projection_matrix)
		glUseProgram(0)
		pass

	def paintGL(self):
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
		self.draw_cube()
		pass

	def create_cube(self):
		vertices = [
		[[-0.5, -0.5, 0.5, 1.0], 	[0.0, 0.0, 1.0, 1.0]],
		[[-0.5, 0.5, 0.5, 1.0], 	[1.0, 0.0, 0.0, 1.0]],
		[[0.5, 0.5, 0.5, 1.0], 		[0.0, 1.0, 0.0, 1.0]],
		[[0.5, -0.5, 0.5, 1.0], 	[1.0, 1.0, 0.0, 1.0]],
		[[-0.5, -0.5, -0.5, 1.0], 	[1.0, 1.0, 1.0, 1.0]],
		[[-0.5, 0.5, -0.5, 1.0], 	[1.0, 0.0, 0.0, 1.0]],
		[[0.5, 0.5, -0.5, 1.0], 	[1.0, 0.0, 1.0, 1.0]],
		[[0.5, -0.5, -0.5, 1.0], 	[0.0, 0.0, 1.0, 1.0]]
		]

		indices = [
			0, 2, 1,	0, 3, 2,
			4, 3, 0,	4, 7, 3,
			4, 1, 5,	4, 0, 1,
			3, 6, 2,	3, 7, 6,
			1, 6, 5,	1, 2, 6,
			7, 5, 6,	7, 4, 5
		]

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
		glShaderSource(self.vertex_shader, [vertex_shader_src])
		glCompileShader(self.vertex_shader)

		compiled, info = GLTools.get_shader_info(self.vertex_shader, 'Vertex Shader')
		if not compiled:
			print info
			sys.exit(1)

		self.fragment_shader = glCreateShader(GL_FRAGMENT_SHADER)
		glShaderSource(self.fragment_shader, [fragment_shader_src])
		glCompileShader(self.fragment_shader)

		compiled, info = GLTools.get_shader_info(self.fragment_shader, 'Fragment Shader')
		if not compiled:
			print info
			sys.exit(1)

		self.shader_program = glCreateProgram()
		glAttachShader(self.shader_program, self.vertex_shader)
		glAttachShader(self.shader_program, self.fragment_shader)
		glLinkProgram(self.shader_program)

		linked, info = GLTools.get_program_info(self.shader_program, 'Shader Program')
		if not linked:
			print info
			sys.exit(1)

		self.model_m_uniloc = glGetUniformLocation(self.shader_program, "model_matrix")
		self.view_m_uniloc = glGetUniformLocation(self.shader_program, "view_matrix")
		self.projection_m_uniloc = glGetUniformLocation(self.shader_program, "projection_matrix")

		print GLTools.format_info('Model Uniloc', self.model_m_uniloc)
		print GLTools.format_info('View Uniloc', self.view_m_uniloc)
		print GLTools.format_info('Projection Uniloc', self.projection_m_uniloc)

		self.position_attloc = glGetAttribLocation(self.shader_program, "in_Position")
		self.color_attloc = glGetAttribLocation(self.shader_program, "in_Color")

		print GLTools.format_info('Position Attloc', self.position_attloc)
		print GLTools.format_info('Color Attloc', self.color_attloc)

		self.vao = glGenVertexArrays(1)
		glBindVertexArray(self.vao)

		glEnableVertexAttribArray(self.position_attloc)
		glEnableVertexAttribArray(self.color_attloc)

		self.vertices_vbo = GLTools.create_vbo(GLTools.flatten_3(vertices), 'float')

		vertices_offset = GLTools.get_offset_ptr(0, 'float')
		colors_offset 	= GLTools.get_offset_ptr(4, 'float')
		record_len		= 8 * GLTools.get_ctype_size('float')
		glVertexAttribPointer(self.position_attloc, 4, GL_FLOAT, GL_FALSE, record_len, vertices_offset)
		glVertexAttribPointer(self.color_attloc, 4, GL_FLOAT, GL_FALSE, record_len, colors_offset)

		self.indices_vbo = GLTools.create_vbo(indices, 'u_int', target_buffer = GL_ELEMENT_ARRAY_BUFFER)

		glBindVertexArray(0)
		pass

	def draw_cube(self):
		self.now = time.clock()

		if self.last_time == 0:
			self.last_time = self.now

		self.cube_rotation += 45.0 * float(self.now - self.last_time)
		self.cube_angle = math.radians(self.cube_rotation)
		self.last_time = self.now

		self.model_matrix = MatrixTools.get_identity()
		MatrixTools.rot_about_y(self.model_matrix, self.cube_angle)
		MatrixTools.rot_about_x(self.model_matrix, self.cube_angle)

		glUseProgram(self.shader_program)

		glProgramUniformMatrix4fv(self.shader_program, self.model_m_uniloc, 1, GL_FALSE, self.model_matrix)
		glProgramUniformMatrix4fv(self.shader_program, self.view_m_uniloc, 1, GL_FALSE, self.view_matrix)

		glBindVertexArray(self.vao)

		glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_INT, None)

		glBindVertexArray(0)
		glUseProgram(0)
		pass

	def clean_up(self):
		print 'cleaning up'
		self.timer.stop()

		glDetachShader(self.shader_program, self.vertex_shader)
		glDetachShader(self.shader_program, self.fragment_shader)
		glDeleteShader(self.vertex_shader)
		glDeleteShader(self.fragment_shader)
		glDeleteProgram(self.shader_program)

		glDeleteBuffers(1, self.vertices_vbo)
		glDeleteBuffers(1, self.indices_vbo)
		# glDeleteVertexArrays(1, self.vao)
		pass