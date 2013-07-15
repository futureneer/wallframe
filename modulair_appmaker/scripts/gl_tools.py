#!/usr/bin/env python

import math
import ctypes
from OpenGL.GL import *

class MatrixTools:

	MATRIX = ctypes.c_float * 16

	@staticmethod
	def get_identity():
		return MatrixTools.MATRIX(1, 0, 0, 0,
					0, 1, 0, 0,
					0, 0, 1, 0,
					0, 0, 0, 1)

	@staticmethod
	def matrix_cpy(dst_matrix, src_matrix):
		ctypes.memmove(ctypes.addressof(dst_matrix), src_matrix, ctypes.sizeof(src_matrix))
		pass

	@staticmethod
	def print_matrix(matrix, name="Matrix"):
		out = (str('\n' + name + ':').ljust(20) + '[')
		for val in matrix:
			out += str(val) + ' '
		out += ']'
		print out

	@staticmethod
	def multiply_matrices(matrix_one, matrix_two):
		out = MatrixTools.get_identity()

		row = row_offset = column = 0
		while row < 4:
			column = 0
			while column < 4:
				out[row_offset + column] = ((matrix_one[row_offset + 0] * matrix_two[column + 0]) +
											(matrix_one[row_offset + 1] * matrix_two[column + 4]) +
											(matrix_one[row_offset + 2] * matrix_two[column + 8]) +
											(matrix_one[row_offset + 3] * matrix_two[column + 12]))
				column += 1
			row += 1
			row_offset = row * 4

		return out

	@staticmethod
	def scale_matrix(matrix, x, y, z):
		scale = MatrixTools.get_identity()

		translation[12] = x
		translation[13] = y
		translation[14] = z

		MatrixTools.matrix_cpy(matrix, MatrixTools.multiply_matrices(matrix, translation))
		pass

	@staticmethod
	def translate_matrix(matrix, x, y, z):
		translation = MatrixTools.get_identity()

		translation[12] = x
		translation[13] = y
		translation[14] = z

		return MatrixTools.matrix_cpy(matrix, MatrixTools.multiply_matrices(matrix, translation))

	@staticmethod
	def rot_about_x(matrix, angle):
		rotation = MatrixTools.get_identity()

		sine = math.sin(angle)
		cosine = math.cos(angle)

		rotation[5] = cosine
		rotation[6] = -sine
		rotation[9] = sine
		rotation[10] = cosine

		MatrixTools.matrix_cpy(matrix, MatrixTools.multiply_matrices(matrix, rotation))
		pass

	@staticmethod
	def rot_about_y(matrix, angle):
		rotation = MatrixTools.get_identity()

		sine = math.sin(angle)
		cosine = math.cos(angle)

		rotation[0] = cosine
		rotation[8] = sine
		rotation[2] = -sine
		rotation[10] = cosine

		MatrixTools.matrix_cpy(matrix, MatrixTools.multiply_matrices(matrix, rotation))
		pass

	@staticmethod
	def rot_about_z(matrix, angle):
		rotation = MatrixTools.get_identity()

		sine = math.sin(angle)
		cosine = math.cos(angle)

		rotation[0] = cosine
		rotation[1] = -sine
		rotation[4] = sine
		rotation[5] = cosine

		MatrixTools.matrix_cpy(matrix, MatrixTools.multiply_matrices(matrix, rotation))
		pass

	@staticmethod
	def get_projection_matrix(matrix, fovy, aspect_ratio, near_plane, far_plane):
		out = MatrixTools.MATRIX()

		y_scale = MatrixTools.cotangent(math.radians(int(fovy / 2)))
		x_scale = y_scale / aspect_ratio
		frustom_length = far_plane - near_plane

		out[0] = x_scale
		out[5] = y_scale
		out[10] = -int((far_plane + near_plane) / frustom_length)
		out[11] = -1
		out[14] = -int((2 * near_plane * far_plane) / frustom_length)

		MatrixTools.matrix_cpy(matrix, out)

	@staticmethod
	def cotangent(angle):
		return float(1.0 / math.tan(angle))

class GLTools:

	@staticmethod
	def get_opengl_info():
		vendor = str(glGetString(GL_VENDOR))
		gl_version = str(glGetString(GL_VERSION))
		glsl_version = str(glGetString(GL_SHADING_LANGUAGE_VERSION))

		info = ('\nVendor:'.ljust(20) + vendor + \
				'\nOpenGL Version:'.ljust(20) + gl_version + \
				'\nGLSL Version:'.ljust(20) + glsl_version)
		return info

	@staticmethod
	def format_info(type, info):
		return str('\n' + type + ':').ljust(20) + str(info)

	@staticmethod
	def flatten_3(lll):
		return [item for ll in lll for l in ll for item in l]

	@staticmethod
	def flatten_2(ll):
		return [item for l in ll for item in l]

	@staticmethod
	def flatten_any(n_list):
		if isinstance(nlist[0], list):
			return flatten_any(flatten_2(n_list))
		else:
			return n_list

	@staticmethod
	def get_offset_ptr(offset, data_type):
		return ctypes.c_void_p(offset * GLTools.get_ctype_size(data_type))

	@staticmethod
	def get_ctype_size(type):
		return ctypes.sizeof(GLTools.get_ctype(type))

	@staticmethod
	def get_ctype(data_type = 'c_int'):
		if data_type == 'byte':
			c_type = ctypes.c_byte
		elif data_type == 'u_byte':
			c_type = ctypes.c_ubyte
		elif data_type == 'short':
			c_type == ctypes.c_short
		elif data_type == 'u_short':
			c_type == ctypes.c_ushort
		elif data_type == 'int':
			c_type = ctypes.c_int
		elif data_type == 'u_nit':
			c_type = ctypes.c_unit
		elif data_type == 'float':
			c_type = ctypes.c_float
		elif data_type == 'double':
			c_type = ctypes.c_double
		else:
			c_type = ctypes.c_int

		return c_type

	@staticmethod
	def convert_data(data, data_type='c_int'):
		c_type = GLTools.get_ctype(data_type)

		return (c_type * len(data))(*data)

	@staticmethod
	def create_vbo(data, data_type, target_buffer = GL_ARRAY_BUFFER, buffer_usg = GL_STATIC_DRAW):
		c_type_buffer = GLTools.convert_data(data, data_type)

		vbo = glGenBuffers(1)
		glBindBuffer(target_buffer, vbo)
		glBufferData(target_buffer, c_type_buffer, buffer_usg)

		return vbo

	@staticmethod
	def load_shader(shdr_src, shdr_type):
		pass

	@staticmethod
	def get_shader_info(shader, name='Shader'):
		out = '\n' + name
		if glGetShaderiv(shader, GL_COMPILE_STATUS) != GL_TRUE:
			out += ': failed to compile.\n' + str(glGetShaderInfoLog(shader))
			return (False, out)
		out += ': compiled properly.'
		return (True, out)

	@staticmethod
	def get_program_info(program, name='Program'):
		out = '\n' + name
		if glGetProgramiv(program, GL_LINK_STATUS) != GL_TRUE:
			out += ': failed to link.\n' + str(glGetProgramInfoLog(program))
			return (False, out)
		out += ': linked properly.'
		return (True, out)