#!/usr/bin/env python
import math

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