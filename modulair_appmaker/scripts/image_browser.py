#!/usr/bin/env python

import os
import re
import sys
import math

from PySide import QtCore, QtGui, QtOpenGL
from PySide.QtCore import QTimer
from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
from PIL.Image import *
from modulair_core import ModulairAppWidget

import Quad 
import rospy        

class GLWidget(QtOpenGL.QGLWidget):

    def __init__(self, parent=None):
        QtOpenGL.QGLWidget.__init__(self, parent)    
        
        self.str = "/home/kel/modulair/modulair_appmaker/scripts/pics/"
        
        self.img_file_formats = ['.jpg', '.png', '.jpeg', '.bmp']
        self.img_file_locations = []
        self.images = []
        self.raw_images = []

        self.find_image_files(self.str)
        self.load_image_files(self.img_file_locations)
        self.num_images = len(self.images)
        self.current_index = 0
        
        self.quad = Quad.Rectangle()
        pass

    def initializeGL(self):        
    	self.quad.loadTextures(self.images, self.raw_images)
    	glEnable(GL_TEXTURE_2D)
    	glClearColor(0.0, 0.0, 0.0, 0.0)
    	glClearDepth(1.0)
    	glDepthFunc(GL_LESS)
    	glShadeModel(GL_SMOOTH)
        glEnable(GL_DEPTH_TEST)    	
        glEnable(GL_CULL_FACE)
        self.quad.intializeQuad(0,0,4,4)
        pass

    def paintGL(self):
    	self.quad.update()
        pass

    def resizeGL(self, width, height):                
    	if height == 0:
            height = 1

    	glViewport(0, 0, width, height)
    	glMatrixMode(GL_PROJECTION)
    	glLoadIdentity()
        gluPerspective(45.0, float(width)/float(height), 1.0, 100.0)
    	glMatrixMode(GL_MODELVIEW)


    def next_image(self):
        self.current_index = (self.current_index + 1) % self.num_images
        self.quad.set_texture_index(self.current_index)
        self.updateGL()
        rospy.logwarn("REQUEST: " + str(self.current_index))
        pass

    def prev_image(self):
        self.current_index = (self.current_index - 1) % self.num_images
        self.quad.set_texture_index(self.current_index)
        self.updateGL()
        rospy.logwarn("REQUEST: " + str(self.current_index))
        pass

    def add_file_format(self, file_type):
        self.img_file_formats.append(file_type)
        pass

    def find_image_files(self, file_path):
        files = os.listdir(file_path)

        file_regex = ''
        for file_ext in self.img_file_formats:
            file_regex += '.*' + file_ext + '|'

        for img in files:
            if re.match(file_regex, img):
                self.img_file_locations.append(file_path + img)
        pass

    def load_image_files(self, img_files):
        for img in img_files:
            tmp = open(img)
            self.images.append(tmp)
            self.raw_images.append(tmp.tostring("raw", "RGBX", 0, -1))
        pass