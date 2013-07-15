#!/usr/bin/env python

import sys
import math
import glob

from PySide import QtCore, QtGui, QtOpenGL
from PySide.QtCore import QTimer
from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
from PIL.Image import *
from modulair_core import ModulairAppWidget

import Quad 
import rospy

# class Window(QtGui.QWidget):
#     def __init__(self, parent=None):
#         QtGui.QWidget.__init__(self, parent)
#         self.resize(640,480)
#         self.glWidget = GLWidget()

#         mainLayout = QtGui.QHBoxLayout()
#         mainLayout.addWidget(self.glWidget)
#         self.setLayout(mainLayout)        
#         self.setWindowTitle(self.tr("Image Browser"))            


class GLWidget(QtOpenGL.QGLWidget):
    def __init__(self, parent=None):
        QtOpenGL.QGLWidget.__init__(self, parent)    
        self.quad = Quad.Rectangle()
        self.str = "/home/kel/modulair/modulair_appmaker/scripts/*"
        self.file = glob.glob(self.str + ".png")        
        rospy.logwarn(self.file)

    def initializeGL(self):        
    	self.quad.loadTexture("/home/kel/modulair/modulair_appmaker/scripts/music.png")
    	glEnable(GL_TEXTURE_2D)
    	glClearColor(0.0, 0.0, 0.0, 0.0)
    	glClearDepth(1.0)
    	glDepthFunc(GL_LESS)
    	glShadeModel(GL_SMOOTH)
        glEnable(GL_DEPTH_TEST)    	
        glEnable(GL_CULL_FACE)
        self.quad.intializeQuad(0,0,4,4)

    def paintGL(self):

    	self.quad.update()

    def resizeGL(self, width, height):                
    	if height == 0:
            height = 1

    	glViewport(0, 0, width, height)
    	glMatrixMode(GL_PROJECTION)
    	glLoadIdentity()
        gluPerspective(45.0, float(width)/float(height), 1.0, 100.0)
    	glMatrixMode(GL_MODELVIEW)    


# if __name__ == '__main__':
#     app = QtGui.QApplication(sys.argv)
#     window = Window()
#     window.show()
#     sys.exit(app.exec_())
