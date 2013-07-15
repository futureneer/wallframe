#!/usr/bin/env python

import sys
import math

from PySide import QtCore, QtGui, QtOpenGL
from PySide.QtCore import QTimer
from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
from Image import *

class Window(QtGui.QWidget):
    def __init__(self, parent=None):
        QtGui.QWidget.__init__(self, parent)
        self.resize(640,480)
        self.glWidget = GLWidget()

        mainLayout = QtGui.QHBoxLayout()
        mainLayout.addWidget(self.glWidget)
        self.setLayout(mainLayout)        
        self.setWindowTitle(self.tr("Image Browser"))

    def closeEvent(self, event):
        self.glWidget.cleanUp()
        event.accept()


class GLWidget(QtOpenGL.QGLWidget):
    def __init__(self, parent=None):
        QtOpenGL.QGLWidget.__init__(self, parent)

        self.xrot = 0
        self.yrot = 0
        self.zrot = 0
        self.ix = 0
        self.iy = 0
        self.texture = 0

        self.timer = QTimer()
        self.timer.timeout.connect(self.updateGL)
        self.timer.start(0)
        
    def loadTextures(self):
    	self.image = open("NeHe.bmp")
    	self.ix = self.image.size[0]
    	self.iy = self.image.size[1]
    	self.image = self.image.tostring("raw", "RGBX", 0, -1)

    	# Create Texture
    	# There does not seem to be support for this call or the version of PyOGL I have is broken.
    	glGenTextures(1, self.texture)
    	glBindTexture(GL_TEXTURE_2D, self.texture)   # 2d texture (x and y size)

    	glPixelStorei(GL_UNPACK_ALIGNMENT,1)
    	glTexImage2D(GL_TEXTURE_2D, 0, 3, self.ix, self.iy, 0, GL_RGBA, GL_UNSIGNED_BYTE, self.image)
    	# glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP)
    	# glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP)
    	# glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT)
    	# glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT)
    	# glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST)
    	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)
    	# glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL)

    def initializeGL(self):
    	self.loadTextures()
    	glEnable(GL_TEXTURE_2D)
    	glClearColor(0.0, 0.0, 0.0, 0.0)
    	glClearDepth(1.0)
    	glDepthFunc(GL_LESS)
    	glShadeModel(GL_SMOOTH)
        glEnable(GL_DEPTH_TEST)    	
        glEnable(GL_CULL_FACE)

        # glMatrixMode(GL_PROJECTION)
        # glLoadIdentity()                    # Reset The Projection Matrix

        # # Calculate The Aspect Ratio Of The Window
        # gluPerspective(45.0, float(Width)/float(Height), 0.1, 100.0)
        # glMatrixMode(GL_MODELVIEW)

    def paintGL(self):

    	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    	glLoadIdentity()
    	glTranslatef(0.0, 0.0, -5.0)

    	# glRotatef(self.xrot, 1.0, 0.0, 0.0)
    	# glRotatef(self.yrot, 0.0, 1.0, 0.0)
    	# glRotatef(self.zrot, 0.0, 0.0, 1.0)

        glBindTexture(GL_TEXTURE_2D,self.texture)

    	glBegin(GL_QUADS)


    	# Front Face (note that the texture's corners have to match the quad's corners)
    	glTexCoord2f(0.0, 0.0)
    	glVertex3f(-1.0, -1.0,  1.0)	# Bottom Left Of The Texture and Quad
    	glTexCoord2f(1.0, 0.0)
    	glVertex3f( 1.0, -1.0,  1.0)	# Bottom Right Of The Texture and Quad
    	glTexCoord2f(1.0, 1.0)
    	glVertex3f( 1.0,  1.0,  1.0)	# Top Right Of The Texture and Quad
    	glTexCoord2f(0.0, 1.0)
    	glVertex3f(-1.0,  1.0,  1.0)	# Top Left Of The Texture and Quad

    	# # Back Face
    	# glTexCoord2f(1.0, 0.0)
    	# glVertex3f(-1.0, -1.0, -1.0)	# Bottom Right Of The Texture and Quad
    	# glTexCoord2f(1.0, 1.0)
    	# glVertex3f(-1.0,  1.0, -1.0)	# Top Right Of The Texture and Quad
    	# glTexCoord2f(0.0, 1.0)
    	# glVertex3f( 1.0,  1.0, -1.0)	# Top Left Of The Texture and Quad
    	# glTexCoord2f(0.0, 0.0)
    	# glVertex3f( 1.0, -1.0, -1.0)	# Bottom Left Of The Texture and Quad

    	# # Top Face
    	# glTexCoord2f(0.0, 1.0)
    	# glVertex3f(-1.0,  1.0, -1.0)	# Top Left Of The Texture and Quad
    	# glTexCoord2f(0.0, 0.0)
    	# glVertex3f(-1.0,  1.0,  1.0)	# Bottom Left Of The Texture and Quad
    	# glTexCoord2f(1.0, 0.0)
    	# glVertex3f( 1.0,  1.0,  1.0)	# Bottom Right Of The Texture and Quad
    	# glTexCoord2f(1.0, 1.0)
    	# glVertex3f( 1.0,  1.0, -1.0)	# Top Right Of The Texture and Quad

    	# # Bottom Face
    	# glTexCoord2f(1.0, 1.0)
    	# glVertex3f(-1.0, -1.0, -1.0)	# Top Right Of The Texture and Quad
    	# glTexCoord2f(0.0, 1.0)
    	# glVertex3f( 1.0, -1.0, -1.0)	# Top Left Of The Texture and Quad
    	# glTexCoord2f(0.0, 0.0)
    	# glVertex3f( 1.0, -1.0,  1.0)	# Bottom Left Of The Texture and Quad
    	# glTexCoord2f(1.0, 0.0)
    	# glVertex3f(-1.0, -1.0,  1.0)	# Bottom Right Of The Texture and Quad

    	# # Right face
    	# glTexCoord2f(1.0, 0.0)
    	# glVertex3f( 1.0, -1.0, -1.0)	# Bottom Right Of The Texture and Quad
    	# glTexCoord2f(1.0, 1.0)
    	# glVertex3f( 1.0,  1.0, -1.0)	# Top Right Of The Texture and Quad
    	# glTexCoord2f(0.0, 1.0)
    	# glVertex3f( 1.0,  1.0,  1.0)	# Top Left Of The Texture and Quad
    	# glTexCoord2f(0.0, 0.0)
    	# glVertex3f( 1.0, -1.0,  1.0)	# Bottom Left Of The Texture and Quad

    	# # Left Face
    	# glTexCoord2f(0.0, 0.0)
    	# glVertex3f(-1.0, -1.0, -1.0)	# Bottom Left Of The Texture and Quad
    	# glTexCoord2f(1.0, 0.0)
    	# glVertex3f(-1.0, -1.0,  1.0)	# Bottom Right Of The Texture and Quad
    	# glTexCoord2f(1.0, 1.0)
    	# glVertex3f(-1.0,  1.0,  1.0)	# Top Right Of The Texture and Quad
    	# glTexCoord2f(0.0, 1.0)
    	# glVertex3f(-1.0,  1.0, -1.0)	# Top Left Of The Texture and Quad

    	glEnd();				# Done Drawing The Cube
    	
    	self.xrot = self.xrot + 0.2;
    	self.yrot = self.yrot + 0.2;
    	self.zrot = self.zrot + 0.2;

    def resizeGL(self, width, height):                
    	if height == 0:
            height = 1
    	print("width "+ str(width) + " height " + str(height))
    	glViewport(0, 0, width, height)
    	glMatrixMode(GL_PROJECTION)
    	glLoadIdentity()
    	# glOrtho(-0.1, +0.1, +0.1, -0.1, 4.0, 15.0)
        gluPerspective(45.0, float(width)/float(height), 0.1, 100.0)
    	glMatrixMode(GL_MODELVIEW)

    def cleanUp(self):
        self.timer.stop()


if __name__ == '__main__':
    app = QtGui.QApplication(sys.argv)
    window = Window()
    window.show()
    sys.exit(app.exec_())
