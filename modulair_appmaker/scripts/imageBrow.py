#!/usr/bin/env python

import sys
import math
import glob

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
        self.texture = []
        self.texcount = 0

        self.files=[]
        self.counter=0
        self.imFormat = [".jpeg",".bmp","png",".jpg"]
        self.loadImages()

        self.timer1 = QTimer()
        self.timer1.timeout.connect(self.updateGL)                
        self.timer1.start(0)

        self.timer2 = QTimer()
        self.timer2.timeout.connect(self.incrementCount)
        self.timer2.start(1000)

    def loadImages(self):            
        for f in os.listdir("."):
            for i in range(0,len(self.imFormat)):
                if f.endswith(self.imFormat[i]):
                    self.files.append(f)

    def addImageFormat(self, format):
        self.imFormat.append(format)
        pass

    def loadTextures(self):
        for x in range(0,4):            
            self.image = open(self.files[x])
            self.ix = self.image.size[0]
            self.iy = self.image.size[1]
            self.image = self.image.tostring("raw", "RGBX", 0, -1)

            # Create Texture
            # There does not seem to be support for this call or the version of PyOGL I have is broken.
            self.texture.append(glGenTextures(1))
            glBindTexture(GL_TEXTURE_2D, self.texture[x])   # 2d texture (x and y size)

            glPixelStorei(GL_UNPACK_ALIGNMENT,1)
            glTexImage2D(GL_TEXTURE_2D, 0, 3, self.ix, self.iy, 0, GL_RGBA, GL_UNSIGNED_BYTE, self.image)    	
            glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)

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

    # def mousePressEvent(self, event):
    #     self.counter = self.counter + 1
    #     if self.counter == len(self.files):
    #         self.counter = 0
    #     self.loadTextures()
    #     self.updateGL()        

    def paintGL(self):        
    	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    	glLoadIdentity()
    	glTranslatef(0.0, 0.0, -5.0)

    	glRotatef(self.xrot, 1.0, 0.0, 0.0)
    	glRotatef(self.yrot, 0.0, 1.0, 0.0)
    	glRotatef(self.zrot, 0.0, 0.0, 1.0)
        
        glBindTexture(GL_TEXTURE_2D,self.texture[self.texcount])

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

    	# Back Face
    	glTexCoord2f(1.0, 0.0)
    	glVertex3f(-1.0, -1.0, -1.0)	# Bottom Right Of The Texture and Quad
    	glTexCoord2f(1.0, 1.0)
    	glVertex3f(-1.0,  1.0, -1.0)	# Top Right Of The Texture and Quad
    	glTexCoord2f(0.0, 1.0)
    	glVertex3f( 1.0,  1.0, -1.0)	# Top Left Of The Texture and Quad
    	glTexCoord2f(0.0, 0.0)
    	glVertex3f( 1.0, -1.0, -1.0)	# Bottom Left Of The Texture and Quad

    	# Top Face
    	glTexCoord2f(0.0, 1.0)
    	glVertex3f(-1.0,  1.0, -1.0)	# Top Left Of The Texture and Quad
    	glTexCoord2f(0.0, 0.0)
    	glVertex3f(-1.0,  1.0,  1.0)	# Bottom Left Of The Texture and Quad
    	glTexCoord2f(1.0, 0.0)
    	glVertex3f( 1.0,  1.0,  1.0)	# Bottom Right Of The Texture and Quad
    	glTexCoord2f(1.0, 1.0)
    	glVertex3f( 1.0,  1.0, -1.0)	# Top Right Of The Texture and Quad

    	# Bottom Face
    	glTexCoord2f(1.0, 1.0)
    	glVertex3f(-1.0, -1.0, -1.0)	# Top Right Of The Texture and Quad
    	glTexCoord2f(0.0, 1.0)
    	glVertex3f( 1.0, -1.0, -1.0)	# Top Left Of The Texture and Quad
    	glTexCoord2f(0.0, 0.0)
    	glVertex3f( 1.0, -1.0,  1.0)	# Bottom Left Of The Texture and Quad
    	glTexCoord2f(1.0, 0.0)
    	glVertex3f(-1.0, -1.0,  1.0)	# Bottom Right Of The Texture and Quad

    	# Right face
    	glTexCoord2f(1.0, 0.0)
    	glVertex3f( 1.0, -1.0, -1.0)	# Bottom Right Of The Texture and Quad
    	glTexCoord2f(1.0, 1.0)
    	glVertex3f( 1.0,  1.0, -1.0)	# Top Right Of The Texture and Quad
    	glTexCoord2f(0.0, 1.0)
    	glVertex3f( 1.0,  1.0,  1.0)	# Top Left Of The Texture and Quad
    	glTexCoord2f(0.0, 0.0)
    	glVertex3f( 1.0, -1.0,  1.0)	# Bottom Left Of The Texture and Quad

    	# Left Face
    	glTexCoord2f(0.0, 0.0)
    	glVertex3f(-1.0, -1.0, -1.0)	# Bottom Left Of The Texture and Quad
    	glTexCoord2f(1.0, 0.0)
    	glVertex3f(-1.0, -1.0,  1.0)	# Bottom Right Of The Texture and Quad
    	glTexCoord2f(1.0, 1.0)
    	glVertex3f(-1.0,  1.0,  1.0)	# Top Right Of The Texture and Quad
    	glTexCoord2f(0.0, 1.0)
    	glVertex3f(-1.0,  1.0, -1.0)	# Top Left Of The Texture and Quad

    	glEnd();				# Done Drawing The Cube
    	
    	self.xrot = self.xrot + 0.2;
    	self.yrot = self.yrot + 0.2;
    	self.zrot = self.zrot + 0.2;

    def resizeGL(self, width, height):                
    	if height == 0:
            height = 1

    	glViewport(0, 0, width, height)
    	glMatrixMode(GL_PROJECTION)
    	glLoadIdentity()
    	# glOrtho(-0.1, +0.1, +0.1, -0.1, 4.0, 15.0)
        gluPerspective(45.0, float(width)/float(height), 0.1, 100.0)
    	glMatrixMode(GL_MODELVIEW)

    def cleanUp(self):
        self.timer1.stop()
        self.timer2.stop()

    def incrementCount(self):        
        self.texcount = (self.texcount + 1) % 4



if __name__ == '__main__':
    app = QtGui.QApplication(sys.argv)
    window = Window()
    window.show()
    sys.exit(app.exec_())
