#!/usr/bin/env python
"""
    Modulair App Menu
    First prototype of modulair_app_menu. Uses Model-View/Controller approach.
    View and controller are combined because Qt signal/slots intrinsically do not cater
    to a separated view and controller.
    
    @author Andy Tien
"""
import os, collections, sys, math
from PySide.QtGui import QWidget, QApplication, QGridLayout, QLabel, QPushButton
from PySide.QtCore import QSize

class ModulairMenuModel:
    def __init__(self, app_path):
        self.app_path = app_path
        self.applist = []
        self.App = collections.namedtuple('App', ['name', 'folderpath', 'launchpath'])
        self.Viewers = [] 
    
    def getAppList(self):
        """ Clears the applist, and returns an updated version via a call to scan. """
        self.applist = [] #return latest version
        self.scan()
        return self.applist
    
    def scan(self):
        """ Scan the path for application instances. Return a list of paths
        for all the applications contained inside. Uses os.walk to traverse subdirectories
        from the app_path passed to the model. 
        
        Assumes standard ROS package file organization, which has .launch files in a subdirectory
        named "launch".
        """   
        for root, unused_dirs, files in os.walk(self.app_path):
            for f in files:
                if f.endswith(".launch"): #recognized this directory as an application directory
                    assert os.path.basename(root) == "launch"
                    appname = os.path.basename(os.path.dirname(root))
                    launchpath = os.path.join(root, f)
                    self.applist.append(self.App(appname, root, launchpath))
                    
class ModulairMenuView(QWidget):
    def __init__(self, filelocation):
        self.qt_app = QApplication(sys.argv)
        QWidget.__init__(self)
        
        self.model = ModulairMenuModel(filelocation)
        
        # Applications List
        self.appList = sorted(self.model.getAppList())
        self.gridSet = False
        
        self.height = 400
        self.width = 400
        self.setMinimumSize(QSize(self.width,self.height))
        self.setWindowTitle("Modulair")
        
        self.gridLayout = QGridLayout()
        self.assignWidgets()
        
        self.setLayout(self.gridLayout)
        
    def setup_grid(self):
        """
        Sets up the grid size that will represent the applications in the menu. 
        """
        length = len(self.appList) + 1
        n = int(math.ceil(math.sqrt(length)))
        self.max_x = self.max_y = n
        self.cur_x = 0 
        self.cur_y = 0
        self.gridSet = True

    def next_pos(self):
        """ Return a tuple of the next position """
        if not self.gridSet:
            self.setup_grid()
        
        this_x = self.cur_x
        this_y = self.cur_y
        
        self.cur_y += 1
        if self.cur_y == self.max_y:
            self.cur_y = 0
            self.cur_x += 1
        
        return this_x, this_y
        
    def assignWidgets(self):
        for app in self.appList:
            widget = QLabel(app.name + '\n' + app.launchpath)
            nextx, nexty = self.next_pos()
            self.gridLayout.addWidget(widget, nextx, nexty )
        
        widget = QPushButton('Refresh')
        widget.clicked.connect(self.refresh)
        nextx, nexty = self.next_pos()
        self.gridLayout.addWidget(widget, nextx, nexty)
    
    def refresh(self):
        for unused_i in range(0, len(self.appList)+1):
            g = self.gridLayout.takeAt(0)
            g.widget().deleteLater()
        self.appList = sorted(self.model.getAppList())
        self.gridSet = False
        self.assignWidgets()
        
    def run(self):
        self.show()
        self.qt_app.exec_()
        
def test():
    filelocation = '/home/nd/Dropbox/Workspaces/EclipseWorkspace/MenuApp/Apps'
    menuview = ModulairMenuView(filelocation)
    menuview.setup_grid()
    menuview.run()

if __name__ == "__main__":
    test() 