#!/usr/bin/env python
"""
  Modulair App_ Menu
  First prototype of modulair_app_menu. Uses Model-View/Controller approach.
  View and controller are combined because Qt signal/slots intrinsically do not cater
  to a separated view and controller.


    
  @author Andy Tien, Kel Guerin, Zihan Chen
"""

# TODO ZC
# 
# use QLabel for now + may use modulair_app_menu_button class later
# use QPushbutton
# call launch file when button is pushed
# Ask kel:
#   1. test using real kinect 
#   2. find out correct service call load (app name)
#   3. GUI side 


# ROS import
import roslib; roslib.load_manifest('modulair_core')
import rospy

# system import
import os, collections, sys, math

# PySide import
from PySide.QtGui import * 
from PySide.QtCore import *
from PySide import QtCore

# modulair import
# msg
from modulair_msgs.msg import ModulairUser
from modulair_msgs.msg import ModulairUserArray
from modulair_msgs.msg import ModulairUserEvent
# srv
import modulair_core
from modulair_core.srv import *

from modulair_app_menu_button import ModulairAppButton
from math import fabs


# #=========== MODEL =======================
# class ModulairMenuModel:
#   def __init__(self, app_path):
#     self.app_path_ = app_path
#     self.applist = []
#     self.App = collections.namedtuple('App', ['name', 'folderpath', 'launchpath'])
#     self.Viewers = [] 
  
#   def getAppList(self):
#     """ Clears the applist, and returns an updated version via a call to scan. """
#     self.applist = [] #return latest version
#     self.scan()
#     return self.applist
  
#   def scan(self):
#     """ Scan the path for application instances. Return a list of paths
#     for all the applications contained inside. Uses os.walk to traverse subdirectories
#     from the app_path passed to the model. 
    
#     Assumes standard ROS package file organization, which has .launch files in a subdirectory
#     named "launch".
#     """   
#     for root, unused_dirs, files in os.walk(self.app_path_):
#       for f in files:
#         if f.endswith(".launch"): #recognized this directory as an application directory
#           assert os.path.basename(root) == "launch"
#           appname = os.path.basename(os.path.dirname(root))
#           launchpath = os.path.join(root, f)
#           self.applist.append(self.App(appname, root, launchpath))

class ModulairCursor(QWidget):
  def __init__(self,image,parent):
    super(ModulairCursor,self).__init__(parent)
    self.label_ = QLabel('cursor',self)

    # self.label_.setPixmap(image)
    bold_font = QFont()
    bold_font.setBold(True)
    bold_font.setPixelSize(150)

    self.label_.setStyleSheet("background-color:#222222;color:#ffffff")
    self.label_.setAutoFillBackground(True)
    self.label_.setAlignment(QtCore.Qt.AlignCenter)
    self.label_.setFont(bold_font)
    self.w_ = 400
    self.h_ = 200
    self.resize(self.w_,self.h_)
    self.label_.move(0,0)
    self.label_.show()

  def set_position(self,pos):
    self.move(pos[0]-self.w_/2, pos[1]-self.h_/2)
    pass

  def set_position(self,pos):
    self.move(pos[0]-self.w_/2, pos[1]-self.h_/2)
    pass

# ============ VIEW =======================                    
class ModulairMenuView(QWidget):

  signal_hide_ = QtCore.Signal()
  signal_show_ = QtCore.Signal()

  # constructor
  def __init__(self):
    self.qt_app_ = QApplication(sys.argv)
    QWidget.__init__(self)

    # member variables
    self.current_users_ = []     # list of users
    self.users_ = {}             # dict: (modulair_id, user)
    self.num_users_ = 0          # total num of users
    self.focused_user_id_ = -1   # focused user
    self.app_menu_items_ = {}         # dict: (app_name, qwidget)
    self.y_offset_ = 0
    self.hidden_ = False
    self.run_ = False

    # ROS
    rospy.init_node('modulair_app_menu', anonymous=True)

    # ---- ROS subscriber ---
    self.user_state_sub_ = rospy.Subscriber("/modulair/users/state",
                                            ModulairUserArray,
                                            self.user_state_cb)

    self.user_event_sub_ = rospy.Subscriber("/modulair/users/events",
                                            ModulairUserEvent,
                                            self.user_event_cb)
    
    # ---- ROS get params -----
    # height
    if rospy.has_param("/modulair/core/params/height"):
      self.height_ = rospy.get_param("/modulair/core/params/height")
    else:
      rospy.logerr("ModulairInfobar: parameter [height] not found on server")

    # width
    if rospy.has_param("/modulair/core/params/width"):
      self.width_ = rospy.get_param("/modulair/core/params/width")
    else:
      rospy.logerr("ModulairInfobar: parameter [width] not found on server")

    ### x ###
    if rospy.has_param("/modulair/core/params/x"):
      self.x_ = rospy.get_param("/modulair/core/params/x")
    else:
      rospy.logerr("ModulairInfobar: parameter [x] not found on server")

    ### y ###
    if rospy.has_param("/modulair/core/params/y"):
      self.y_ = rospy.get_param("/modulair/core/params/y")
    else:
      rospy.logerr("ModulairInfobar: parameter [y] not found on server")

    ### y ###
    if rospy.has_param("/modulair/core/params/border_scale"):
      self.border_scale_ = rospy.get_param("/modulair/core/params/border_scale")
    else:
      rospy.logerr("ModulairInfobar: parameter [border_scale] not found on server")
    self.border_ = int(self.width_ * self.border_scale_)

    ### Cursor Icon ###
    if rospy.has_param("/modulair/menu/params/cursor_path"):
      self.cursor_path_ = rospy.get_param("/modulair/menu/params/cursor_path")
    else:
      rospy.logerr("ModulairInfobar: parameter [cursor_path] not found on server")

    ### Application Locations ###
    if rospy.has_param("/modulair/core/available_apps"):
      self.app_paths_ = rospy.get_param("/modulair/core/available_apps")
    else:
      rospy.logerr("ModulairInfobar: parameter [available_apps] not found on server")

    ### Workspace Limits ###
    if rospy.has_param("/modulair/menu/params/workspace_size"):
      self.workspace_limits_ = rospy.get_param("/modulair/menu/params/workspace_size")
    else:
      rospy.logerr("ModulairInfobar: parameter [workspace_size] not found on server")

    ### Y Offset ###
    if rospy.has_param("/modulair/menu/params/y_offset"):
      self.y_offset_ = rospy.get_param("/modulair/menu/params/y_offset")
    else:
      rospy.logerr("ModulairInfobar: parameter [y_offset] not found on server")

    ### Height Scaling ###
    if rospy.has_param("/modulair/menu/params/height_percentage"):
      self.height_perc_ = rospy.get_param("/modulair/menu/params/height_percentage")
    else:
      rospy.logerr("ModulairInfobar: parameter [height_percentage] not found on server")
    rospy.logwarn("ModulairInfobar: height percentage set to " + str(self.height_perc_))
    self.height_ = int(self.height_*self.height_perc_)
    # create model
    # self.model_ = ModulairMenuModel(self.app_paths_)
    # Applications List
    # self.app_list_ = sorted(self.model_.getAppList())
    self.app_list_ = self.app_paths_.keys()

    rospy.logwarn("ModulairMenu: found " + str(len(self.app_list_)) + " applications")
    print self.app_list_

    self.grid_set_up_ = False
    self.setup_grid()

    # setup Qt GUI
#    self.setMinimumSize(QSize(self.width_,self.height_))
#    self.setMaximumSize(QSize(self.width_,self.height_))
    
    self.setWindowTitle("Modulair Main Menu")
    self.gridLayout_ = QGridLayout() 
    self.setWindowFlags(QtCore.Qt.FramelessWindowHint)
    self.resize(self.width_, self.height_)
    self.move(self.x_,self.y_)
    self.setLayout(self.gridLayout_)

    self.assignWidgets() # create widget


    self.ok_timer_ = QtCore.QTimer()
    self.connect(self.ok_timer_, QtCore.SIGNAL("timeout()"), self.check_ok)
    self.ok_timer_.start(15)

    # create cursor
    # cursor_pixmap = QPixmap(self.cursor_path_)
    # print os.getcwd()
    # self.cursor_ = QCursor(cursor_pixmap.scaledToHeight(self.height_/20.0))
    # self.cursor_.setPos(100, 100)
    # self.setCursor(self.cursor_)

    self.cursor_ = ModulairCursor(self.cursor_path_,self)
    self.cursor_.set_position([self.width_/2,self.height_/2])
    self.cursor_.show()
    self.run_ = True

    self.signal_show_.connect(self.show_menu)
    self.signal_hide_.connect(self.hide_menu)

    # self.signal_cursor_.connect(self.slot_update_cursor)

  def check_ok(self):
    if rospy.is_shutdown():
          self.qt_app_.exit()
    pass

  def setup_grid(self):
    """
    Sets up the grid size that will represent the applications in the menu.
    """
    # length = len(self.app_list_) + 1
    # n = int(math.ceil(math.sqrt(length)))
    # self.max_x_ = self.max_y_ = n
    self.max_x_ = 3
    self.max_y_ = 2
    rospy.logwarn('ModulairMenu:  Grid size is '+str(self.max_x_)+' (w) by , '+str(self.max_y_)+' (h)')
    self.cur_ind_x_ = 0 
    self.cur_ind_y_ = 0
    self.grid_set_up_ = True

  def next_pos(self):
    """ Return a tuple of the next position """

    # check if grid ind has been set
    if not self.grid_set_up_:
      self.setup_grid()

    ind_x = self.cur_ind_x_
    ind_y = self.cur_ind_y_

    # change line if necessary
    self.cur_ind_y_ += 1
    if self.cur_ind_y_ == self.max_y_:
      self.cur_ind_y_ = 0
      self.cur_ind_x_ += 1

    # return current grid index
    return ind_x, ind_y


  def convert_workspace(self,user_pos):
    screen_pos = []
    x_min = self.workspace_limits_[0]
    x_max = self.workspace_limits_[1]
    y_min = self.workspace_limits_[2]
    y_max = self.workspace_limits_[3]
    x_total = fabs(x_max)+fabs(x_min)
    y_total = fabs(y_max)+fabs(y_min)

    x_center = int(self.width_/2)
    y_center = int(self.height_/2)

    x_pos = int(x_center + (self.width_/x_total)*user_pos[0])
    y_pos = int(y_center + -(self.height_/y_total)*user_pos[1] + self.y_offset_)
    if y_pos < 0.0:
      y_pos = 100
    if y_pos > self.height_:
      y_pos = self.height_-100
    if x_pos < 0.0:
      x_pos = 100
    if x_pos > self.width_:
      x_pos = self.width_-100
    # y_pos = y_center

    screen_pos = [x_pos, y_pos]
    return screen_pos
    pass

  # create QLabel for each app + refresh button
  def assignWidgets(self):
    self.app_menu_items_.clear()
    for app, app_path in self.app_paths_.items():
      # widget = QLabel(app.name + '\n' + app.launchpath)
      widget = QLabel(app,self)
      # set label icon for testing 
      image = app_path + '/menu_icon.jpg'
      rospy.logwarn('ModulairMenu:  Adding button for '+app+" app.")
      widget.setPixmap(image)
      widget.setFixedSize((self.width_/self.max_x_)-self.border_, (self.height_/self.max_y_)-self.border_)
      # widget.setFixedSize(500,500)
      widget.show()
      nextx, nexty = self.next_pos()
      self.gridLayout_.addWidget(widget, nextx, nexty )
      self.app_menu_items_[app] = widget
    
    # widget = QPushButton('Refresh')
    # widget.clicked.connect(self.refresh)
    # nextx, nexty = self.next_pos()
    # self.gridLayout_.addWidget(widget, nextx, nexty)

  # delete all widget + recreate all widget ???
  # def refresh(self):
  #   for unused_i in range(0, len(self.app_list_)+1):
  #     g = self.gridLayout_.takeAt(0)
  #     g.widget().deleteLater()
  #   self.app_list_ = sorted(self.model_.getAppList())
  #   self.grid_set_up_ = False
  #   self.assignWidgets()

  # user_state_cb ros callback
  def user_state_cb(self, msg):
    if self.run_:
      self.current_users_ = msg.users
      self.num_users_ = len(self.current_users_)
      self.users_.clear()
      self.focused_user_id_ = -1
      for user in self.current_users_:
        if user.focused == True:
          self.focused_user_id_ = user.modulair_id
        self.users_[user.modulair_id] = user
      
      self.update_cursor()      
    pass
  

  # user_event_cb ros callback
  def user_event_cb(self, msg):
    if self.run_:
      print msg.user_id
      print self.focused_user_id_

      if msg.event_id == 'hand_event' and msg.user_id == self.focused_user_id_:
        print msg.event_id
        # assume HANDS_HEAD = PAUSE
        if msg.message == 'hands_on_head':
          rospy.logdebug("ModulairMenu: HANDS_HEAD received, should resume menu")
          rospy.wait_for_service('modulair/core/app_manager/close_all_apps')
          try:
            self.srv_close_all_apps = rospy.ServiceProxy('modulair/core/app_manager/close_all_apps',
                                                   modulair_core.srv.close_all_apps)
            ret_success = self.srv_close_all_apps('none')
            print ret_success
            print 'emitting show'
            self.signal_show_.emit()
          except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s" % e)
          
          
        # assume RIGHT_ELBOW_CLICK
        if all( [ msg.message == 'left_elbow_click', 
                  self.current_app_name_ != "NONE"] ):
          if self.hidden_ == False:
            print msg.message
            rospy.logwarn("ModulairMenu: LEFT_ELBOW_CLICK received, let's launch app")
            rospy.wait_for_service('modulair/core/app_manager/load_app')
            try:
              self.srv_load_app = rospy.ServiceProxy('modulair/core/app_manager/load_app',
                                                     modulair_core.srv.load_app)
              ret_success = self.srv_load_app(self.current_app_name_)
              print ret_success
              print 'emitting hide'
              self.signal_hide_.emit()
            except rospy.ServiceException, e:
              rospy.logerr("Service call failed: %s" % e)
    pass # user_event_cb

  def hide_menu(self):
    self.hide()
    self.hidden_ = True
    print 'done hiding'
    pass

  def show_menu(self):
    self.show()
    self.hidden_ = False
    print 'done showing'
    pass

  # Qt slot
  def update_cursor(self):

    if self.focused_user_id_ != -1:
      # update curser position to focused user
      # pos(cursor) = self.users_[focused_user_id_]
      # if self.users_[self.focused_user_id_].right_in_front:
      #   cursorx = self.users_[self.focused_user_id_].translations_mm[7].x
      #   cursory = self.users_[self.focused_user_id_].translations_mm[7].y
      # else:
      cursorx = self.users_[self.focused_user_id_].translations_mm[8].x
      cursory = self.users_[self.focused_user_id_].translations_mm[8].y

      cursor_position = self.convert_workspace([cursorx,cursory])
      self.cursor_.set_position(cursor_position)
      # print str(cursor_position[0]) + " " + str(cursor_position[1]) + ' -- '+str(cursorx) + " " + str(cursory)

      # check which app is under cursor (mouse)
      self.current_app_name_ = "NONE"
      for appname, appwidget in self.app_menu_items_.items():
        if appwidget.geometry().contains(cursor_position[0],cursor_position[1]):
          self.current_app_name_ = appname

      # print self.current_app_name_
    
    pass # slot_update_cursor
      
  # show widget and Qt.exec()
  def run(self):
    self.show()
    self.qt_app_.exec_()
    pass


### MAIN ###
if __name__ == "__main__":
  menu = ModulairMenuView()
  menu.run() 
