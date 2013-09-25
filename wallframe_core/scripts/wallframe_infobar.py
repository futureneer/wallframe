#!/usr/bin/env python
import roslib; roslib.load_manifest('wallframe_core')
import rospy,sys
### PySide ###
import PySide
from PySide.QtGui import *
from PySide.QtCore import *
from PySide import QtCore

from geometry_msgs.msg import Transform
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool
from std_msgs.msg import String

from wallframe_msgs.msg import WallframeUser
from wallframe_msgs.msg import WallframeUserArray
from wallframe_msgs.msg import WallframeUserEvent
from wallframe_msgs.msg import TrackerUser
from wallframe_msgs.msg import TrackerUserArray as tracker_msg

import wallframe_core
from wallframe_core.srv import *

'''
class UserTag(QWidget):
  def __init__(self,uid,parent,parent_width,parent_height):
    super(UserTag,self).__init__(parent)
    self.uid_ = uid
    self.parent_height_ = parent_height
    self.parent_width_ = parent_width
    self.state_ = 'IDLE'
    self.mode_ = 'MINIMIZED'

    self.height_ = int(self.parent_height_ * 0.7)
    self.width_ = int(self.parent_width_ * 0.05)
    self.y_ = int(self.parent_height_ * 0.15)

    self.resize(self.width_, self.height_)
    self.move(int(self.parent_width_/2.0),self.y_)
    self.setStyleSheet("background-color:#ffffff;color:#222222")
    # self.setRenderHints(QtGui.QPainter.Antialiasing | QtGui.QPainter.SmoothPixmapTransform)
    self.setAutoFillBackground(True)
    self.show()

    bold_font = QFont()
    bold_font.setBold(True)
    bold_font.setPixelSize(self.height_ * 0.6)

    self.label_ = QLabel("USER " + str(uid), self)
    self.label_.resize(self.width_, self.height_)
    self.label_.setStyleSheet("background-color:#ffffff;color:#222222")
    self.label_.setAutoFillBackground(True)
    self.label_.setAlignment(QtCore.Qt.AlignCenter)
    self.label_.setFont(bold_font)
    self.label_.show()

  def set_pos(self,xpos):
    self.move(xpos,self.y_)
    pass

  def update_state(self):
    pass
'''

################################################################################
class UserTag(QWidget):
  def __init__(self,uid,parent,parent_width,parent_height):
    super(UserTag,self).__init__()
    self.uid_ = uid
    self.parent_height_ = parent_height
    self.parent_width_ = parent_width
    self.state_ = 'IDLE'
    self.mode_ = 'MINIMIZED'
    self.parent_ = parent

    self.joints_ = {
    'head' : 0, 'neck' : 1, 'torso' : 2, 'right_shoulder' : 3,
    'left_shoulder' : 4, 'right_elbow' : 5, 'left_elbow' : 6, 
    'right_hand' : 7, 'left_hand' : 8, 'right_hip' : 9, 
    'left_hip' : 10, 'right_knee' : 11, 'left_knee' : 12,
    'right_foot' : 13, 'left_foot' : 14
    }
    self.joint_labels_ = {}
    self.create_joint_labels()

    if rospy.has_param("/wallframe/menu/params/workspace_size"):
      self.workspace_limits = rospy.get_param("/wallframe/menu/params/workspace_size")
    else:
      self.rospy.logerr("WallframeInfobar: parameter [workspace_size] not found on server")

    self.height_ = int(self.parent_height_ * 3)
    self.width_ = int(self.parent_width_ * 0.05)
    self.y_ = int(self.parent_.wall_height_ - self.parent_height_ * 3)
    self.x_ratio = abs(self.workspace_limits[0] - self.workspace_limits[1]) // self.width_ - 3#MURICA
    self.y_ratio = abs(self.workspace_limits[2] - self.workspace_limits[3]) // self.height_ + 2

    self.resize(self.width_, self.height_)
    self.move(int(self.parent_width_/2.0),self.y_)
    self.setStyleSheet("background-color:#ffffff;color:#222222")
    self.setWindowFlags(QtCore.Qt.FramelessWindowHint)
    #self.setRenderHints(QtGui.QPainter.Antialiasing | QtGui.QPainter.SmoothPixmapTransform)
    self.setAutoFillBackground(True)
    self.show()

  def create_joint_labels(self):
    for name in self.joints_.keys():
      joint_label = QLabel()
      joint_label.resize(16, 16)
      circle_region = QRegion(QRect(0, 0, 14, 14), QRegion.Ellipse)
      joint_label.setMask(circle_region)
      joint_label.setAutoFillBackground(True)
      self.joint_labels_[name] = joint_label
    pass


  def get_joint(self, name):
    return self.parent_.users_[self.uid_].translations_mm[self.joints_[name]]

  def get_confidence(self,jid):
    return self.parent_.users_[self.uid_].frame_confs[jid]

  def update_mini_skel(self, xpos):
    self.activateWindow()
    for j_name, j_id in self.joints_.items():
      joint_x = self.get_joint(j_name).x
      joint_y = self.get_joint(j_name).y
      torso_x = self.get_joint('torso').x
      joint_lbl = self.joint_labels_[j_name]
      self.draw_joint_lbl(j_id, torso_x, joint_x, -joint_y, joint_lbl)

    self.move(xpos+self.parent_.wall_x_,self.y_)
    pass

  def hide_mini_skel(self):
    for label in self.joint_labels_.values():
      label.hide()
    pass

  def draw_joint_lbl(self, jid, xtorso, xpos, ypos, label):
    confidence = self.get_confidence(jid)
    if jid > self.joints_['left_hip'] or confidence == 1.0:
      label.setStyleSheet("background-color:#00ff00")
    elif confidence == 0.0:
      label.setStyleSheet("background-color:#ff0000")
    else:
      label.setStyleSheet("background-color:#ffa500")
    label.setParent(self)
    x_torso = (xtorso // self.x_ratio)
    x_cord = (xpos // self.x_ratio) - 8
    y_cord = (ypos // self.y_ratio) + 100
    x_anchor = x_cord - x_torso + (self.width_ // 2)
    label.move(x_anchor, y_cord)
    label.show()
    pass

################################################################################
class WallframeInfobar(QWidget):

  toast_message_ = QtCore.Signal()
  
  def __init__(self,app):
    super(WallframeInfobar,self).__init__()
    # Member variables    
    self.app_ = app
    self.ok_timer_ = QTimer(self)
    self.current_users_ = []
    self.users_ = {}
    self.user_tags_ = {}
    self.num_users_ = 0
    self.focused_user_id_ = -1
    rospy.logwarn("WallframeInfobar: Starting")
    # ROS Subscribers
    self.user_state_sub_ = rospy.Subscriber("/wallframe/users/state", WallframeUserArray, self.user_state_cb)
    self.user_event_sub_ = rospy.Subscriber("/wallframe/users/events", WallframeUserEvent, self.user_event_cb)
    self.toast_sub_ = rospy.Subscriber("/wallframe/info/toast", String, self.toast_cb)
    # App parameters
    if rospy.has_param("/wallframe/core/params/x"):
      self.wall_x_ = rospy.get_param("/wallframe/core/params/x")
    else:
      rospy.logerr("WallframeInfobar: parameter [x] not found on server")

    if rospy.has_param("/wallframe/core/params/y"):
      self.wall_y_ = rospy.get_param("/wallframe/core/params/y")
    else:
      rospy.logerr("WallframeInfobar: parameter [y] not found on server")

    if rospy.has_param("/wallframe/core/params/width"):
      self.wall_width_ = rospy.get_param("/wallframe/core/params/width")
    else:
      rospy.logerr("WallframeInfobar: parameter [width] not found on server")

    if rospy.has_param("/wallframe/core/params/height"):
      self.wall_height_ = rospy.get_param("/wallframe/core/params/height")
    else:
      rospy.logerr("WallframeInfobar: parameter [height] not found on server")

    if rospy.has_param("/wallframe/infobar/params/height_percentage"):
      self.height_perc_ = rospy.get_param("/wallframe/infobar/params/height_percentage")
    else:
      rospy.logerr("WallframeInfobar: parameter [height_percentage] not found on server")

    # Set base app widget size and hints based on parameters
    self.setWindowFlags(QtCore.Qt.FramelessWindowHint)
    self.height_ = int(self.wall_height_ * self.height_perc_)
    self.width_ = self.wall_width_
    self.resize(self.width_, self.height_)
    self.move(self.wall_x_,self.wall_y_ + self.wall_height_-self.height_)
    self.setStyleSheet("background-color:#222222;color:#aaaaaa")
    self.show()

    self.toast_notifier_ = QLabel("Initial Text",self);
    self.toast_w_ = int(self.width_ * 0.25)
    self.toast_h_ = int(self.height_ * 0.7)
    self.toast_x_ = int(self.width_ * 0.75)
    self.toast_y_start_ = int(self.height_)
    self.toast_y_end_ = int(self.height_ * 0.15)

    bold_font = QFont()
    bold_font.setBold(True)
    bold_font.setPixelSize(self.toast_h_ * 0.9)

    self.toast_notifier_.resize(self.toast_w_,self.toast_h_)
    self.toast_notifier_.move(self.toast_x_,self.toast_y_start_)
    self.toast_notifier_.setStyleSheet("background-color:#ffffff;color:#222222")
    self.toast_notifier_.setAutoFillBackground(True)
    self.toast_notifier_.setAlignment(QtCore.Qt.AlignCenter)
    self.toast_notifier_.setFont(bold_font)
    self.toast_notifier_.show()

    self.toast_down_timer_ = QTimer()
    self.toast_down_timer_.setSingleShot(True)
    self.connect(self.toast_down_timer_, QtCore.SIGNAL("timeout()"), self.toast_down)

    self.toast_message_.connect(self.toast)

    # Set up ros_ok watchdog timer to handle termination and ctrl-c
    self.connect(self.ok_timer_, QtCore.SIGNAL("timeout()"), self.check_ok)
    self.ok_timer_.start(10)

  def toast_cb(self,message):
    # rospy.logwarn("toast cb" + str(message.data))
    self.toast_notifier_.setText(str(message.data))
    self.toast_message_.emit()
    pass
  
  @QtCore.Slot()
  def toast(self):
    # rospy.logwarn("toast call")
    self.toast_up()
    pass

  def toast_up(self):
    self.toast_down_timer_.stop()
    animation = QPropertyAnimation(self.toast_notifier_, "geometry",self)
    animation.setDuration(500)
    animation.setStartValue(QRect(self.toast_x_, self.toast_y_start_, self.toast_w_,self.toast_h_))
    animation.setEndValue(QRect(self.toast_x_, self.toast_y_end_, self.toast_w_,self.toast_h_))
    animation.start()
    self.toast_down_timer_.start(4000)
    pass

  def toast_down(self):
    # rospy.logwarn("toasting down")
    animation = QPropertyAnimation(self.toast_notifier_, "geometry",self)
    animation.setDuration(500)
    animation.setStartValue(QRect(self.toast_x_, self.toast_y_end_, self.toast_w_,self.toast_h_))
    animation.setEndValue(QRect(self.toast_x_, self.toast_y_start_, self.toast_w_,self.toast_h_))
    animation.start()
    pass

  def check_ok(self):
    if rospy.is_shutdown():
      self.clean_up()
      self.app_.exit()
    else:
      self.update_tags()

  def clean_up(self):
    rospy.logwarn("WallframeInfobar: Cleaning up")  
    pass

  def update_tag(self,uid):
    tag = self.user_tags_[uid]
    # userx = self.users_[user_id].translations_mm[joint_id].x
    userx = self.users_[uid].translations_mm[2].x
    #tag.set_pos(int(self.width_/2.0)+userx*1.5) # todo make rosparam
    tag.update_mini_skel(int(self.width_/2.0)+userx*1.5) 

    # tag.update_user(self.users_[uid])

    # do stuff to tag (set position, update state etc)

  def update_tags(self):
    # Update or create new user tags
    for uid,user in self.users_.items():
      if uid in self.user_tags_.keys():
        print uid
        self.update_tag(uid)
      else:
        # make new tag
        rospy.logwarn("WallframeInfobar: Found [user " + str(uid) + "], creating tag")
        t = UserTag(uid,self,self.width_,self.height_)
        self.user_tags_[uid] = t
        self.update_tag(uid)

    # Check through tags to make sure each one has a user
    tags_to_remove = []
    for uid in self.user_tags_.keys():
      if uid not in self.users_.keys():
        tags_to_remove.append(uid)
    # Remove tags without users
    for tag in tags_to_remove:
      rospy.logwarn("WallframeInfobar: Lost [user " + str(tag) + "], cleaning up tag")
      self.user_tags_[tag].hide_mini_skel()
      self.user_tags_[tag].hide()
      del(self.user_tags_[tag])

    pass    

  def user_state_cb(self,msg):
    self.current_users_ = msg.users
    self.num_users_ = len(self.current_users_)
    self.users_.clear()
    for user in self.current_users_:
      if user.focused == True:
        self.focused_user_id_ = user.wallframe_id
      self.users_[user.wallframe_id] = user

    pass

  def user_event_cb(self,msg):
    pass

# MAIN
if __name__ == '__main__':
  rospy.init_node('wallframe_infobar',anonymous=True)
  app = QApplication(sys.argv)
  infobar = WallframeInfobar(app)
  # Running
  app.exec_()
  # Done
  rospy.logwarn('WallframeInfobar: Finished')