#!/usr/bin/env python
import roslib; roslib.load_manifest('modulair_core')
import rospy,sys
### PySide ###
import PySide
from PySide.QtGui import *
from PySide.QtCore import QTimer
from PySide import QtCore

from geometry_msgs.msg import Transform
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool
from std_msgs.msg import String

from modulair_msgs.msg import ModulairUser
from modulair_msgs.msg import ModulairUserArray
from modulair_msgs.msg import ModulairUserEvent
from modulair_msgs.msg import TrackerUser
from modulair_msgs.msg import TrackerUserArray as tracker_msg

import modulair_core
from modulair_core.srv import *

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

class ModulairInfobar(QWidget):
  def __init__(self,app):
    super(ModulairInfobar,self).__init__()
    # Member variables    
    self.app_ = app
    self.ok_timer_ = QTimer(self)
    self.current_users_ = []
    self.users_ = {}
    self.user_tags_ = {}
    self.num_users_ = 0
    self.focused_user_id_ = -1
    rospy.logwarn("ModulairInfobar: Starting")
    # ROS Subscribers
    self.user_state_sub_ = rospy.Subscriber("/modulair/users/state", ModulairUserArray, self.user_state_cb)
    self.user_event_sub_ = rospy.Subscriber("/modulair/users/events", ModulairUserEvent, self.user_event_cb)
    
    # App parameters
    if rospy.has_param("/modulair/core/params/x"):
      self.wall_x_ = rospy.get_param("/modulair/core/params/x")
    else:
      rospy.logerr("ModulairInfobar: parameter [x] not found on server")

    if rospy.has_param("/modulair/core/params/y"):
      self.wall_y_ = rospy.get_param("/modulair/core/params/y")
    else:
      rospy.logerr("ModulairInfobar: parameter [y] not found on server")

    if rospy.has_param("/modulair/core/params/width"):
      self.wall_width_ = rospy.get_param("/modulair/core/params/width")
    else:
      rospy.logerr("ModulairInfobar: parameter [width] not found on server")

    if rospy.has_param("/modulair/core/params/height"):
      self.wall_height_ = rospy.get_param("/modulair/core/params/height")
    else:
      rospy.logerr("ModulairInfobar: parameter [height] not found on server")

    if rospy.has_param("/modulair/infobar/params/height_percentage"):
      self.height_perc_ = rospy.get_param("/modulair/infobar/params/height_percentage")
    else:
      rospy.logerr("ModulairInfobar: parameter [height_percentage] not found on server")

    # Set base app widget size and hints based on parameters
    self.setWindowFlags(QtCore.Qt.FramelessWindowHint)
    self.height_ = int(self.wall_height_ * self.height_perc_)
    self.width_ = self.wall_width_
    self.resize(self.width_, self.height_)
    self.move(self.wall_x_,self.wall_y_ + self.wall_height_-self.height_)
    self.setStyleSheet("background-color:#222222;color:#aaaaaa")
    self.show()

    # Set up ros_ok watchdog timer to handle termination and ctrl-c
    self.connect(self.ok_timer_, QtCore.SIGNAL("timeout()"), self.check_ok)
    self.ok_timer_.start(10)


  def check_ok(self):
    if rospy.is_shutdown():
      self.clean_up()
      self.app_.exit()
    else:
      self.update_tags()

  def clean_up(self):
    rospy.logwarn("ModulairInfobar: Cleaning up")  
    pass

  def update_tag(self,uid):
    tag = self.user_tags_[uid]
    userx = self.users_[uid].translations_mm[2].x
    tag.set_pos(int(self.width_/2.0)+userx*1.5)
    # do stuff to tag (set position, update state etc)

  def update_tags(self):
    # Update or create new user tags
    for uid,user in self.users_.items():
      if uid in self.user_tags_.keys():
        self.update_tag(uid)
      else:
        # make new tag
        rospy.logwarn("ModulairInfobar: Found [user " + str(uid) + "], creating tag")
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
      rospy.logwarn("ModulairInfobar: Lost [user " + str(tag) + "], cleaning up tag")
      self.user_tags_[tag].label_.hide()
      self.user_tags_[tag].hide()
      del(self.user_tags_[tag])

    pass    

  def user_state_cb(self,msg):
    self.current_users_ = msg.users
    self.num_users_ = len(self.current_users_)
    self.users_.clear()
    for user in self.current_users_:
      if user.focused == True:
        self.focused_user_id_ = user.modulair_id
      self.users_[user.modulair_id] = user

    pass

  def user_event_cb(self,msg):
    pass

# MAIN
if __name__ == '__main__':
  rospy.init_node('modulair_infobar',anonymous=True)
  app = QApplication(sys.argv)
  infobar = ModulairInfobar(app)
  # Running
  app.exec_()
  # Done
  rospy.logwarn('ModulairInfobar: Finished')