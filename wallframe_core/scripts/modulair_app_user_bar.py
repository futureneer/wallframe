'''
Create User Bar at bottom of screen, that is able to update position dynamically.
'''

import sys, rospy, random

from std_msgs.msg import String
from PySide.QtGui import QWidget, QApplication
from PySide.QtCore import QSize, Qt

from modulair_app_user_bar_tab import ModulairAppUserBarTab 

class ModulairAppUserBar(QWidget):
  def __init__(self):
    self.qapp_ = QApplication(sys.argv)
    QWidget.__init__(self)
    
    # Keep a dictionary of users_.
    self.users_ = {}
    # Keep a dictionary of user popups_.
#    self.popups_ = {}
    # List of users_ (probably via RosParam)
    self.userlist_ = ["user1", "user2", "user3"] 
    
    # Generate boxes
    for user in self.userlist_:
      self.users_[user] = ModulairAppUserBarTab(user, self)
      self.users_[user].update_position(random.randint(0, 1366), 0)
    # Generate Popups
#    self.popup_root = QWidget()
#    for user in self.userlist_:
#      self.popups_[user] = ModulairAppUserBarPopUp(self.popup_root)
    
    
    # Size restrictions
    self.width_ = 1366 # These values probably should be passed in via rosparams
    self.height_ = 25
    self.setFixedSize(QSize(self.width_, self.height_))
    # Set Initial Position
    self.setGeometry(0, 300, self.width_, self.height_)
    # Frameless Window
    self.setWindowFlags(Qt.FramelessWindowHint)
        
    # Set Background color to Red
    p = self.palette()
    p.setColor(self.backgroundRole(), Qt.red)
    self.setPalette(p)

    # Setup ROS and subscribers
    # TODO: Do we need to keep track of these subs?
    rospy.init_node('userbar')
    for user in self.userlist_:
      rospy.Subscriber(user, String, self.update, user)
    
  def update(self, data, user):
    # Call back function for subscribers to update position
    newx = int(data.data)
    self.users_[user].update_position(newx, 0)
    
  def run(self):
    self.show()
    self.qapp_.exec_()
    
def test():
  bar = ModulairAppUserBar()
  bar.run()
  
if __name__ == "__main__":
  test()  