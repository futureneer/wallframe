#!/usr/bin/env python
import roslib; roslib.load_manifest('modulair_user')
import rospy

from geometry_msgs.msg import Transform
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool
from std_msgs.msg import String

from modulair_msgs.msg import ModulairUser as userMsg
from modulair_msgs.msg import TrackerUser
from modulair_msgs.msg import TrackerUserArray as trackerMsg

class User():
  def __init__(self,uid):
    self.uid_ = uid
    self.tracker_names_ = []
    self.tracker_uids_ = []

class UserManager():

  def __init__(self):
    # Members
    self.users_ = {}
    self.current_uid = 1

    # ROS 
    rospy.init_node('modulair_user_manager',anonymous=True)
    # Publisher
    self.user_pub_ = rospy.Publisher("/modulair/users",userMsg)
    # Subscriber
    self.tracker_sub = rospy.Subscriber("/modulair/tracker/users",trackerMsg,self.tracker_cb)

    rospy.logwarn('Modulair UserManager: Started')

    while not rospy.is_shutdown():
      self.publish_user_state();
      rospy.sleep(.033) # for 30 hz operation

    # Finish
    rospy.logwarn('Modulair UserManager: Finished')

  def publish_user_state(self):
    pass

  def add_user(self,tracker,tracker_uid):
    # Check that user doesnt exist
    for existing_user in self.users_:
      if tracker in existing_user.tracker_names_:

  def tracker_cb(self,tracker_msg):

    num_users = len(tracker_msg.users)

    for user in tracker_msg.users:
      user_tracker = user.tracker_id
      user_tracker_id = user.uid

      if len(self.users_) == 0: # No users, add a user
        self.users_[self.current_uid] = User(self.current_uid)
        self.current_uid++
      else:






    pass

# MAIN
if __name__ == '__main__':
  m = UserManager()