#!/usr/bin/env python
import roslib; roslib.load_manifest('modulair_user')
import rospy

from geometry_msgs.msg import Transform
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool
from std_msgs.msg import String

from modulair_user.msgs import _modulair_user as userMsg
from modulair_tracker.msgs import _user_array as trackerMsg

class UserManager():

  def __init__(self):
    # Members
    self.users_ = {}

    # Publisher
    self.user_pub_ = rospy.Publisher("/modulair/users",userMsg.modulair_user)
    # Subscriber
    self.tracker_sub = rospy.Subscriber("/modulair/tracker/raw",trackerMsg.user_array)

    rospy.logwarn('Modulair UserManager: Started')

    while not rospy.is_shutdown():
      publish_user_state();
      rospy.sleep(.033) # for 30 hz operation

    # Finish
    rospy.logwarn('Modulair UserManager: Finished')

  def publish_user_state(self):
    pass

  def tracker_cb(self,tracker_msg):

    num_users = len(tracker_msg.users)
    



    pass
