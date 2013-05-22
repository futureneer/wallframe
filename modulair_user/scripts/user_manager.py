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

from math import sqrt, pow

class User():
  def __init__(self,uid):
    self.uid_ = uid
    self.tracker_uids_ = {}
    self.tracker_packets_ = {}
    self.state_ = []
    self.is_active_ = False
    self.is_primary_ = False
    self.refresh_ = 0.0

  def get_info(self):
    return [self.uid_, self.tracker_uids_]

  def update_state(self):
    if len(self.tracker_packets_) > 1:
      self.merge_packets()
    else:
      self.state_ = self.tracker_packets_.values()[0]

  def merge_packets(self):
    # TODO
    pass

class UserManager():

  def __init__(self):
    # Members
    self.users_ = {}
    self.current_uid = 1
    self.active_trackers_ = []

    # Pull in parameters
    self.com_dist_thresh_ = rospy.get_param('modulair/user/center_distance_threshold',100) # default is 100 mm

    # ROS 
    rospy.init_node('modulair_user_manager',anonymous=True)
    # Publisher
    self.user_pub_ = rospy.Publisher("/modulair/users",userMsg)
    # Subscriber
    self.tracker_sub = rospy.Subscriber("/modulair/tracker/users",trackerMsg,self.tracker_cb)

    rospy.logwarn('Modulair UserManager: Started')

    while not rospy.is_shutdown():
      self.update_user_state()
      self.check_users_exist()
      self.publish_user_state()
      rospy.sleep(.033) # for 30 hz operation

    # Finish
    rospy.logwarn('Modulair UserManager: Finished')

  def publish_user_state(self):
    # TODO
    pass

  def update_user_state(self):
    for uid,user in self.users_.items():
      user.update_state()

  def check_users_exist(self):
    gone = []
    for uid, user in self.users_.items():
      if user.state_.center_of_mass == Vector3(0.0,0.0,0.0):
        gone.append(uid)

    for g in gone:
      if rospy.has_param("modulair/user_data/"+str(g)):
        rospy.delete_param("modulair/user_data/"+str(g))
      rospy.logwarn("User [modulair UID: " + str(g) + " lost, removing from list")
      del self.users_[g]


  def check_com(self,user_packet,user):
    packet_com = user_packet.center_of_mass
    user_com = user.state_.center_of_mass

    dist_sum = sqrt(pow(packet_com.x-user_com.x , 2) +
                    pow(packet_com.y-user_com.y , 2) +
                    pow(packet_com.z-user_com.z , 2) )

    if dist_sum < self.com_dist_thresh_:
      return True
    else:
      return False

  def tracker_cb(self,tracker_msg):
    for user_packet in tracker_msg.users:
      # print user_packet.uid
      add_new_user = False

      # print "number of modulair users: " + str(len(self.users_))

      # Check and update active trackers
      if user_packet.tracker_id not in self.active_trackers_:
        rospy.logwarn("New tracker is broadcasting messages: [" + str(user_packet.tracker_id) + "]")
        self.active_trackers_.append(user_packet.tracker_id)

      if len(self.users_) == 0:
        # rospy.logwarn("First user found: [tracker ID: " + str(user_packet.uid) + "]")
        add_new_user = True

      else:
        update_target = None
        
        for uid, user in self.users_.items():
          # m = "checking packet "+str(user_packet.uid)+" against current users "+str(user.uid_)+" ... "
          if user_packet.tracker_id in user.tracker_uids_.keys():
            found_tracker = True
            # m = m + "tracker matched ... "
            if user_packet.uid in user.tracker_uids_.values():
              # m = m + "uid matched ... "
              update_target = uid
            else:
              # m = m + "uid did not match ... "
              pass
          # else:
          #   # user is not being tracked by this packet's tracker
          #   if check_com(user_packet,user) == True:
          #     # this is the same user in a different tracker
          #     rospy.logwarn("Found user [modulair UID:" + str(user.uid_) + "] in new tracker [tracker " + str(user_packet.tracker_id) + ", tracker ID: " + str(user_packet.uid) + "]")
          #     user.tracker_packets_[user_packet.tracker_id] = user_packet
          #     user.tracker_uids_[user_packet.tracker_id] = user_packet.uid
          #   else:
          #     add_new_user = True
          # print m

        if update_target is not None:
          # m = m + "updating ... "
          self.users_[update_target].tracker_packets_[user_packet.tracker_id] = user_packet
        elif update_target is None:
          # m = m + "uid did not match ... adding"
          add_new_user = True

      # Add new user
      if add_new_user == True:
        if user_packet.center_of_mass == Vector3(0.0,0.0,0.0):
          pass
        else:
          u = User(self.current_uid)
          u.tracker_uids_[user_packet.tracker_id] = user_packet.uid
          u.tracker_packets_[user_packet.tracker_id] = user_packet
          rospy.logwarn("Adding user: [tracker ID: " + str(user_packet.uid) + "], [modulair ID: " + str(self.current_uid) + "]")
          self.users_[u.uid_] = u
          rospy.set_param("modulair/user_data/"+str(u.uid_),self.users_[u.uid_].get_info())
          self.current_uid += 1
          add_new_user = False
    pass

# MAIN
if __name__ == '__main__':
  m = UserManager()