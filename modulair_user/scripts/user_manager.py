#!/usr/bin/env python
import roslib; roslib.load_manifest('modulair_user')
import rospy
import tf

from geometry_msgs.msg import Transform
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool
from std_msgs.msg import String

from modulair_msgs.msg import ModulairUser as user_msg
from modulair_msgs.msg import ModulairUserArray as user_array_msg
from modulair_msgs.msg import ModulairUserEvent as user_event_msg
from modulair_msgs.msg import TrackerUser
from modulair_msgs.msg import TrackerUserArray as tracker_msg

from math import sqrt, pow

class User():
  def __init__(self,uid,base_frame,tf_listener):
    self.mid_ = uid
    self.exists_in_tracker = {}
    self.tracker_uids_ = {}
    self.tracker_packets_ = {}
    self.transforms_ = {}
    self.translations_ = {}
    self.translations_mm_ = {}
    self.transform_exists_ = {}
    self.current_state_msg = user_msg()
    self.is_active_ = False
    self.is_primary_ = False
    self.refresh_ = 0.0
    self.exists_ = False
    self.base_frame_ = base_frame
    self.listener_ = tf_listener

  def get_frames(self):
    self.transforms_.clear()
    self.translations_.clear()
    self.translations_mm_.clear()
    self.transform_exists_.clear()
    for tracker,uid in self.tracker_uids_.items():
      # print "for tracker " + str(tracker)
      self.transforms_[tracker] = []
      self.translations_[tracker] = []
      self.translations_mm_[tracker] = []
      self.transform_exists_[tracker] = []
      for frame_id in self.tracker_packets_[tracker].frames:
        # print "for frame " + str(frame_id)
        parent_frame = self.base_frame_
        child_frame = '/' + str(tracker) + '/' + frame_id + '_' + str(uid)
        try:
          (trans,rot) = self.listener_.lookupTransform(parent_frame, child_frame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
          # print "NO TRANSFORM EXISTS"
          self.transform_exists_[tracker].append(False)
          continue
        # print "TRANSFORM FOUND"
        self.transform_exists_[tracker].append(True)

        t = Transform()
        t.translation.x = trans[0]
        t.translation.y = trans[1]
        t.translation.z = trans[2]
        t.rotation.x = rot[0]
        t.rotation.y = rot[1]
        t.rotation.z = rot[2]
        t.rotation.w = rot[3]

        v = Vector3()
        v.x = trans[0]
        v.y = trans[1]
        v.z = trans[2]
        
        v_mm = Vector3()
        v_mm.x = trans[0]*1000.0
        v_mm.y = trans[1]*1000.0
        v_mm.z = trans[2]*1000.0

        self.transforms_[tracker].append(t) 
        self.translations_[tracker].append(v)
        self.translations_mm_[tracker].append(v_mm) 

  def get_info(self):
    return [self.mid_, self.tracker_uids_]

  def update_exists(self):
    for tracker,packet in self.tracker_packets_.items():
      if packet.center_of_mass == Vector3(0.0,0.0,0.0):
        self.exists_in_tracker[tracker] = False
      else:
        self.exists_in_tracker[tracker] = True

    ex = False
    for tracker,packet in self.tracker_packets_.items():
      if self.exists_in_tracker[tracker] == True:
        ex = True
    self.exists_ = ex  

  def update_state(self):

    self.get_frames()
    self.merge_frames_to_state_msg()
    self.evaluate_state()
    self.evaluate_events()
    self.update_exists()

  def evaluate_state(self):
    pass

  def evaluate_events(self):
    pass

  def merge_frames_to_state_msg(self):
    # Create Packet
    msg = user_msg()
    msg.modulair_id = self.mid_
    msg.frame_names = self.tracker_packets_.values()[0].frames

    msg.transforms = []
    msg.translations = []
    msg.translations_mm = []
    index = 0
    for frame in msg.frame_names:
      if len(self.tracker_packets_) > 1:
        # multiple tracker frames to move
        best_conf = 0.0
        best_tracker = ''
        for tracker,packet in self.tracker_packets_.items():
          if packet.confs[index] > best_conf:
            best_conf = packet.confs[index]
            best_tracker = tracker

        msg.transforms.append(self.transforms_[best_tracker][index])
        msg.translations.append(self.translations_[best_tracker][index]) 
        msg.translations_mm.append(self.translations_mm_[best_tracker][index]) 
      else:
        tracker_id = self.tracker_uids_.keys()[0] # first and only tracker
        if self.transform_exists_[tracker_id][index] == True:
          msg.transforms.append(self.transforms_[tracker_id][index])
          msg.translations.append(self.translations_[tracker_id][index])
          msg.translations_mm.append(self.translations_mm_[tracker_id][index])
        pass
      index = index + 1
    self.current_state_msg = msg
    pass

class UserManager():

  def __init__(self):
    # Members
    self.users_ = {}
    self.current_uid = 1
    self.active_trackers_ = []

    # Pull in parameters
    self.com_dist_thresh_ = rospy.get_param('modulair/user/center_distance_threshold',100) # default is 100 mm
    self.base_frame_ = rospy.get_param('modulair/user/base_frame','wall_frame')

    # ROS 
    rospy.init_node('modulair_user_manager',anonymous=True)
    # Publisher
    self.user_state_pub_ = rospy.Publisher("/modulair/users/state",user_array_msg)
    self.user_event_pub_ = rospy.Publisher("/modulair/users/events",user_event_msg)
    # Subscriber
    self.tracker_sub = rospy.Subscriber("/modulair/tracker/users",tracker_msg,self.tracker_cb)

    self.tf_listener_ = tf.TransformListener()

    rospy.logwarn('Modulair UserManager: Started')

    while not rospy.is_shutdown():
      self.update_user_state()
      self.check_users_exist()
      self.publish_user_state()
      rospy.sleep(.033) # for 30 hz operation

    # Finish
    rospy.logwarn('Modulair UserManager: Finished')

  def publish_user_state(self):
    state_packet = user_array_msg()
    state_packet.num_users = len(self.users_)
    for uid, user in self.users_.items():
      state_packet.users.append(user.current_state_msg)

    self.user_state_pub_.publish(state_packet)
    # print state_packet
    pass

  def update_user_state(self):
    for uid,user in self.users_.items():
      user.update_state()

  def check_users_exist(self):
    gone = []
    for uid, user in self.users_.items():
      if user.exists_ == False:
        gone.append(uid)

    for g in gone:
      if rospy.has_param("modulair/user_data/"+str(g)):
        rospy.delete_param("modulair/user_data/"+str(g))
      rospy.logwarn("User [modulair UID: " + str(g) + "] lost, removing from list")
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
          u = User(self.current_uid,self.base_frame_,self.tf_listener_)
          u.tracker_uids_[user_packet.tracker_id] = user_packet.uid
          u.tracker_packets_[user_packet.tracker_id] = user_packet
          u.exists_in_tracker[user_packet.tracker_id] = True
          u.exists_ = True
          rospy.logwarn("Adding user: [tracker ID: " + str(user_packet.uid) + "], [modulair ID: " + str(self.current_uid) + "]")
          self.users_[u.mid_] = u
          rospy.set_param("modulair/user_data/"+str(u.mid_),self.users_[u.mid_].get_info())
          self.current_uid += 1
          add_new_user = False
    pass

# MAIN
if __name__ == '__main__':
  m = UserManager()