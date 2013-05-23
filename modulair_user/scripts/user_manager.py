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

from OneEuroFilter import OneEuroFilter

from math import sqrt, pow
import itertools
from itertools import izip

class User():
  def __init__(self,uid,base_frame,tf_listener,filter_freq,filter_mincutoff,filter_beta,filter_dcutoff):
    self.mid_ = uid
    self.exists_in_tracker = {}
    self.frame_names_ = []
    self.tracker_uids_ = {}
    self.tracker_packets_ = {}
    self.transforms_ = {}
    self.transforms_merged_ = []
    self.translations_ = {}
    self.translations_merged = []
    self.translations_merged_filtered_ = []
    self.merged_transform_exists_ = []
    self.tracker_transform_exists_ = {}
    self.current_state_msg = user_msg()
    self.is_active_ = False
    self.is_primary_ = False
    self.refresh_ = 0.0
    self.exists_ = False
    self.base_frame_ = base_frame
    self.listener_ = tf_listener
    self.init_filters_ = False
    self.filter_freq_ = filter_freq
    self.filter_mincutoff_ = filter_mincutoff
    self.filter_beta_ = filter_beta
    self.filter_dcutoff_ = filter_dcutoff

  def get_transforms(self):
    self.transforms_.clear()
    self.translations_.clear()
    self.tracker_transform_exists_.clear()
    self.translations_merged = []
    self.frame_names_ = []
    for tracker,uid in self.tracker_uids_.items():
      # print "for tracker " + str(tracker)
      self.transforms_[tracker] = []
      self.translations_[tracker] = []
      self.tracker_transform_exists_[tracker] = []
      for frame_id in self.tracker_packets_[tracker].frames:
        self.frame_names_.append(frame_id)
        # print "for frame " + str(frame_id)
        parent_frame = self.base_frame_
        child_frame = '/' + str(tracker) + '/' + frame_id + '_' + str(uid)
        try:
          (trans,rot) = self.listener_.lookupTransform(parent_frame, child_frame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
          # print "NO TRANSFORM EXISTS"
          self.tracker_transform_exists_[tracker].append(False)
          continue
        # print "TRANSFORM FOUND"
        self.tracker_transform_exists_[tracker].append(True)

        t = Transform()
        t.translation.x = trans[0]
        t.translation.y = trans[1]
        t.translation.z = trans[2]
        t.rotation.x = rot[0]
        t.rotation.y = rot[1]
        t.rotation.z = rot[2]
        t.rotation.w = rot[3]

        tl = Vector3()
        tl.x = trans[0]
        tl.y = trans[1]
        tl.z = trans[2]
        # print tracker
        self.transforms_[tracker].append(t)
        self.translations_[tracker].append(tl)

    ### Initialize filters if not already done
    if self.init_filters_ == False:
      self.translation_filters_ = [[OneEuroFilter(self.filter_freq_,self.filter_mincutoff_,self.filter_beta_,self.filter_dcutoff_),
                        OneEuroFilter(self.filter_freq_,self.filter_mincutoff_,self.filter_beta_,self.filter_dcutoff_),
                        OneEuroFilter(self.filter_freq_,self.filter_mincutoff_,self.filter_beta_,self.filter_dcutoff_)]]*len(self.frame_names_)
      self.translation_timestamps_ = [[0.0,0.0,0.0]]*len(self.frame_names_)
      self.translations_merged_filtered_ = [Vector3(0.0,0.0,0.0)]*len(self.frame_names_)
      self.translations_merged_filtered_mm_ = [Vector3(0.0,0.0,0.0)]*len(self.frame_names_)
      self.transforms_merged = [Transform()]*len(self.frame_names_)
      self.translations_merged_ = [Vector3(0.0,0.0,0.0)]*len(self.frame_names_)
      self.merged_transform_exists_ = [False]*len(self.frame_names_)
      self.init_filters_ = True

    ### Merge transformation frames from multiple trackers
    index = 0
    for frame in iter(self.frame_names_):
      if len(self.tracker_packets_) > 1:
        # multiple tracker frames to move
        best_conf = 0.0
        best_tracker = ''
        for tracker,packet in self.tracker_packets_.items():
          if packet.confs[index] > best_conf:
            best_conf = packet.confs[index]
            best_tracker = tracker
        if self.tracker_transform_exists_[tracker_id][index] == True:
          self.transforms_merged[index] = self.transforms_[best_tracker][index]
          self.translations_merged_[index] = self.translations_[best_tracker][index]
          self.merged_transform_exists_[index] = True
        else:
          self.merged_transform_exists_[index] = False
      else:
        tracker_id = self.tracker_uids_.keys()[0] # first and only tracker
        if self.tracker_transform_exists_[tracker_id][index] == True:
          self.transforms_merged[index] = self.transforms_[tracker_id][index]
          self.translations_merged_[index] = self.translations_[tracker_id][index]
          self.merged_transform_exists_[index] = True
        else:
          self.merged_transform_exists_[index] = False
        pass
      index += 1

    ### Filter frames
    index = 0
    for translation, timestamp in izip(self.translations_merged_, self.translation_timestamps_):
      if self.merged_transform_exists_[index] == True:
        t = Vector3()
        # print translation
        t.x = self.translation_filters_[index][0](translation.x, timestamp[0])
        timestamp[0] += 1.0/self.filter_freq_
        t.y = self.translation_filters_[index][1](translation.y, timestamp[1])
        timestamp[1] += 1.0/self.filter_freq_
        t.z = self.translation_filters_[index][2](translation.z, timestamp[2])
        timestamp[2] += 1.0/self.filter_freq_

        self.translations_merged_filtered_[index] = t
        self.translations_merged_filtered_mm_[index] = Vector3(t.x*1000,t.y*1000,t.z*1000)

      index += 1


    pass

  def filter_transforms(self):

    pass

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

    self.get_transforms()
    self.merge_to_state_msg()
    self.evaluate_state()
    self.evaluate_events()
    self.update_exists()

  def evaluate_state(self):
    pass

  def evaluate_events(self):
    pass

  def merge_to_state_msg(self):
    # Create Packet
    msg = user_msg()
    msg.modulair_id = self.mid_
    msg.frame_names = self.frame_names_

    msg.transforms = self.transforms_merged_
    msg.translations = self.translations_merged_
    msg.translations_filtered = self.translations_merged_filtered_
    msg.translations_mm = self.translations_merged_filtered_mm_
    self.current_state_msg = msg

    print(str(msg.translations[0].x) + " , " + 
          str(msg.translations[0].y) + " , " +
          str(msg.translations[0].z) + " , " + " vs " + 
          str(msg.translations_filtered[0].x) + " , " +
          str(msg.translations_filtered[0].y) + " , " +
          str(msg.translations_filtered[0].z))
    pass

class UserManager():

  def __init__(self):
    # Members
    self.users_ = {}
    self.current_uid = 1
    self.active_trackers_ = []

    # ROS 
    rospy.init_node('modulair_user_manager',anonymous=True)
    # ROS Params
    self.com_dist_thresh_   = rospy.get_param('modulair/user/center_distance_threshold',100) # default is 100 mm
    self.base_frame_        = rospy.get_param('modulair/user/base_frame','wall_frame')
    self.filter_mincutoff_  = rospy.get_param('modulair/user/filter_mincutoff',1.0)
    self.filter_beta_       = rospy.get_param('modulair/user/filter_beta',0.1)    
    self.filter_dcutoff_    = rospy.get_param('modulair/user/filter_dcutoff',1.0) 
    self.run_frequency_    = rospy.get_param('modulair/user/run_frequency',1.0)    
    # Publishers
    self.user_state_pub_ = rospy.Publisher("/modulair/users/state",user_array_msg)
    self.user_event_pub_ = rospy.Publisher("/modulair/users/events",user_event_msg)
    # Subscriber
    self.tracker_sub = rospy.Subscriber("/modulair/tracker/users",tracker_msg,self.tracker_cb)
    # TF Listener
    self.tf_listener_ = tf.TransformListener()

    rospy.logwarn('Modulair UserManager: Started')

    while not rospy.is_shutdown():
      self.update_user_state()
      self.check_users_exist()
      self.publish_user_state()
      rospy.sleep(1.0/self.run_frequency_) # for 30 hz operation

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
          u = User( self.current_uid,
                    self.base_frame_,
                    self.tf_listener_,
                    self.run_frequency_,
                    self.filter_mincutoff_,
                    self.filter_beta_,
                    self.filter_dcutoff_)
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