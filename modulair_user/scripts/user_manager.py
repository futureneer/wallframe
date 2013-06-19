#!/usr/bin/env python
# ROS Imports
import roslib; roslib.load_manifest('modulair_user')
import rospy
import tf
# ROS Messages
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_msgs.msg import Float32
# Modulair Imports
from modulair_msgs.msg import ModulairUser as user_msg
from modulair_msgs.msg import ModulairUserArray as user_array_msg
from modulair_msgs.msg import ModulairUserEvent as user_event_msg
from modulair_msgs.msg import TrackerUser
from modulair_msgs.msg import TrackerUserArray as tracker_msg
# Helper functions and classes (local)
from OneEuroFilter import OneEuroFilter
from Switch import switch
# Python Library Functions
from math import sqrt, pow
import itertools
from itertools import izip

### FOR REFERENCE frame IDs ###
#----- 0["head"]
#----- 1["neck"]
#----- 2["torso"]
#----- 3["right_shoulder"]
#----- 4["left_shoulder"]
#----- 5["right_elbow"]
#----- 6["left_elbow"]
#----- 7["right_hand"]
#----- 8["left_hand"]
#----- 9["right_hip"]
#----- 10["left_hip"]
#----- 11["right_knee"]
#----- 12["left_knee"]
#----- 13["right_foot"]
#----- 14["left_foot"]

################################################################################
class User():

  def __init__(self,uid,tf_listener,user_event_pub,filtering,hand_click):
    # Init from constructor
    self.mid_ = uid
    self.listener_ = tf_listener
    self.filtering_ = filtering
    self.user_event_pub_ = user_event_pub
    # Initial Structures    
    self.exists_in_tracker = {}
    self.frame_names_ = []
    self.tracker_uids_ = {}
    self.tracker_packets_ = {}
    self.transforms_ = {}
    self.transforms_merged_ = []
    self.translations_ = {}
    self.translations_merged = []
    self.confs_merged_ = []
    self.merged_transform_exists_ = []
    self.tracker_transform_exists_ = {}
    self.current_state_msg = user_msg()
    self.is_active_ = False
    self.is_primary_ = False
    self.exists_ = False
    self.init_filters_ = False
    self.hand_click_ = hand_click
    # State
    self.state__ = "IDLE"
    # Prameters from parameter server
    self.hand_limit_        = rospy.get_param('/modulair/user/hand_limit') 
    self.head_limit_        = rospy.get_param('/modulair/user/head_limit')  
    self.elbow_limit_       = rospy.get_param('/modulair/user/elbow_limit')    
    self.base_frame_        = rospy.get_param('/modulair/user/base_frame','wall_frame')
    self.filter_mincutoff_  = rospy.get_param('/modulair/user/filter_mincutoff',1.0)
    self.filter_beta_       = rospy.get_param('/modulair/user/filter_beta',0.1)    
    self.filter_dcutoff_    = rospy.get_param('/modulair/user/filter_dcutoff',1.0) 
    self.run_frequency_     = rospy.get_param('/modulair/user/run_frequency',1.0)   
    self.workspace_limits_  = rospy.get_param('/modulair/user/workspace_limits')

    self.vpub_ = rospy.Publisher('/modulair/user/vel',Float32)
    self.hand_x_ = 0.0
    self.hand_y_ = 0.0
    self.up_cnt_ = 0
    self.vels_ = []
    self.cl_ = False

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
        if frame_id not in self.frame_names_:
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

  def set_up_filtering(self):
    ### Initialize filters if not already done
    if self.init_filters_ == False:
      self.translation_filters_ = [[OneEuroFilter(self.run_frequency_,self.filter_mincutoff_,self.filter_beta_,self.filter_dcutoff_),
                        OneEuroFilter(self.run_frequency_,self.filter_mincutoff_,self.filter_beta_,self.filter_dcutoff_),
                        OneEuroFilter(self.run_frequency_,self.filter_mincutoff_,self.filter_beta_,self.filter_dcutoff_)]]*len(self.frame_names_)
      self.translation_timestamps_ = [[0.0,0.0,0.0]]*len(self.frame_names_)
      self.transforms_merged_ = [Transform()]*len(self.frame_names_)
      self.confs_merged_ = [0]*len(self.frame_names_)
      self.translations_merged_ = [Vector3(0.0,0.0,0.0)]*len(self.frame_names_)
      self.translations_merged_mm_ = [Vector3(0.0,0.0,0.0)]*len(self.frame_names_)
      self.translations_merged_body_mm_ = [Vector3(0.0,0.0,0.0)]*len(self.frame_names_)
      self.merged_transform_exists_ = [False]*len(self.frame_names_)
      self.init_filters_ = True

  def merge_multiple_trackers(self):
    ### Merge transformation frames from multiple trackers
    index = 0
    for frame in iter(self.frame_names_):
      if len(self.tracker_packets_) > 1:
        # multiple tracker frames to move
        best_conf = 0.0
        best_tracker = ''
        for tracker, packet in self.tracker_packets_.items():
          if packet.confs[index] > best_conf:
            best_conf = packet.confs[index]
            best_tracker = tracker
        if self.tracker_transform_exists_[tracker_id][index] == True:
          self.transforms_merged_[index] = self.transforms_[best_tracker][index]
          self.translations_merged_[index] = self.translations_[best_tracker][index]
          # Set merged conf to best from all trackers
          self.confs_merged_[index] = best_conf
          self.merged_transform_exists_[index] = True
        else:
          self.merged_transform_exists_[index] = False
      else:
        single_tracker_id = self.tracker_uids_.keys()[0] # first and only tracker
        if self.tracker_transform_exists_[single_tracker_id][index] == True:
          self.transforms_merged_[index] = self.transforms_[single_tracker_id][index]
          self.translations_merged_[index] = self.translations_[single_tracker_id][index]
          # set merged conf to single tracker conf
          self.confs_merged_[index] = self.tracker_packets_[single_tracker_id].confs[index]
          self.merged_transform_exists_[index] = True
        else:
          self.merged_transform_exists_[index] = False
        pass
      index += 1

  def calculate_frames(self):
    ### Filter frames
    index = 0
    for translation_merged, timestamp in izip(self.translations_merged_, self.translation_timestamps_):
      if self.merged_transform_exists_[index] == True:
        if self.filtering_ == True:
          t = Vector3()
          t.x = self.translation_filters_[index][0](translation_merged.x, timestamp[0])
          timestamp[0] += 1.0/self.run_frequency_
          t.y = self.translation_filters_[index][1](translation_merged.y, timestamp[1])
          timestamp[1] += 1.0/self.run_frequency_
          t.z = self.translation_filters_[index][2](translation_merged.z, timestamp[2])
          timestamp[2] += 1.0/self.run_frequency_
          # TODO Fix filtering
          self.translations_merged_[index] = t
          self.translations_merged_mm_[index] = Vector3(t.x*1000,t.y*1000,t.z*1000)
          self.translations_merged_body_mm_[index] = Vector3( self.translations_merged_mm_[index].x - self.translations_merged_mm_[2].x, 
                                                              self.translations_merged_mm_[index].y - self.translations_merged_mm_[2].y, 
                                                              self.translations_merged_mm_[index].z - self.translations_merged_mm_[2].z)
        else:
          # Not filtering, translations_merged_ already calculated
          self.translations_merged_mm_[index] = Vector3( self.translations_merged_[index].x*1000,
                                                      self.translations_merged_[index].y*1000,
                                                      self.translations_merged_[index].z*1000)
          self.translations_merged_body_mm_[index] = Vector3( self.translations_merged_mm_[index].x - self.translations_merged_mm_[2].x, 
                                                              self.translations_merged_mm_[index].y - self.translations_merged_mm_[2].y, 
                                                              self.translations_merged_mm_[index].z - self.translations_merged_mm_[2].z)
      index += 1

  def broadcast_frames(self):
    ### Broadcast Frames
    index = 0
    br = tf.TransformBroadcaster()
    for frame_id in self.frame_names_:
      if self.merged_transform_exists_[index] == True:
        trans = self.translations_merged_[index]
        rot = tf.transformations.quaternion_from_euler(0, 0, 1)
        br.sendTransform((trans.x, trans.y, trans.z),
                         rot,
                         rospy.Time.now(),
                         "user_"+str(self.mid_)+"_"+self.frame_names_[index]+"_merged",
                         self.base_frame_)
      index += 1
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
    self.set_up_filtering()
    self.merge_multiple_trackers()
    self.calculate_frames()
    self.broadcast_frames()
    self.merge_to_state_msg()
    self.evaluate_events()
    self.evaluate_state()
    self.update_exists()

  def joint_by_name(self,name):
    return self.frame_names_.index(name)

  def joint_dist(self,joint1,joint2):

    j1 = self.translations_merged_mm_[self.joint_by_name(joint1)]
    j2 = self.translations_merged_mm_[self.joint_by_name(joint2)]

    if self.not_zero(j1,j2) == False:
      return -1
    dist = sqrt(pow(j1.x-j2.x , 2) +
                    pow(j1.y-j2.y , 2) +
                    pow(j1.z-j2.z , 2) )
    return dist

  def check_joint_dist(self,thresh,joint1,joint2):
    dist = self.joint_dist(joint1,joint2)
    if dist == -1:
      return False
    # print joint1 +" - "+str(dist)
    if dist > thresh:
      return False
    else:
      return True

  def check_outside_workspace(self):
    torso = self.translations_merged_mm_[self.joint_by_name('torso')]
    if all( [ torso.x > self.workspace_limits_[0],
              torso.x < self.workspace_limits_[1],
              torso.y > self.workspace_limits_[2],
              torso.y < self.workspace_limits_[3],
              torso.z > self.workspace_limits_[4],
              torso.z < self.workspace_limits_[5], ] ):
      return False
    else:
      return True

  def joint_pos(self,j):
    return self.translations_merged_mm_[self.joint_by_name(j)]
  
  def joint_body_pos(self,j):
    return self.translations_merged_body_mm_[self.joint_by_name(j)]

  def not_zero(self,joint1,joint2):
    nz = True
    if joint1 == Vector3(0.0,0.0,0.0):
      nz = False
    if joint2 == Vector3(0.0,0.0,0.0):
      nz = False
    return nz

  def evaluate_events(self):
    # Hands Together
    if self.check_joint_dist(self.hand_limit_,'left_hand','right_hand'):
      self.current_state_msg.hands_together = True
    else:
      self.current_state_msg.hands_together = False
    # Hands on Head
    if all( [self.check_joint_dist(self.head_limit_,'right_hand','head'),
                self.check_joint_dist(self.head_limit_,'left_hand','head')] ):
      self.current_state_msg.hands_on_head = True
    else:
      self.current_state_msg.hands_on_head = False
    # Right Elbow Click
    if self.check_joint_dist(self.hand_limit_,'left_hand','right_elbow'):
      self.current_state_msg.right_elbow_click = True
    else:
      self.current_state_msg.right_elbow_click = False
    # Left Elbow Click
    if self.check_joint_dist(self.hand_limit_,'right_hand','left_elbow'):
      self.current_state_msg.left_elbow_click = True
    else:
      self.current_state_msg.left_elbow_click = False
    # Right Hand Front 
    if self.joint_body_pos('right_hand').y > self.joint_body_pos('left_hand').y:
      self.current_state_msg.right_in_front = True
      self.current_state_msg.left_in_front = False
    else:
      self.current_state_msg.right_in_front = False
      self.current_state_msg.left_in_front = True
    # Workspace Limit
    self.current_state_msg.outside_workspace = self.check_outside_workspace()
    # Hand Click
    if self.hand_click_ == True:
      cur_vel_x = (self.joint_pos('left_hand').x - self.hand_x_)/2.0
      cur_vel_y = (self.joint_pos('left_hand').y - self.hand_y_)/2.0
      self.hand_x_ = self.joint_pos('left_hand').x
      self.hand_y_ = self.joint_pos('left_hand').y
      self.vpub_.publish(Float32(cur_vel_x))
      if cur_vel_y > -5.0 and cur_vel_y < 5.0:
        if cur_vel_x < -5.0:
          self.cl_ = True
        if self.cl_ == True:
          self.up_cnt_ += 1
          self.vels_.append(cur_vel_x)
        if self.cl_ == True and self.up_cnt_ == 6:
          v = False
          p = ''
          for vel in self.vels_:
            p = p + ' ' + str(vel)
            if vel > 4.0:
              v = True
          if v == True:
            msg = user_event_msg()  
            msg.user_id = self.mid_
            msg.event_id = 'hand_event'
            msg.message = 'left_elbow_click'
            self.user_event_pub_.publish(msg)
          # Reset
          self.cl_ = False
          self.up_cnt_ = 0
          self.vels_ = []
    pass

  def evaluate_state(self):
    msg = user_event_msg()  
    msg.user_id = self.mid_
    msg.message = 'none'

    for case in switch(self.state__):

      if case('IDLE'):
        if self.current_state_msg.outside_workspace:
            msg.event_id = 'workspace_event'
            msg.message = 'outside_workspace'
            self.state__ = 'OUTSIDE_WORKSPACE'
            break
        # Hand Events
        msg.event_id = 'hand_event'
        if self.current_state_msg.hands_on_head:
            msg.message = 'hands_on_head'
            self.state__ = 'HANDS_HEAD'
            break
        if self.current_state_msg.hands_together:
          msg.message = 'hands_together'
          self.state__ = 'HANDS_TOGETHER'
          break
        if self.current_state_msg.right_elbow_click:
          msg.message = 'right_elbow_click'
          self.state__ = 'RIGHT_ELBOW_CLICK'
          break
        if self.current_state_msg.left_elbow_click:
          msg.message = 'left_elbow_click'
          self.state__ = 'LEFT_ELBOW_CLICK'
          break
        break

      if case('OUTSIDE_WORKSPACE'):
        if not self.current_state_msg.outside_workspace:
          self.state__ = 'IDLE'
        break

      if case('HANDS_TOGETHER'):
        if not self.current_state_msg.hands_together:
          self.state__ = 'IDLE'
        break

      if case('HANDS_HEAD'):
        if not self.check_joint_dist(self.head_limit_,'left_hand','head'):
          self.state__ = 'IDLE'
          break
        elif not self.check_joint_dist(self.head_limit_,'right_hand','head'):
          self.state__ = 'IDLE'
          break
        break

      if case('RIGHT_ELBOW_CLICK'):
        if not self.current_state_msg.right_elbow_click:
          self.state__ = 'IDLE'
        break

      if case('LEFT_ELBOW_CLICK'):
        if not self.current_state_msg.left_elbow_click:
          self.state__ = 'IDLE'
        break

    # Check for populated message and publish
    if 'none' not in msg.message:
      self.user_event_pub_.publish(msg)
    pass

  def merge_to_state_msg(self):
    # Create Packet
    msg = user_msg()
    msg.modulair_id = self.mid_
    msg.frame_names = self.frame_names_
    msg.frame_confs = self.confs_merged_

    msg.transforms = self.transforms_merged_
    msg.translations = self.translations_merged_
    msg.translations_mm = self.translations_merged_mm_
    msg.translations_body_mm = self.translations_merged_body_mm_
    self.current_state_msg = msg
    pass

################################################################################
class UserManager():

  def __init__(self):
    # Members
    self.users_ = {}
    self.current_uid_ = 1
    self.active_trackers_ = []
    self.no_users_ = True

    # ROS 
    rospy.init_node('modulair_user_manager',anonymous=True)
    self.time_ = rospy.get_time()
    # ROS Params
    self.com_dist_thresh_   = rospy.get_param('/modulair/user/center_distance_threshold',100) # default is 100 mm
    self.run_frequency_    = rospy.get_param('/modulair/user/run_frequency',30.0)

    filtering = rospy.get_param('/modulair/user/user_filtering')
    if filtering == False:
      self.filtering_ = False
      rospy.logwarn("ModulairUserManager: User Filtering set to FALSE")
    elif filtering == True:
      self.filtering_ = True
      rospy.logwarn("ModulairUserManager: User Filtering set to TRUE")
    else:
      rospy.logwarn("ModulairUserManager: user_filtering parameter invalid value ["+str(filtering)+"]or not found")

    hand_click = rospy.get_param('/modulair/user/hand_click')
    if hand_click == False:
      self.hand_click_ = False
      rospy.logwarn("ModulairUserManager: User Hand Based Click set to FALSE")
    elif hand_click == True:
      self.hand_click_ = True
      rospy.logwarn("ModulairUserManager: User Hand Based Click set to TRUE")
    else:
      rospy.logwarn("ModulairUserManager: hand_click parameter invalid value ["+str(hand_click)+"]or not found")

    # Publishers
    self.user_state_pub_ = rospy.Publisher("/modulair/users/state",user_array_msg)
    self.user_event_pub_ = rospy.Publisher("/modulair/users/events",user_event_msg)
    self.modulair_event_pub = rospy.Publisher("/modulair/events",String)
    self.toast_pub_ = rospy.Publisher("/modulair/info/toast",String)
    self.debug_pub_ = rospy.Publisher("/modulair/info/debug",String)
    # Subscriber
    self.tracker_sub = rospy.Subscriber("/modulair/tracker/users",tracker_msg,self.tracker_cb)
    # TF Listener
    self.tf_listener_ = tf.TransformListener()

    rospy.logwarn('ModulairUserManager: Started')

    while not rospy.is_shutdown():
      # self.time_ = rospy.get_time()

      self.update_user_state()
      self.check_users_exist()
      self.publish_user_state()

      # print str(rospy.get_time() - self.time_)
      
      rospy.sleep(1.0/self.run_frequency_) # for 30 hz operation

    # Finish
    rospy.logwarn('ModulairUserManager: Finished')

  def publish_user_state(self):
    state_packet = user_array_msg()
    state_packet.num_users = len(self.users_)
    for uid, user in self.users_.items():
      state_packet.users.append(user.current_state_msg)

    self.user_state_pub_.publish(state_packet)

    # print state_packet
    pass

  def check_focused_user(self):
    closest = -1
    closest_dist = 1000000
    for uid, user in self.users_.items():
      user_dist = user.current_state_msg.translations_mm[2].z
      user.current_state_msg.focused = False
      if all([user_dist < closest_dist,user_dist != 0.0]):
        closest_dist = user_dist
        closest = uid
    if closest != -1:
      self.users_[closest].current_state_msg.focused = True 
    pass

  def update_user_state(self):
    for uid,user in self.users_.items():
      user.update_state()
    self.check_focused_user()

  def check_users_exist(self):
    gone = []
    for uid, user in self.users_.items():
      if user.exists_ == False:
        gone.append(uid)

    for g in gone:
      if rospy.has_param("modulair/user_data/"+str(g)):
        rospy.delete_param("modulair/user_data/"+str(g))
      rospy.logwarn("User [modulair UID: " + str(g) + "] lost, removing from list")
      self.toast_pub_.publish(String("User "+str(g)+" Lost"))
      
      msg = user_event_msg()  
      msg.user_id = g
      msg.event_id = 'workspace_event'
      msg.message = 'user_left'
      self.user_event_pub_.publish(msg)

      del self.users_[g]

    if len(self.users_) == 0:
      if self.no_users_ == False:
        # Send all users left message
        msg = user_event_msg()  
        msg.user_id = g
        msg.event_id = 'workspace_event'
        msg.message = 'all_users_left'
        self.user_event_pub_.publish(msg)
        self.no_users_ = True

  def check_com(self,user_packet,user):
    packet_com = user_packet.center_of_mass
    user_com = user.state_.center_of_mass

    dist = sqrt(pow(packet_com.x-user_com.x , 2) +
                    pow(packet_com.y-user_com.y , 2) +
                    pow(packet_com.z-user_com.z , 2) )

    if dist < self.com_dist_thresh_:
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
          # Increment the user id to assign if that id is already taken
          while self.current_uid_ in self.users_.keys():
            self.current_uid_ += 1
            if self.current_uid_ > 6:
              self.current_uid_ = 0

          u = User( self.current_uid_,
                    self.tf_listener_,
                    self.user_event_pub_,
                    self.filtering_,
                    self.hand_click_)
          u.tracker_uids_[user_packet.tracker_id] = user_packet.uid
          u.tracker_packets_[user_packet.tracker_id] = user_packet
          u.exists_in_tracker[user_packet.tracker_id] = True
          u.exists_ = True

          msg = user_event_msg()  
          msg.user_id = self.current_uid_
          msg.event_id = 'workspace_event'
          msg.message = 'user_entered'
          self.user_event_pub_.publish(msg)

          rospy.logwarn("Adding user: [tracker ID: " + str(user_packet.uid) + "], [modulair ID: " + str(self.current_uid_) + "]")
          self.toast_pub_.publish(String("User "+str(self.current_uid_)+" Joined"))
          self.users_[u.mid_] = u
          rospy.set_param("modulair/user_data/"+str(u.mid_),self.users_[u.mid_].get_info())
          self.current_uid_ += 1
          if self.current_uid_ > 6:
            self.current_uid_ = 0
          self.no_users_ = False
          add_new_user = False
    pass

################################################################################
# MAIN
if __name__ == '__main__':
  m = UserManager()