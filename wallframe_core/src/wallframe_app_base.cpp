/*********************************************************************
* Software License Agreement (BSD License)
*
* Copyright (c) 2013, Johns Hopkins University
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above
* copyright notice, this list of conditions and the following
* disclaimer in the documentation and/or other materials provided
* with the distribution.
* * Neither the name of the Johns Hopkins University nor the names of its
* contributors may be used to endorse or promote products derived
* from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/*
 * Author: Kelleher Guerin, futureneer@gmail.com, Johns Hopkins University
 */

#include "wallframe_core/wallframe_app_base.h"

namespace wallframe{

	WallframeAppBase::WallframeAppBase(std::string app_name, ros::NodeHandle nh, int event_deque_size = 10){  
    node_ = nh;
    name_ = app_name;
    deque_size_ = event_deque_size;
    initBaseApp();
	}

  bool WallframeAppBase::initBaseApp(){
    // Initialize User Listeners
    user_state_subscriber_ = node_.subscribe("/wallframe/users/state", 1000, &WallframeAppBase::userStateCallback, this);
    user_event_subscriber_ = node_.subscribe("/wallframe/users/events", 1000, &WallframeAppBase::userEventCallback, this);
    // Wallframe Event Listener
    wallframe_event_subscriber_ = node_.subscribe("/wallframe/events", 1000, &WallframeAppBase::wallframeEventCallback, this);
    // Publishers for outgoing messages
    toast_publisher_ = node_.advertise<std_msgs::String>("/wallframe/info/toast", 1000);
    debug_publisher_ = node_.advertise<std_msgs::String>("/wallframe/info/debug", 1000);

    // Get Container Widget Params
    if (!node_.getParam("/wallframe/core/params/x", x_)){
      ROS_ERROR("Wallframe%s: No x position on parameter server (namespace: %s)",
        name_.c_str(), node_.getNamespace().c_str());
      return false;
    }
    if (!node_.getParam("/wallframe/core/params/y", y_)){
      ROS_ERROR("Wallframe%s: No y position on parameter server (namespace: %s)",
        name_.c_str(), node_.getNamespace().c_str());
      return false;
    }
    if (!node_.getParam("/wallframe/core/params/width", width_)){
      ROS_ERROR("Wallframe%s: No width on parameter server (namespace: %s)",
        name_.c_str(), node_.getNamespace().c_str());
      return false;
    }
    if (!node_.getParam("/wallframe/core/params/height", height_)){
      ROS_ERROR("Wallframe%s: No height on parameter server (namespace: %s)",
        name_.c_str(), node_.getNamespace().c_str());
      return false;
    }
    if (!node_.getParam("/wallframe/app/params/height_percentage", height_perc_)){
      ROS_ERROR("Wallframe%s: No height percentage on parameter server (namespace: %s)",
        name_.c_str(), node_.getNamespace().c_str());
      return false;
    }

    ROS_WARN_STREAM("WallframeAppBase: App Dimensions are ["<<width_<<","<<height_<<"]");
    ROS_WARN_STREAM("WallframeAppBase: Height Percentage set to " << height_perc_);
    this->height_ = int(double(height_) * height_perc_);
    ROS_WARN_STREAM("WallframeAppBase: Height is now " <<int(double(height_) * height_perc_));
    ROS_WARN_STREAM("WallframeAppBase: Set up successfully");
    return true;
  }

  void WallframeAppBase::userStateCallback(const wallframe_msgs::WallframeUserArrayConstPtr &user_packet){
    this->current_user_packet_ = *user_packet;
    this->user_data_ = this->current_user_packet_.users;
    updateUserData();
  }

  void WallframeAppBase::userEventCallback(const wallframe_msgs::WallframeUserEventConstPtr &user_event){
    current_user_event_ = *user_event;
    user_event_deque_.push_front(*user_event);
    while(user_event_deque_.size() > deque_size_){
      user_event_deque_.pop_back();
    }
  }

  void WallframeAppBase::wallframeEventCallback(const std_msgs::StringConstPtr &wallframe_event){
  }

  void WallframeAppBase::updateUserData(){
    // Get number of users
    this->num_users_ = this->user_data_.size();
    // Clear user structures
    this->focused_user_id_ = -1;
    this->active_user_ids_.clear();
    this->users_.clear();
    // Update new user data
    for(int i = 0;i<num_users_;i++){
      AppUser user;
      // User State
      user.user_id = user_data_[i].wallframe_id;
      active_user_ids_.push_back(user_data_[i].wallframe_id);
      user.focused = user_data_[i].focused;
      user.leaving = user_data_[i].leaving;
      user.joined = user_data_[i].joined;
      user.hands_together = user_data_[i].hands_together;
      user.hands_on_head = user_data_[i].hands_on_head;
      user.right_elbow_click = user_data_[i].right_elbow_click;
      user.left_elbow_click = user_data_[i].left_elbow_click;
      user.right_in_front = user_data_[i].right_in_front;
      user.left_in_front = user_data_[i].left_in_front;
      user.outside_workspace = user_data_[i].outside_workspace;
      // User Joints
      user.joint_names_ = user_data_[i].frame_names;
      if (user.focused == true)
        this->focused_user_id_ = user.user_id;
      for(unsigned int j_id = 0; j_id < user_data_[i].frame_names.size();j_id++){
        user.joint_positions_[user_data_[i].frame_names[j_id]] 
                = Eigen::Vector3d(  user_data_[i].translations_mm[j_id].x,
                                    user_data_[i].translations_mm[j_id].y,
                                    user_data_[i].translations_mm[j_id].z);
        user.joint_body_positions_[user_data_[i].frame_names[j_id]] 
                = Eigen::Vector3d(  user_data_[i].translations_body_mm[j_id].x,
                                    user_data_[i].translations_body_mm[j_id].y,
                                    user_data_[i].translations_body_mm[j_id].z);
      }
      this->users_[user.user_id] = user;
    }
  }

  bool WallframeAppBase::getFocusedUser(AppUser& u){
    if (focused_user_id_ != -1){
      u = this->users_[focused_user_id_];
      return true;
    }else{
      return false;
    }
  }

} // end namespace wallframe