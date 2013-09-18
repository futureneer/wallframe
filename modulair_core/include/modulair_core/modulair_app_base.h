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

#ifndef modulair_app_base_h
#define modulair_app_base_h
// STD
#include <deque>
// ROS
#include <ros/ros.h>
#include <ros/node_handle.h>
#include <std_msgs/String.h>
// MODULAIR
#include <modulair_msgs/ModulairUserArray.h>
#include <modulair_msgs/ModulairUserEvent.h>
#include <modulair_msgs/ModulairUser.h>
// TF and EIGEN
#include <tf_conversions/tf_eigen.h>

namespace modulair{

  class AppUser{
  public:
    AppUser(){};
    ~AppUser(){};

    // joint names are ['head', 'neck', 'torso', 'right_shoulder', 'left_shoulder', 'right_elbow', 'left_elbow', 'right_hand', 'left_hand', 'right_hip', 'left_hip', 'right_knee', 'left_knee', 'right_foot', 'left_foot']


    Eigen::Vector3d jtPosByName(std::string j){ return joint_positions_[j]; };
    Eigen::Vector3d jtPosBodyByName(std::string j){ return joint_body_positions_[j]; };
    Eigen::Vector3d jtPosById(int id){ return joint_positions_[joint_names_[id]]; };
    Eigen::Vector3d jtPosBodyById(int id){ return joint_positions_[joint_names_[id]]; };
    std::string jtNameById(int id){ return joint_names_[id]; }
    int jtIdByName(std::string id){
      for(unsigned int i=0;i<joint_names_.size();i++){
        if(joint_names_[i] == id){
          return i;
        } 
      }
      return -1;
    }
    std::map<std::string, Eigen::Vector3d> joint_positions_;
    std::map<std::string, Eigen::Vector3d> joint_body_positions_;
    std::vector<std::string> joint_names_;
    int user_id;
    bool focused;
    bool leaving;
    bool joined;
    bool hands_together;
    bool hands_on_head;
    bool right_elbow_click;
    bool left_elbow_click;
    bool right_in_front;
    bool left_in_front;
    bool outside_workspace;
  };

  typedef std::map<int,AppUser> AppUserMap;

  class ModulairAppBase{
  public:
    ModulairAppBase(std::string app_name, ros::NodeHandle nh, int event_deque_size);
    ~ModulairAppBase(){};

    void userStateCallback(const modulair_msgs::ModulairUserArrayConstPtr &user_packet);
    void userEventCallback(const modulair_msgs::ModulairUserEventConstPtr &user_event);
    void modulairEventCallback(const std_msgs::StringConstPtr &modulair_event);
    void updateUserData();
    bool getFocusedUser(AppUser& u);
    // Virtual Methods
    virtual bool build() = 0;
    virtual bool start() = 0;
    virtual bool stop() = 0;
    virtual bool pause() = 0;
    virtual bool resume() = 0;
  private:
    bool initBaseApp();
  protected:
    ros::NodeHandle node_;
    int x_,y_,width_,height_;
    double height_perc_;
    unsigned int deque_size_;
    std::string name_;
    int num_users_, focused_user_id_;
    AppUserMap users_;
    std::vector<int> active_user_ids_;
    ros::Subscriber user_state_subscriber_;
    ros::Subscriber user_event_subscriber_;
    ros::Subscriber modulair_event_subscriber_;
    ros::Publisher debug_publisher_;
    ros::Publisher toast_publisher_;
    modulair_msgs::ModulairUserArray current_user_packet_;
    std::vector<modulair_msgs::ModulairUser> user_data_;
    modulair_msgs::ModulairUserEvent current_user_event_;
    std::deque<modulair_msgs::ModulairUserEvent> user_event_deque_;
  };

}
#endif