#include "modulair_core/modulair_app_base_qt.h"

namespace modulair{

  ModulairAppBaseQt::ModulairAppBaseQt(std::string app_name, ros::NodeHandle nh, int event_deque_size = 10): ModulairAppBase(app_name,nh,event_deque_size)
  {  
    // node_ = nh;
    // name_ = app_name;
    // deque_size_ = event_deque_size;
    // initBaseApp();

    // Build Container Widget
    this->setWindowFlags(Qt::FramelessWindowHint);
    this->setAutoFillBackground(true);
    this->setStyleSheet("background-color:#222222;");

    this->move(x_,y_);
    this->resize(width_,height_);
    this->setFocus();
    this->show();

    connect( &__ros_ok_timer, SIGNAL(timeout()), this, SLOT(checkRosOk()) );
    __ros_ok_timer.start(15);

  }

  void ModulairAppBaseQt::checkRosOk(){
    if(!ros::ok()){
      qApp->quit();
    }else{
      ros::spinOnce();
    }
  }

  // void ModulairAppBase::userStateCallback(const modulair_msgs::ModulairUserArrayConstPtr &user_packet){
  //   this->current_user_packet_ = *user_packet;
  //   this->user_data_ = this->current_user_packet_.users;
  //   updateUserData();
  // }

  // void ModulairAppBase::userEventCallback(const modulair_msgs::ModulairUserEventConstPtr &user_event){
  //   current_user_event_ = *user_event;
  //   user_event_deque_.push_front(*user_event);
  //   while(user_event_deque_.size() > deque_size_){
  //     user_event_deque_.pop_back();
  //   }
  // }

  // void ModulairAppBase::modulairEventCallback(const std_msgs::StringConstPtr &modulair_event){
  // }

  // void ModulairAppBase::updateUserData(){
  //   // Get number of users
  //   this->num_users_ = this->user_data_.size();
  //   // Clear user structures
  //   this->focused_user_id_ = -1;
  //   this->active_user_ids_.clear();
  //   this->users_.clear();
  //   // Update new user data
  //   for(int i = 0;i<num_users_;i++){
  //     AppUser user;
  //     // User State
  //     user.user_id = user_data_[i].modulair_id;
  //     active_user_ids_.push_back(user_data_[i].modulair_id);
  //     user.focused = user_data_[i].focused;
  //     user.leaving = user_data_[i].leaving;
  //     user.joined = user_data_[i].joined;
  //     user.hands_together = user_data_[i].hands_together;
  //     user.hands_on_head = user_data_[i].hands_on_head;
  //     user.right_elbow_click = user_data_[i].right_elbow_click;
  //     user.left_elbow_click = user_data_[i].left_elbow_click;
  //     user.right_in_front = user_data_[i].right_in_front;
  //     user.left_in_front = user_data_[i].left_in_front;
  //     user.outside_workspace = user_data_[i].outside_workspace;
  //     // User Joints
  //     user.joint_names_ = user_data_[i].frame_names;
  //     if (user.focused == true)
  //       this->focused_user_id_ = user.user_id;
  //     for(unsigned int j_id = 0; j_id < user_data_[i].frame_names.size();j_id++){
  //       user.joint_positions_[user_data_[i].frame_names[j_id]] 
  //               = Eigen::Vector3d(  user_data_[i].translations_mm[j_id].x,
  //                                   user_data_[i].translations_mm[j_id].y,
  //                                   user_data_[i].translations_mm[j_id].z);
  //       user.joint_body_positions_[user_data_[i].frame_names[j_id]] 
  //               = Eigen::Vector3d(  user_data_[i].translations_body_mm[j_id].x,
  //                                   user_data_[i].translations_body_mm[j_id].y,
  //                                   user_data_[i].translations_body_mm[j_id].z);
  //     }
  //     this->users_[user.user_id] = user;
  //   }
  // }

  // bool ModulairAppBase::getFocusedUser(AppUser& u){
  //   if (focused_user_id_ != -1){
  //     u = this->users_[focused_user_id_];
  //     return true;
  //   }else{
  //     return false;
  //   }
  // }

} // end namespace modulair