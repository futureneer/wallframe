#include "modulair_core/modulair_app_base.h"

namespace modulair{

	ModulairAppBase::ModulairAppBase(std::string app_name, ros::NodeHandle nh, int event_deque_size = 10){  
    node_ = nh;
    name_ = app_name;
    deque_size_ = event_deque_size;
    initBaseApp();
    // connect( &__ros_ok_timer, SIGNAL(timeout()), this, SLOT(checkRosOk()) );
    // __ros_ok_timer.start(15);

	}

  // void ModulairAppBase::checkRosOk(){
  //   if(!ros::ok()){}

  //   // if(!ros::ok()){
  //   //   qApp->quit();
  //   // }else{
  //   //   ros::spinOnce();
  //   // }
  // }

  bool ModulairAppBase::initBaseApp(){
    // Initialize User Listeners
    user_state_subscriber_ = node_.subscribe("/modulair/users/state", 1000, &ModulairAppBase::userStateCallback, this);
    user_event_subscriber_ = node_.subscribe("/modulair/users/events", 1000, &ModulairAppBase::userEventCallback, this);
    // Modulair Event Listener
    modulair_event_subscriber_ = node_.subscribe("/modulair/events", 1000, &ModulairAppBase::modulairEventCallback, this);
    // Publishers for outgoing messages
    toast_publisher_ = node_.advertise<std_msgs::String>("/modulair/info/toast", 1000);
    debug_publisher_ = node_.advertise<std_msgs::String>("/modulair/info/debug", 1000);

    // Get Container Widget Params
    if (!node_.getParam("/modulair/core/params/x", x_)){
      ROS_ERROR("Modulair%s: No x position on parameter server (namespace: %s)",
        name_.c_str(), node_.getNamespace().c_str());
      return false;
    }
    if (!node_.getParam("/modulair/core/params/y", y_)){
      ROS_ERROR("Modulair%s: No y position on parameter server (namespace: %s)",
        name_.c_str(), node_.getNamespace().c_str());
      return false;
    }
    if (!node_.getParam("/modulair/core/params/width", width_)){
      ROS_ERROR("Modulair%s: No width on parameter server (namespace: %s)",
        name_.c_str(), node_.getNamespace().c_str());
      return false;
    }
    if (!node_.getParam("/modulair/core/params/height", height_)){
      ROS_ERROR("Modulair%s: No height on parameter server (namespace: %s)",
        name_.c_str(), node_.getNamespace().c_str());
      return false;
    }
    if (!node_.getParam("/modulair/app/params/height_percentage", height_perc_)){
      ROS_ERROR("Modulair%s: No height percentage on parameter server (namespace: %s)",
        name_.c_str(), node_.getNamespace().c_str());
      return false;
    }

    ROS_WARN_STREAM("ModulairAppBase: App Dimensions are ["<<width_<<","<<height_<<"]");
    ROS_WARN_STREAM("ModulairAppBase: Height Percentage set to " << height_perc_);
    this->height_ = int(double(height_) * height_perc_);
    ROS_WARN_STREAM("ModulairAppBase: Height is now " <<int(double(height_) * height_perc_));

    // Build Container Widget
    // this->setWindowFlags(Qt::FramelessWindowHint);
    // this->setAutoFillBackground(true);
    // this->setStyleSheet("background-color:#222222;");

    // this->move(x_,y_);
    // this->resize(width_,int(double(height_) * height_perc_));
    // this->setFocus();
    // this->show();

    ROS_WARN_STREAM("ModulairAppBase: Set up successfully");

    return true;
  }

  void ModulairAppBase::userStateCallback(const modulair_msgs::ModulairUserArrayConstPtr &user_packet){
    this->current_user_packet_ = *user_packet;
    this->user_data_ = this->current_user_packet_.users;
    updateUserData();

    //// Debugging Test
    // AppUser u;
    // if(getFocusedUser(u)){
    //   ROS_WARN_STREAM( "["<<u.joint_positions_["torso"][0]<<","
    //                       <<u.joint_positions_["torso"][1]<<","
    //                       <<u.joint_positions_["torso"][2]<<"]");
    // }else{
    //   ROS_WARN_STREAM("No Focused User");
    // }
  }

  void ModulairAppBase::userEventCallback(const modulair_msgs::ModulairUserEventConstPtr &user_event){
    current_user_event_ = *user_event;
    user_event_deque_.push_front(*user_event);
    while(user_event_deque_.size() > deque_size_){
      user_event_deque_.pop_back();
    }
  }

  void ModulairAppBase::modulairEventCallback(const std_msgs::StringConstPtr &modulair_event){
  }

  void ModulairAppBase::updateUserData(){
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
      user.user_id = user_data_[i].modulair_id;
      active_user_ids_.push_back(user_data_[i].modulair_id);
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

  bool ModulairAppBase::getFocusedUser(AppUser& u){
    if (focused_user_id_ != -1){
      u = this->users_[focused_user_id_];
      return true;
    }else{
      return false;
    }
  }

} // end namespace modulair