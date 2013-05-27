#include "modulair_core/modulair_app_base.h"

namespace modulair{

	ModulairAppBase::ModulairAppBase(QString app_name, ros::NodeHandle nh, int event_deque_size = 10){  
    node_ = nh;
    name_ = app_name;
    deque_size_ = event_deque_size;
    initBaseApp();
    connect( &__ros_ok_timer, SIGNAL(timeout()), this, SLOT(checkRosOk()) );
    __ros_ok_timer.start(30);

	}

  void ModulairAppBase::checkRosOk(){
    if(!ros::ok()){
      qApp->quit();
    }
  }

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
        name_.toStdString().c_str(), node_.getNamespace().c_str());
      return false;
    }
    if (!node_.getParam("/modulair/core/params/y", y_)){
      ROS_ERROR("Modulair%s: No y position on parameter server (namespace: %s)",
        name_.toStdString().c_str(), node_.getNamespace().c_str());
      return false;
    }
    if (!node_.getParam("/modulair/core/params/width", width_)){
      ROS_ERROR("Modulair%s: No width on parameter server (namespace: %s)",
        name_.toStdString().c_str(), node_.getNamespace().c_str());
      return false;
    }
    if (!node_.getParam("/modulair/core/params/height", height_)){
      ROS_ERROR("Modulair%s: No height on parameter server (namespace: %s)",
        name_.toStdString().c_str(), node_.getNamespace().c_str());
      return false;
    }

    // Build Container Widget
    this->setWindowFlags(Qt::FramelessWindowHint);
    this->setAutoFillBackground(true);
    this->setStyleSheet("background-color:#222222;");

    this->move(x_,y_);
    this->resize(width_,height_);
    this->setFocus();
    this->show();

    return true;
  }

  void ModulairAppBase::userStateCallback(const modulair_msgs::ModulairUserArrayConstPtr &user_packet){
    current_user_data_ = *user_packet;
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

}