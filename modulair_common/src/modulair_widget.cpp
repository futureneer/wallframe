#include <modulair_common/modulair_widget.h>

namespace modulair{

std::string GetEnv( const std::string & var ) {
  const char * val = ::getenv( var.c_str() );
  if ( val == 0 ) {
    return "";
  }
  else {
    return val;
  }
}


ModulairWidget::UserCallback(const std_msgs::String::ConstPtr& msg){

ModulairWidget::ModulairWidget(QWidget* parent, QString name, ros::NodeHandle n){

  // Set Node Handle
  node_ = n;
  name_ = name;

  user_sub_ = node_.subscribe("kinect_users", 100, UserCallback);

  // Get Asset path from environment variable
  asset_path_ = QString(GetEnv("MODULAIR_ASSET_PATH").c_str());

  // Get Params
  if (!node_.getParam("modulair/params/x", x_)){
    ROS_ERROR("Modulair: No x position on parameter server (namespace: %s)",
      node_.getNamespace().c_str());
    return false;
  }
  if (!node_.getParam("modulair/params/y", y_)){
    ROS_ERROR("Modulair: No y position on parameter server (namespace: %s)",
      node_.getNamespace().c_str());
    return false;
  }
  if (!node_.getParam("modulair/params/width", width_)){
    ROS_ERROR("Modulair: No width on parameter server (namespace: %s)",
      node_.getNamespace().c_str());
    return false;
  }
  if (!node_.getParam("modulair/params/height", height_)){
    ROS_ERROR("Modulair: No height on parameter server (namespace: %s)",
      node_.getNamespace().c_str());
    return false;
  }
 
  this->setWindowFlags(Qt::FramelessWindowHint);
  this->setAutoFillBackground(true);
  this->setFocus();
  this->show();

  return 0;
}

} //namespace