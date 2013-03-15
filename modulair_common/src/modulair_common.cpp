#include <modulair_common/modulair_common.h>

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

ModulairMainWindow::ModulairMainWindow(QWidget* parent, QString name, ros::NodeHandle n)
{
  // Set Node Handle
  node_ = n;
  name_ = name;
}

bool ModulairMainWindow::build(){

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

  std::string background_path;
  if (!node_.getParam("modulair/directories/background_image", background_path)){
    ROS_ERROR("Modulair: No height on parameter server (namespace: %s)",
      node_.getNamespace().c_str());
    return false;
  }

  QString background_full_path = asset_path_ + "/" + QString(background_path.c_str());
  // Set up QT Window
  ROS_INFO_STREAM("Modulair: Background image found at " << background_full_path.toStdString());
  QPixmap pic(background_full_path);
  QPalette palette;
  palette.setBrush(this->backgroundRole(), QBrush(pic));
  this->setPalette(palette);

  this->setWindowFlags(Qt::FramelessWindowHint);
  this->setAutoFillBackground(true);
  this->setStyleSheet("background-color:#222222;");

  this->move(x_,y_);
  this->resize(width_,height_);
  this->setFocus();
  this->show();
  ROS_INFO_STREAM("Modulair "<< name_.toStdString() <<" Window Set Up");

  return 0;
}

ModulairMainWindow::~ModulairMainWindow()
{
	
}

} //namespace