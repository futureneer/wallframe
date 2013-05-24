#ifndef MODULAIR_MODULAIR_USER_MANAGER_H
#define MODULAIR_MODULAIR_USER_MANAGER_H

#include <ros/ros.h>
#include <ros/node_handle.h>

#include <modulair_common/modulair_common.h>
#include <openni_msgs/UserArray.h>
#include <openni_msgs/User.h>

namespace modulair{

  struct UserJoint{
    double conf;
    Eigen::Vector3d translation;
    Eigen::Vector3d translation_mm;
    Eigen::Affine3d frame;
    Eigen::Vector3d projective;
  };

  typedef std::map<int,UserJoint> KinectJointMap;

  class ModulairUserManager{
  public:
    ModulairUserManager(QWidget* parent, QString name, ros::NodeHandle n);
    ~ModulairUserManager(){};
    UserCallback();
  private:
    ros::NodeHandle node_;
    ros::Subscriber user_sub_;
  };

}


#endif
