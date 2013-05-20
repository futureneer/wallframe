#include "openni_tracker_fusion/tracker_fusor.h"
#include <ros/ros.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "tracker_fusion");
  tracker_fusor trackerFusor;
  ROS_INFO("Fusion Started\n");
  trackerFusor.startStandalone();
  std::cerr<<"Stuff"<<std::endl;
  ros::spin();
  // ros::Rate r(30);
  // while(ros::ok())
  // {
  // 	ros::spinOnce();
  // 	r.sleep();
  // }
  return 0;
}
