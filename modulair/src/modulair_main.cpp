#include <modulair/modulair_manager.h>
#include <ros/ros.h>

#include <stdio.h>
#include <stdlib.h>
#include <iostream>

int main(int argc, char* argv[]){

  ros::init(argc,argv, "modulair_menu");
  ROS_ERROR_STREAM("STARTING MODULAIR MENU");
  int ex = system("roslaunch modulair_menu modulair_menu.launch");
  ROS_ERROR_STREAM("MODULAIR MENU STARTED SUCCESSFULLY");
  
  return 0;
}
