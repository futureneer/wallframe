#include <modulair_menu/modulair_menu.h>
#include <ros/ros.h>

#include <stdio.h>
#include <stdlib.h>
#include <iostream>

int main(int argc, char* argv[]){

  ros::init(argc,argv, "modulair_menu");
  
  ROS_ERROR_STREAM("I RAN!!");

  return 0;
}
