#!/usr/bin/env python
#####################################################################
# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Johns Hopkins University
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# # Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.
# # Redistributions in binary form must reproduce the above
# copyright notice, this list of conditions and the following
# disclaimer in the documentation and/or other materials provided
# with the distribution.
# # Neither the name of the Johns Hopkins University nor the names of its
# contributors may be used to endorse or promote products derived
# from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#####################################################################

###
# Author: Kelleher Guerin, futureneer@gmail.com, Johns Hopkins University
###

import roslib; roslib.load_manifest('wallframe_core')
import rospy, os, sys, glob, fnmatch
import rosgraph.masterapi
from ros import roslaunch
from roslaunch.core import Node
from roslaunch.scriptapi import ROSLaunch
import subprocess

from geometry_msgs.msg import Transform
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool
from std_msgs.msg import String

from wallframe_msgs.msg import WallframeUser as user_msg
from wallframe_msgs.msg import WallframeUserArray as user_array_msg
from wallframe_msgs.msg import WallframeUserEvent as user_event_msg
from wallframe_msgs.msg import TrackerUser
from wallframe_msgs.msg import TrackerUserArray as tracker_msg

import wallframe_core
from wallframe_core.srv import *

class AppLaunchFile():
  def __init__(self,name,launch_name,pack,launch_path,package_path,active):
    self.launch_file_path_ = launch_path
    self.package_path_ = package_path
    self.name_ = name
    self.launch_name_ = launch_name
    self.package_ = pack
    self.active_ = active

class WallframeAppManager():
  def __init__(self):
    # Member variables
    self.apps_in_manifest_ = []
    self.apps_ = {}
    self.active_app_launchers_ = {}
    # Roslaunch
    self.roslaunch_master_ = ROSLaunch()
    # ROS Init
    rospy.init_node('wallframe_app_manager',anonymous=True)
    # ROS Subscribers
    self.wallframe_event_sub = rospy.Subscriber("/wallframe/events", String, self.wallframe_event_cb)
    
    # Load Apps
    self.load_application_manifest()
    self.load_applications()

    # ROS Services
    print''
    self.load_app_srv_ = rospy.Service('app_manager/load_app', wallframe_core.srv.load_app, self.load_app_service)
    rospy.logwarn("WallframeAppManager: Service Ready [ load_app ]")
    self.close_app_srv_ = rospy.Service('app_manager/close_app', wallframe_core.srv.close_app, self.close_app_service)
    rospy.logwarn("WallframeAppManager: Service Ready [ close_app ]")
    self.close_all_apps_srv_ = rospy.Service('app_manager/close_all_apps', wallframe_core.srv.close_all_apps, self.close_all_apps_service)
    rospy.logwarn("WallframeAppManager: Service Ready [ close_all_apps ]")
    
    # Running
    rospy.logwarn("WallframeAppManager: Started")  
    rospy.spin()
    
    # Quitting
    rospy.logwarn("WallframeAppManager: Cleaning up running applications")  
    self.shutdown_all_apps()
    self.clean_up()
    rospy.logwarn("WallframeAppManager: Finished")
  
  def load_app_service(self,req):
    message = "WallframeAppManager: Service Call to LOAD APP ["+req.app_name+"]"
    if self.launch_app(req.app_name) == True:
      return "LOAD APP -- SUCCESS"
    else:
      return "LOAD APP -- FAILED -- App not found or could not load"
    pass

  def close_app_service(self,req):
    message = "WallframeAppManager: Service Call to CLOSE APP ["+req.app_name+"]"
    app_full_name = "wallframe_app_"+req.app_name
    if app_full_name in self.active_app_launchers_.keys():
      rospy.logwarn(message + "SUCCESS")
      self.shutdown_app(req.app_name)
      return "CLOSE APP -- SUCCESS"
    else:
      rospy.logwarn(message + "FAILED -- App not running")
      return "CLOSE APP -- FAIL -- App not running"
    pass

  def close_all_apps_service(self,req):
    message = "WallframeAppManager: Service Call to CLOSE ALL APPS -- "
    if len(self.active_app_launchers_) == 0:
      rospy.logwarn(message + "FAILED -- No apps are running")
      return "CLOSE ALL APPS -- FAIL -- No apps are running" 
    else:
      self.shutdown_all_apps()
      rospy.logwarn(message + "SUCCESS")
      return "CLOSE ALL APPS -- SUCCESS"
    pass

  def wallframe_event_cb(self):
    pass

  def clean_up(self):
    for app_id,app in self.apps_.items():
      if rospy.has_param("/wallframe/core/available_app/" + app_id):
        rospy.delete_param("/wallframe/core/available_apps/" + app_id)
        print("App parameters for [" + app_id + "] cleaned up")

      if rospy.has_param("/wallframe/core/available_apps"):
        rospy.delete_param("/wallframe/core/available_apps")
        print("Remaining parameters cleaned up")
    pass    

  def shutdown_all_apps(self):
    for full_app_name,app_process in self.active_app_launchers_.items():
      app_process.terminate()
      while app_process.poll() == None:
        pass
      if rospy.has_param("/wallframe/core/apps/running/" + full_app_name):
        rospy.delete_param("/wallframe/core/apps/running/" + full_app_name)
      rospy.logwarn("WallframeAppManager: App [" + full_app_name + "] shutdown successfully")
    self.active_app_launchers_.clear()

  def shutdown_app(self,app_name):
    full_app_name = "wallframe_app_" + app_name
    app_process  = self.active_app_launchers_[full_app_name]
    app_process.terminate()
    while app_process.poll() == None:
      pass
    if rospy.has_param("/wallframe/core/apps/running/" + full_app_name):
      rospy.delete_param("/wallframe/core/apps/running/" + full_app_name)
    del self.active_app_launchers_[full_app_name]
    rospy.logwarn("WallframeAppManager: App [" + full_app_name + "] shutdown successfully")

  def launch_app(self,app_name):
    if app_name in self.apps_in_manifest_:
      full_app_name = "wallframe_app_" + app_name
      message = "AdjutantLauncher: Launching [" + full_app_name + "] ..."

      if full_app_name not in self.apps_.keys():
        rospy.logerr(message + " FAILED! File [" + full_app_name + "] not found in path")
        return False
      else:
        app_name = self.apps_[full_app_name].name_
        launch_name = self.apps_[full_app_name].launch_name_
        launch_package = self.apps_[full_app_name].package_
        launch_args = ['roslaunch', launch_package, launch_name]

        P = subprocess.Popen(launch_args)
        # (stdoutdata, stderrdata) = P.communicate()
        self.active_app_launchers_[full_app_name] = P
        success = True
        self.apps_[full_app_name].active_ = True

        rospy.logwarn(message + " SUCCESS! File [" + launch_name + "]")
        rospy.set_param("/wallframe/core/apps/running/" + app_name, [self.apps_[full_app_name]])
        return True

    else:
      rospy.logerr("WallframeAppManager: Requested app [" + app_name + "] not found in manifest.")

  def load_applications(self):
    if rospy.has_param('/wallframe/core/paths/application_path'):
      self.app_path_ = rospy.get_param('/wallframe/core/paths/application_path')
    else:
      rospy.logerr("WallframeAppManager: application path not found on parameter server")
    rospy.logwarn("WallframeAppManager: Loading Applications from [" + self.app_path_ + "]")

    available_app_list = {}

    for app_full_path in self.find_files(self.app_path_, '*.launch'):
      split_path = app_full_path.split("/")
      if 'wallframe_app' in split_path[len(split_path)-1]:
        app_launch_file = split_path[len(split_path)-1]
        app_launch_name = app_launch_file[:len(app_launch_file)-len('.launch')]
        app_short_name = app_launch_name[len('wallframe_app_'):]
        if split_path[len(split_path)-2] == 'launch':
          app_launch_package = split_path[len(split_path)-3]
        else:
          app_launch_package = split_path[len(split_path)-2]

        full_path_split = app_full_path.split('/')
        if full_path_split[len(full_path_split)-2] == 'launch':
          app_package_path = '/'.join(full_path_split[:len(full_path_split)-2])
        else:
          app_package_path = '/'.join(full_path_split[:len(full_path_split)-1])
        
        available_app_list[app_short_name] = app_package_path
        rospy.set_param("/wallframe/core/available_app/" + app_launch_name, app_package_path)


        A = AppLaunchFile(app_launch_name,app_launch_file,app_launch_package,app_full_path,app_package_path,False)
        self.apps_[app_launch_name] = A

        rospy.loginfo("WallframeAppManager: Found [" + app_launch_name + "]  in package  [" + app_launch_package + "]")

    rospy.set_param("/wallframe/core/available_apps", available_app_list)

    pass

  def load_application_manifest(self):
    if rospy.has_param('/wallframe/core/paths/app_manifest'):
      self.app_manifest_ = rospy.get_param('/wallframe/core/paths/app_manifest')
    else:
      rospy.logerr("WallframeAppManager: application manifest not found on parameter server")
    rospy.logwarn("WallframeAppManager: Loading Manifest")
    print("Applications found in app_manifest:")
    manifest_elements = self.app_manifest_.split("\n")
    for e in manifest_elements:
      if e != '': # blank line check
        if '#' not in e: # commented line check
          app = e[len('wallframe_app_'):] # strip off wallframe prefix
          print "-- " + app
          self.apps_in_manifest_.append(app)

  def find_files(self, directory, pattern):
    for root, dirs, files in os.walk(directory):
        for basename in files:
            if fnmatch.fnmatch(basename, pattern):
                filename = os.path.join(root, basename)
                yield filename

# MAIN
if __name__ == '__main__':
  m = WallframeAppManager()