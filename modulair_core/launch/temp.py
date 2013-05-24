#!/usr/bin/env python
import roslib; roslib.load_manifest('adjutant_managers')
import rospy, os, sys, glob, fnmatch
import rosgraph.masterapi
from ros import roslaunch
from roslaunch.core import Node
from roslaunch.scriptapi import ROSLaunch
import subprocess

class LaunchFile():
def __init__(self,name,pack,path,active):
self.path_ = path
self.name_ = name
self.package_ = pack
self.active_ = active

class AdjutantLauncher:
def __init__(self):

# Members
self.launch_manifests_ = {}
self.active_launchers_ = {}
self.launch_files_ = {}
self.active_files_ = []

# Roslaunch
self.roslaunch_master_ = ROSLaunch()

rospy.init_node('adjutant_launch',anonymous=True)
self.launch_path_ = rospy.get_param("/adjutant/managers/launch_path")
rospy.logwarn("AdjutantLauncher: Loading all launch files")
self.load_launch_files()
self.load_launch_manifest()

# test launch manifest
self.launch_manifest('merlin_simulated_teleop')
# c = 0
# while c < 20:
# rospy.sleep(1)
# print c
# c = c + 1
# self.kill_all_active()
rospy.spin()

rospy.logwarn("AdjutantLauncher: Finished")


def kill_all_active(self):
for name in self.active_files_:
self.active_launchers_[name].terminate()
msg_string = "AdjutantLauncher: Terminating [" + name + "]"
# active.terminate()
# self.launch_files_[active_name].active_ = False
self.active_launchers_.clear()
pass

def launch_manifest(self,name):
manifest_name = name
if manifest_name not in self.launch_manifests_.keys():
rospy.logwarn("AdjutantLauncher: INVALID MANIFEST NAME")
else:
rospy.logwarn("AdjutantLauncher: Launching Modules in Manifest [" + manifest_name + "]")
for manifest_launch_file in self.launch_manifests_[manifest_name]:
msg_string = "AdjutantLauncher: Launching [" + manifest_launch_file + "]"
success = False
for launch_file in self.launch_files_.values():
if manifest_launch_file in launch_file.name_:
launch_name = launch_file.name_
launch_package = launch_file.package_
launch_args = ['roslaunch', launch_package, launch_name]
P = subprocess.Popen(launch_args)
self.active_launchers_[launch_name] = P
self.active_files_.append(launch_name)
success = True
launch_file.active_ = True
if success == False:
rospy.logerr("AdjutantLauncher: ERROR, file [" + manifest_launch_file + "] not found in path")
else:
rospy.logwarn(msg_string + " with file [" + launch_name + "]")
pass

def load_launch_manifest(self):
if rospy.has_param('/adjutant/managers/launch_manifest'):
self.launch_manifest_file_ = rospy.get_param('/adjutant/managers/launch_manifest')
else:
rospy.logerr("Launch Manifest loaded from parameter server.")

manifests = self.launch_manifest_file_.split("\n\n")

for manifest in manifests:
manifest_elements = manifest.split("\n")
manifest_name = manifest_elements[0].strip(":")
rospy.logwarn("AdjutantLauncher: Found manifest " + manifest_name)
manifest_launch_files = []
for element in manifest_elements:
if ':' not in element:
manifest_launch_files.append(element)
rospy.loginfo("-- Launch element: " + element)

self.launch_manifests_[manifest_name] = manifest_launch_files

def load_launch_files(self):
for filename in self.find_files(self.launch_path_, '*.launch'):
file_sp = filename.split('/')
if 'adjutant' in file_sp[len(file_sp)-1]:
l_path = filename
l_name = file_sp[len(file_sp)-1]
if file_sp[len(file_sp)-2] == 'launch':
l_pack = file_sp[len(file_sp)-3]
else:
l_pack = file_sp[len(file_sp)-2]

L = LaunchFile(l_name,l_pack,l_path,False)
self.launch_files_[l_name] = L

rospy.loginfo("Found: [" + l_name + "] in package [" + l_pack + "]")

def find_files(self, directory, pattern):
for root, dirs, files in os.walk(directory):
for basename in files:
if fnmatch.fnmatch(basename, pattern):
filename = os.path.join(root, basename)
yield filename




# MAIN
if __name__ == '__main__':
launch_manager = AdjutantLauncher()