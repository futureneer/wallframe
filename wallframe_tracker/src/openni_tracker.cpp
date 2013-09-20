/*
 * multi_tracker.cpp
 *
 *  Created on: May 5, 2012
 *      Author: Kyle Mercer & Kelleher Guerin
 */

//ROS deps
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_broadcaster.h>
#include <kdl/frames.hpp>
#include <openni_camera/openni_driver.h>
#include <openni_camera/openni_device.h>
#include <openni_camera/openni_device_kinect.h>

//OPENNI deps
#include <XnOpenNI.h>
#include <XnCodecIDs.h>
#include <XnCppWrapper.h>

//STD lib
#include <vector>
#include <boost/tokenizer.hpp>
#include <cmath>

//My deps
#include "EnumWrapper.h"
#include <modulair_msgs/TrackerUser.h>
#include <modulair_msgs/TrackerUserArray.h>

using namespace std;
using namespace openni_wrapper;

string device_id = ""; //TODO: make this local once we are able to make more than one UserGenerator in a single thread

//BEGIN TF PUBLISHER FUNCTIONS
void publishTransform(modulair_msgs::TrackerUser& user_msg, XnUserID const& user, XnSkeletonJoint const& joint, string const& frame_id, string const& child_frame_id, EnumWrapper *kinect) {
    static tf::TransformBroadcaster br;
    xn::DepthGenerator *g_DepthGenerator = kinect -> getDepthGeneratorNode();
    xn::UserGenerator *g_UserGenerator = kinect -> getUserGeneratorNode();

    XnSkeletonJointPosition joint_position;
    g_UserGenerator -> GetSkeletonCap().GetSkeletonJointPosition(user, joint, joint_position);
    double x = -joint_position.position.X / 1000.0;
    double y = joint_position.position.Y / 1000.0;
    double z = joint_position.position.Z / 1000.0;

    XnSkeletonJointOrientation joint_orientation;
    g_UserGenerator -> GetSkeletonCap().GetSkeletonJointOrientation(user, joint, joint_orientation);

    XnFloat* m = joint_orientation.orientation.elements;
    KDL::Rotation rotation(m[0], m[1], m[2],
    					   m[3], m[4], m[5],
    					   m[6], m[7], m[8]);
    double qx, qy, qz, qw;
    rotation.GetQuaternion(qx, qy, qz, qw);

    char child_frame_no[128];
    snprintf(child_frame_no, sizeof(child_frame_no), "%s_%d", child_frame_id.c_str(), user);

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(x, y, z));
    transform.setRotation(tf::Quaternion(qx, -qy, -qz, qw));

    // #4994
    tf::Transform change_frame;
    change_frame.setOrigin(tf::Vector3(0, 0, 0));
    tf::Quaternion frame_rotation;
    frame_rotation.setEulerZYX(1.5708, 0, 1.5708);
    change_frame.setRotation(frame_rotation);

    transform = change_frame * transform;

    // Send TF Transform of Joint
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame_id, child_frame_no));

    // Create Transform Message for UserMsg
    geometry_msgs::Transform transform_msg;
    tf::transformTFToMsg(transform,transform_msg);
    // Create Vector Message for projective image joint coordinates
    geometry_msgs::Vector3 vector_msg;
    XnPoint3D p = joint_position.position;
    g_DepthGenerator -> ConvertRealWorldToProjective(1,&p,&p);
    vector_msg.x = p.X;
    vector_msg.y = p.Y;
    vector_msg.z = 0;

    user_msg.projective.push_back(vector_msg);
    user_msg.confs.push_back(joint_position.fConfidence);
    user_msg.transforms.push_back(transform_msg);

}

void publishTransforms(const std::string& frame_id, EnumWrapper *kinect) {
    XnUserID users[15];
    XnUInt16 users_count = 15;
    xn::UserGenerator *g_UserGenerator = kinect -> getUserGeneratorNode();
    g_UserGenerator -> GetUsers(users, users_count);
    static std::map<int, std::vector<float> > CoM_Z_History; //<uid, CoM >

    /////////////// KGUERIN /////////////////////
    ros::NodeHandle n;
    // std::string prefix = tf::getPrefixParam(n);
    std::string prefix;
    ros::param::get("~/tf_prefix",prefix);



    static ros::Publisher user_array_publisher = n.advertise<modulair_msgs::TrackerUserArray>("/modulair/users/raw", 10);

    modulair_msgs::TrackerUserArray user_array_msg;
    user_array_msg.numUsers = 0;

    /////////////////////////////////////////////
    
      for (int i = 0; i < users_count; ++i) {
        XnUserID user = users[i];
        if (!g_UserGenerator -> GetSkeletonCap().IsTracking(user))
            continue;

        XnPoint3D CoM;
        g_UserGenerator -> GetCoM(user, CoM);

        //
        modulair_msgs::TrackerUser user_msg;
        user_msg.header.seq = 0;
        user_msg.header.stamp = ros::Time::now();
        user_msg.header.frame_id = "/" + prefix + "/" + frame_id;
        // user_msg.header.frame_id = frame_id;
        user_msg.uid = user;
        // TrackerUser center of Mass
        geometry_msgs::Vector3 com_msg;
        com_msg.x = CoM.X;
        com_msg.y = CoM.Y;
        com_msg.z = CoM.Z;
        user_msg.center_of_mass = com_msg;
        //

        user_msg.frames.push_back("head");
        publishTransform(user_msg, user, XN_SKEL_HEAD,           frame_id, "head", kinect);
        user_msg.frames.push_back("neck");
        publishTransform(user_msg, user, XN_SKEL_NECK,           frame_id, "neck", kinect);
        user_msg.frames.push_back("torso");
        publishTransform(user_msg, user, XN_SKEL_TORSO,          frame_id, "torso", kinect);

        user_msg.frames.push_back("right_shoulder");
        publishTransform(user_msg, user, XN_SKEL_RIGHT_SHOULDER, frame_id, "right_shoulder", kinect);
        user_msg.frames.push_back("left_shoulder");
        publishTransform(user_msg, user, XN_SKEL_LEFT_SHOULDER,  frame_id, "left_shoulder", kinect);

        user_msg.frames.push_back("right_elbow");
        publishTransform(user_msg, user, XN_SKEL_RIGHT_ELBOW,    frame_id, "right_elbow", kinect);
        user_msg.frames.push_back("left_elbow");
        publishTransform(user_msg, user, XN_SKEL_LEFT_ELBOW,     frame_id, "left_elbow", kinect);
        
        user_msg.frames.push_back("right_hand");
        publishTransform(user_msg, user, XN_SKEL_RIGHT_HAND,     frame_id, "right_hand", kinect);
        user_msg.frames.push_back("left_hand");
        publishTransform(user_msg, user, XN_SKEL_LEFT_HAND,      frame_id, "left_hand", kinect);

        user_msg.frames.push_back("right_hip");
        publishTransform(user_msg, user, XN_SKEL_RIGHT_HIP,      frame_id, "right_hip", kinect);
        user_msg.frames.push_back("left_hip");
        publishTransform(user_msg, user, XN_SKEL_LEFT_HIP,       frame_id, "left_hip", kinect);
        
        user_msg.frames.push_back("right_knee");
        publishTransform(user_msg, user, XN_SKEL_RIGHT_KNEE,     frame_id, "right_knee", kinect);
        user_msg.frames.push_back("left_knee");
        publishTransform(user_msg, user, XN_SKEL_LEFT_KNEE,      frame_id, "left_knee", kinect);
        
        user_msg.frames.push_back("right_foot");
        publishTransform(user_msg, user, XN_SKEL_RIGHT_FOOT,     frame_id, "right_foot", kinect);
        user_msg.frames.push_back("left_foot");
        publishTransform(user_msg, user, XN_SKEL_LEFT_FOOT,      frame_id, "left_foot", kinect);
        
        user_array_msg.users.push_back(user_msg);
        user_array_msg.numUsers++;
    }
    
    user_array_publisher.publish(user_array_msg);
}
//END TF PUBLISHER FUNCTIONS

/**
 * A utility function that parses a device_id given as either a bus address, an index starting at 1,
 * or a devices serial number.
 *
 * @param OpenNIDriver& driver the singleton OpenNIDriver which manages all devices connected.
 * @param std::string& device_id the device ID given as a bus address, an index starting at 1, or a devices serial number.
 * @return Returns the OpenNIDevice if it's found, NULL if not found.
 */
boost::shared_ptr <OpenNIDevice> getDeviceByID(OpenNIDriver& driver, string &device_id)
{
	try
	{
		if (device_id.find('@') != string::npos)
		{
			size_t pos = device_id.find('@');
			unsigned bus = atoi(device_id.substr(0, pos).c_str());
			unsigned address = atoi(device_id.substr(pos + 1, device_id.length() - pos - 1).c_str());
			ROS_INFO("Searching for device with bus@address = %d@%d\n", bus, address);
			return driver.getDeviceByAddress(bus, address);
		}
		else if (device_id[0] == '#')
		{
			unsigned index = atoi(device_id.c_str() + 1);
			ROS_INFO("Searching for device with index = %d\n", index);
			return driver.getDeviceByIndex(index - 1);
		}
		else
		{
			ROS_INFO("Searching for device with serial number = '%s'\n", device_id.c_str());
			return driver.getDeviceBySerialNumber(device_id);
		}
	}
	catch (const OpenNIException& exception)
	{
		ROS_INFO("Unable to retrieve requested device. Reason: %s", exception.what());
		exit(-1);
	}
	cerr << "ERROR: Should have never reached this step in getDeviceByID()" << endl;
}


int main(int argc, char **argv)
{

	ros::init(argc, argv, "openni_tracker");
	ros::NodeHandle nh;
	ros::NodeHandle pnh("~"); //private node handler

	xn::Context g_Context;
	xn::NodeInfoList nodeList;
	XnStatus nRetVal;

	g_Context.Init();
	nRetVal = g_Context.EnumerateProductionTrees(XN_NODE_TYPE_DEPTH, NULL, nodeList, NULL);
	CHECK_RC(nRetVal, "EnumerateProductionTrees");

	if(!pnh.getParam("device_id", device_id))
	{
		ROS_ERROR("device_id not found on Param Server! Check your launch file!");
		return -1;
	}

	OpenNIDriver& driver = OpenNIDriver::getInstance();

	while(driver.getNumberDevices() == 0)
	{
			cerr << "No devices connected.... waiting for devices to be connected" << endl;
			boost::this_thread::sleep(boost::posix_time::seconds(3)); //wait 3 seconds before trying again.
			driver.updateDeviceList();
	}

	for (unsigned deviceIdx = 0; deviceIdx < driver.getNumberDevices(); ++deviceIdx)
	{
		printf("%u. Device on bus %03u:%02u is a %s (%03x) from %s (%03x) with serial id \'%s\'\n",
				deviceIdx + 1, driver.getBus(deviceIdx),
				driver.getAddress(deviceIdx),
				driver.getProductName(deviceIdx),
				driver.getProductID(deviceIdx),
				driver.getVendorName(deviceIdx),
				driver.getVendorID(deviceIdx),
				driver.getSerialNumber(deviceIdx));
	}

	//find our target device for this thread
	boost::shared_ptr <OpenNIDevice> device = getDeviceByID(driver, device_id);
	if(device == NULL)
	{
		ROS_ERROR("No device with ID: %s. Exiting..", &device_id);
		return -1;
	}

	//compare device to our list and create its UserGenerator
	EnumWrapper *kinect;
	for(xn::NodeInfoList::Iterator nodeDepthIterator = nodeList.Begin(); nodeDepthIterator != nodeList.End(); ++nodeDepthIterator)
	{
		//cerr << "Creation info: " << (*nodeDepthIterator).GetCreationInfo() << endl;
		for(xn::NodeInfoList::Iterator neededDeviceNodeIterator = (*nodeDepthIterator).GetNeededNodes().Begin(); neededDeviceNodeIterator != (*nodeDepthIterator).GetNeededNodes().End(); ++neededDeviceNodeIterator)
		{
			//cerr << "Needed info: " << (*neededDeviceNodeIterator).GetCreationInfo() << endl;
			if(strcmp((*neededDeviceNodeIterator).GetCreationInfo(), device -> getConnectionString()) == 0) //found target device!
			{
				kinect = new EnumWrapper(g_Context, (*nodeDepthIterator));
				break; //no need to inspect other devices
			}
		}
	}

	nRetVal = g_Context.StartGeneratingAll();
	CHECK_RC(nRetVal, "StartGenerating");
	ros::Rate r(30);

	string frame_id("openni_depth_frame");
	pnh.getParam("camera_frame_id", frame_id);

	while (ros::ok())
	{
		g_Context.WaitAndUpdateAll();
		publishTransforms(frame_id, kinect);
		r.sleep();
	}

	g_Context.Shutdown();
	delete kinect;
	return 0;
}
