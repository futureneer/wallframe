/*
 * EnumWrapper.h
 *
 *  Created on: May 6, 2012
 *      Author: Kyle Mercer
 */

#ifndef ENUMWRAPPER_H_
#define ENUMWRAPPER_H_

#include <ros/ros.h>
#include <ros/package.h>

#include <XnOpenNI.h>
#include <XnCodecIDs.h>
#include <XnCppWrapper.h>

#define CHECK_RC(nRetVal, what)											\
	if (nRetVal != XN_STATUS_OK)										\
	{																	\
		ROS_ERROR("%s failed: %s", what, xnGetStatusString(nRetVal));	\
		exit(1);														\
	}

class EnumWrapper
{
	public:
		EnumWrapper(xn::Context &, xn::NodeInfo);
		xn::UserGenerator* getUserGeneratorNode();
		xn::DepthGenerator* getDepthGeneratorNode();

	private:
		xn::DepthGenerator g_DepthGenerator;
		xn::UserGenerator  g_UserGenerator;
		XnCallbackHandle hUserCallback;
		XnCallbackHandle hCalibrationCallback;
		XnCallbackHandle hPoseCallback;
};


#endif /* ENUMWRAPPER_H_ */
