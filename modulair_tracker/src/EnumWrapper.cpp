/*
 * EnumWrapper.cpp
 *
 *  Created on: May 6, 2012
 *      Author: Kyle Mercer
 */

#include "EnumWrapper.h"

XnBool g_bNeedPose   = FALSE;
XnChar g_strPose[20] = "";


void XN_CALLBACK_TYPE User_NewUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie)
{
	ROS_INFO("New User %d", nId);

	if (g_bNeedPose)
		generator.GetPoseDetectionCap().StartPoseDetection(g_strPose, nId);
	else
		generator.GetSkeletonCap().RequestCalibration(nId, TRUE);
}

void XN_CALLBACK_TYPE User_LostUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie)
{
	ROS_INFO("Lost user %d", nId);
}

void XN_CALLBACK_TYPE UserCalibration_CalibrationStart(xn::SkeletonCapability& capability, XnUserID nId, void* pCookie)
{
	ROS_INFO("Calibration started for user %d", nId);
}

void XN_CALLBACK_TYPE UserCalibration_CalibrationEnd(xn::SkeletonCapability& capability, XnUserID nId, XnBool bSuccess, void* pCookie)
{
	if (bSuccess)
	{
		ROS_INFO("Calibration complete, start tracking user %d", nId);
		capability.StartTracking(nId);
	}
	else
	{
		xn::UserGenerator *g_UserGenerator = (xn::UserGenerator*) pCookie;
		ROS_INFO("Calibration failed for user %d", nId);
		if (g_bNeedPose)
			g_UserGenerator -> GetPoseDetectionCap().StartPoseDetection(g_strPose, nId);
		else
			capability.RequestCalibration(nId, TRUE);
	}
}

void XN_CALLBACK_TYPE UserPose_PoseDetected(xn::PoseDetectionCapability& capability, XnChar const* strPose, XnUserID nId, void* pCookie)
{
	xn::UserGenerator *g_UserGenerator = (xn::UserGenerator*) pCookie;
    ROS_INFO("Pose %s detected for user %d", strPose, nId);
    capability.StopPoseDetection(nId);
    g_UserGenerator -> GetSkeletonCap().RequestCalibration(nId, TRUE);
}

//Constructor
EnumWrapper::EnumWrapper(xn::Context &g_Context, xn::NodeInfo info)
{
	static int i = 0;
	XnStatus nRetVal;
	const XnProductionNodeDescription description = info.GetDescription();
	ROS_INFO("Device #%d, Vendor: %s, Name: %s, Instance: %s", i, description.strVendor, description.strName, info.GetInstanceName());

	nRetVal = g_Context.CreateProductionTree(info);
	CHECK_RC(nRetVal, "CreateProductionTree");

	nRetVal = info.GetInstance(g_DepthGenerator);
	CHECK_RC(nRetVal, "info.GetInstance");

	XnMapOutputMode mode;
	mode.nXRes = 640;
	mode.nYRes = 480;
	mode.nFPS = 30;
	nRetVal = g_DepthGenerator.SetMapOutputMode(mode);
	CHECK_RC(nRetVal, "SetMapOutputMode");

	xn::Query pQuery;
	pQuery.SetCreationInfo(info.GetCreationInfo());
	nRetVal = g_UserGenerator.Create(g_Context, &pQuery);
	CHECK_RC(nRetVal, "Find user generator");

	if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_SKELETON))
	{
		ROS_INFO("Supplied user generator(s) don't (doesn't) support skeleton ");
		exit(1);
	}

	g_UserGenerator.RegisterUserCallbacks(User_NewUser, User_LostUser, NULL, hUserCallback);
	g_UserGenerator.GetSkeletonCap().RegisterCalibrationCallbacks(UserCalibration_CalibrationStart, UserCalibration_CalibrationEnd, &g_UserGenerator, hCalibrationCallback);

	if (g_UserGenerator.GetSkeletonCap().NeedPoseForCalibration())
	{
		g_bNeedPose = TRUE;
		if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_POSE_DETECTION))
		{
			ROS_INFO("Pose required, but not supported");
			exit(1);
		}
		g_UserGenerator.GetPoseDetectionCap().RegisterToPoseCallbacks(UserPose_PoseDetected, NULL, &g_UserGenerator, hPoseCallback);

		g_UserGenerator.GetSkeletonCap().GetCalibrationPose(g_strPose);
	}
	g_UserGenerator.GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_ALL);
	i++;
}

xn::UserGenerator* EnumWrapper::getUserGeneratorNode()
{
	return &g_UserGenerator;
}

xn::DepthGenerator* EnumWrapper::getDepthGeneratorNode()
{
	return &g_DepthGenerator;
}
