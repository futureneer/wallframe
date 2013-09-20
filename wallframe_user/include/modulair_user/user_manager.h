#ifndef _tracker_fusor_h
#define _tracker_fusor_h

#include <tf/tf.h>
#include <vector>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/Bool.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <openni_msgs/UserArray.h>
#include <openni_msgs/User.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

namespace enc = sensor_msgs::image_encodings;

struct KinectJoint{
  double conf;
  Eigen::Vector3d translation;
  Eigen::Vector3d translation_mm;
  Eigen::Affine3d frame;
  Eigen::Vector3d projective;
};

typedef std::map<int,KinectJoint> KinectJointMap;

struct KinectSkeleton{
  KinectJointMap joints;
  int updateCounter;
  Eigen::Vector3d center_of_mass;  
};

typedef std::map<int,KinectSkeleton> KinectSkeletonMap;
typedef std::map<int,KinectSkeletonMap> KinectSkeletonGroup;
typedef std::map<int,cv::Mat> CVMatMap;

class KinectPacket{
  public:
    // Constructorsator-build
    KinectPacket(int ID){
      id_ = ID;
      available_ = false;
      users_available_ = false;
      std::cout<<"<<< KinectFusor >>> New Kinect Packet Created"<<std::endl;
    };
    ~KinectPacket(){}
    // Member Functions
    KinectSkeletonMap getUsers(){return users_;}
    cv::Mat getRGB(){return rgb_;}
    cv::Mat getDepth(){return depth_;}
    // Members
    KinectSkeletonMap users_;
    cv::Mat rgb_;
    cv::Mat depth_;
    int users_available_;
    int available_;
    int id_;
};

typedef std::map<int,KinectPacket*> KinectPacketMap;

class tracker_fusor {
  public:
    tracker_fusor();
    ~tracker_fusor(){};
    void init(int kinects_available);
    void startStandalone();
  protected:
    ros::NodeHandle node_;
    // Member Functions
    cv::Mat convertDepthTo8BitImage(cv::Mat img);
    void mergeSkeletons();
    void checkSkeletonsUpToDate();
    bool strContains(std::string s, std::string a);
    int findNumKinect(std::string s);
    int findNumTracker(std::string s);
    double vectDist(Eigen::Vector3d a, Eigen::Vector3d b);
    // Get Functions
    KinectPacket* getFusedKinectPacket();
    int getNumActiveKinects();
    // Subscriber Callbacks
    void depthCallback(const sensor_msgs::ImageConstPtr &image);
    void rgbCallback(const sensor_msgs::ImageConstPtr &image);
    void userCallback(const openni_msgs::UserArrayConstPtr &users_msg);
    // Member Variables
    int runtime;
    int num_active_kinect_;
    KinectPacketMap kinpacks_;
    KinectPacket* kinpack_fused_;
    // Image Subscribers
    image_transport::ImageTransport it_;
    image_transport::Subscriber rgb_sub_1;
    image_transport::Subscriber rgb_sub_2;
    image_transport::Subscriber rgb_sub_3;
    image_transport::Subscriber depth_sub_1;
    image_transport::Subscriber depth_sub_2;
    image_transport::Subscriber depth_sub_3;
    // ROS Subscriber
    ros::Subscriber user_sub_;
    // TF Objects
    tf::TransformListener tf_listener_;
    tf::TransformBroadcaster tf_broadcaster_;
};

#endif
