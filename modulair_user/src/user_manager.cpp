#include "openni_tracker_fusion/UserManager.h"
using namespace std;

UserManager::UserManager() : it_(node_)
{
    num_active_kinect_ = 0;
    kinpack_fused_ = new KinectPacket(-1);

    rgb_sub_1 = it_.subscribe(   "/kinect1/rgb/image_rect_color",
                                 1, &UserManager::rgbCallback, this);
    depth_sub_1 = it_.subscribe( "/kinect1/depth_registered/image_rect",
                                 1, &UserManager::depthCallback, this);

    rgb_sub_2 = it_.subscribe(   "/kinect2/rgb/image_rect_color",
                                 1, &UserManager::rgbCallback, this);
    depth_sub_2 = it_.subscribe( "/kinect2/depth_registered/image_rect",
                                 1, &UserManager::depthCallback, this);

    rgb_sub_3 = it_.subscribe(   "/kinect2/rgb/image_rect_color",
                                 1, &UserManager::rgbCallback, this);
    depth_sub_3 = it_.subscribe( "/kinect2/depth_registered/image_rect",
                                 1, &UserManager::depthCallback, this);

    user_sub_ = node_.subscribe("/users",1,&UserManager::userCallback, this);

    ROS_ERROR("%s","Kinect Fusor Starting Up");
}

void UserManager::startStandalone()
{
    ros::Rate rate(30.0);
    while(node_.ok()){
        runtime++;
        checkSkeletonsUpToDate();
        mergeSkeletons();
        rate.sleep();
        ros::spinOnce();
    }
}

void UserManager::checkSkeletonsUpToDate()
{
    for(KinectPacketMap::iterator it = kinpacks_.begin();it!=kinpacks_.end();++it){
        int tracker_id = it->first;
        std::vector<int> to_delete;
        for( KinectSkeletonMap::iterator its = it->second->users_.begin();
             its != it->second->users_.end();
             ++its){
            // // Increment each skeleton's updateCounter
            // its->second.updateCounter++;
            // // If a skeleton has not been updated for more than 20 seconds, erase it.
            // if(its->second.updateCounter > 150){
            //   std::cerr<<"<<< KinectFusor >>> Tracker "<<tracker_id<<", User "<<its->first<<" Lost"<<std::endl;
            //   it->second->users_.erase(its);
            // }

            // If a skeleton has not been updated for more than 20 seconds, erase it.
            if(its->second.center_of_mass == Eigen::Vector3d(0,0,0)) {
                // std::cerr<<"<<< KinectFusor >>> Tracker "<<tracker_id<<", User "<<its->first<<" Lost"<<std::endl;
                // it->second->users_.erase(its);
                to_delete.push_back(its->first);
            }
        }
        for (unsigned int i = 0; i < to_delete.size(); ++i)
        {
            it->second->users_.erase(it->second->users_.find(to_delete[i]));
            std::cerr<<"<<< KinectFusor >>> Tracker "<<tracker_id<<", User "<<to_delete[i]<<" Lost"<<std::endl;
        }
    }
}

double UserManager::vectDist(Eigen::Vector3d a, Eigen::Vector3d b)
{
    Eigen::Vector3d c = a-b;
    return c.norm();
}

void UserManager::mergeSkeletons()
{
    switch(kinpacks_.size()){
    case 0:
        num_active_kinect_ = 0;
        break;
    case 1:
        num_active_kinect_ = 1;
        kinpack_fused_ = kinpacks_.begin()->second;
        kinpack_fused_->users_available_ = kinpack_fused_->users_.size();
        break;
        // case 2:
        //   kinpack_fused_->users_.clear();
        //   num_active_kinect_ = 2;
        //   KinectPacket* packA, packB;

        //   // Iterate through different Kinects
        //   KinectPacketMap::iterator tracker_it = kinpacks_.begin();
        //   pack_a = tracker_it->second;
        //   pack_b = (tracker_it++)->second;

        //   KinectSkeletonMap::iterator it_a, it_b;

        //   for( it_a = pack_a->users_.begin(); it_a != pack_a->users_.end(); ++it_a){
        //     for( it_b = pack_b->users_.begin(); it_b != pack_b->users_.end(); ++it_b){

        //       if(vectDist(it_a->second.joints[2].translation,it_b->second.joints[2].translation) < .2){

        //       }

        //     }
        //   }

        //   for(trak_it = kinpacks_.begin();trak_it!=kinpacks_.end();++trak_it){
        //     int tracker_id = trak_it->first;
        //     KinectPacket* kinpack = trak_it->second;
        //     KinectSkeletonMap::iterator user_it;
        //     for( user_it=kinpack->users_.begin();user_it!=kinpack->users_.end();++user_it){

        //     }
        //   }

        //   break;
    default:
        // Merge however many kinects are available > 1
        break;
    }
}

KinectPacket* UserManager::getFusedKinectPacket(){return kinpack_fused_;}
int UserManager::getNumActiveKinects(){return num_active_kinect_;}

void UserManager::userCallback(const openni_msgs::UserArrayConstPtr &userArray_msg)
{
    // Get number of users
    int num_users = userArray_msg->users.size();

    if( num_users > 0){
        // Get user message frame id
        std::string tracker_frame_id = userArray_msg->users[0].header.frame_id;

        // Find the kinect id from the frame id
        int tracker_id = findNumTracker(tracker_frame_id);

        // If this message has come from a new tracker, add a new KinectPacket with the id
        if(kinpacks_.count(tracker_id) == 0){
            std::cerr<<"     <<< KinectFusor >>> Adding New Tracker...";
            kinpacks_[tracker_id] = new KinectPacket(tracker_id);
            std::cerr<<"     Done"<<std::endl;
        }

        for(int u = 0;u<num_users;u++){
            // Get a single user message
            openni_msgs::User user_msg = userArray_msg->users[u];
            int user_id = user_msg.uid;
            // Get center of mass
            Eigen::Vector3d user_com( user_msg.center_of_mass.x,
                                      user_msg.center_of_mass.y,
                                      user_msg.center_of_mass.z);

            // Go through each joint / frame
            for(unsigned int j = 0;j<user_msg.frames.size();j++){
                // Build frame name from /Users topic
                std::stringstream s;
                s << user_id;
                std::string tf_frame_id = tracker_frame_id.substr(1,17) + user_msg.frames[j] + "_" + s.str();

                // Get frame from tf
                tf::StampedTransform trans;
                if(tf_listener_.frameExists(tf_frame_id)){
                    try{
                        tf_listener_.waitForTransform("/wall_frame", tf_frame_id, ros::Time(0), ros::Duration(3));
                        tf_listener_.lookupTransform("/wall_frame", tf_frame_id, ros::Time(0), trans);
                        Eigen::Affine3d frame;
                        tf::TransformTFToEigen(trans,frame);

                        // If User already exists
                        if(kinpacks_[tracker_id]->users_.count(user_id) > 0){
                            KinectJoint jt;
                            jt.conf = user_msg.confs[j];
                            jt.translation = frame.translation();
                            jt.translation_mm = jt.translation*1000;
                            jt.frame = frame;
                            jt.projective = Eigen::Vector3d(user_msg.projective[j].x,
                                                            user_msg.projective[j].y,
                                                            user_msg.projective[j].z);
                            kinpacks_[tracker_id]->users_[user_id].joints[j] = jt;
                            kinpacks_[tracker_id]->users_[user_id].updateCounter = 0;
                            kinpacks_[tracker_id]->users_[user_id].center_of_mass = user_com;


                            // If User doesnt exist
                        }else{
                            if(user_com != Eigen::Vector3d(0,0,0)){
                                KinectSkeleton new_user;
                                KinectJoint jt;
                                jt.conf = user_msg.confs[j];
                                jt.translation = frame.translation();
                                jt.translation_mm = jt.translation*1000;
                                jt.frame = frame;
                                jt.projective = Eigen::Vector3d(user_msg.projective[j].x,
                                                                user_msg.projective[j].y,
                                                                user_msg.projective[j].z);
                                new_user.joints[j] = jt;
                                new_user.updateCounter = 0;
                                new_user.center_of_mass = user_com;
                                kinpacks_[tracker_id]->users_[user_id] = new_user;
                                std::cerr<<"<<< KinectFusor >>> Created New User: "<<user_id<<std::endl;
                            }
                        }

                    }catch(tf::TransformException ex){
                        ROS_ERROR("%s",ex.what());
                    }
                }
            }// end frames
        }// end users
    }// mum users > 0
}

void UserManager::rgbCallback(const sensor_msgs::ImageConstPtr &image){
    cv_bridge::CvImagePtr cv_ptr;
    std::string frame_id = image->header.frame_id;
    int tracker_id = findNumKinect(frame_id);

    if(kinpacks_.count(tracker_id) == 0){
        kinpacks_[tracker_id] = new KinectPacket(tracker_id);
    }

    try{
        cv_ptr = cv_bridge::toCvCopy(image, enc::BGR8);
        kinpacks_[tracker_id]->rgb_ = cv_ptr->image;
    }catch(cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s",e.what());
        return;
    }
}

void UserManager::depthCallback(const sensor_msgs::ImageConstPtr &image){
    // std::cerr<<"Depth Image Recieved"<<std::endl;
    cv_bridge::CvImagePtr cv_ptr;
    std::string frame_id = image->header.frame_id;
    int tracker_id = findNumKinect(frame_id);

    if(kinpacks_.count(tracker_id) == 0){
        kinpacks_[tracker_id] = new KinectPacket(tracker_id);
    }

    try{
        cv_ptr = cv_bridge::toCvCopy(image, enc::TYPE_32FC1);
        kinpacks_[tracker_id]->depth_ = cv_ptr->image;
    }catch(cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s",e.what());
        return;
    }
}

bool UserManager::strContains(std::string s, std::string a)
{
    size_t found;
    found = s.find(a);
    if(found!=std::string::npos)
        return true;
    else
        return false;
};

int UserManager::findNumKinect(std::string s)
{
    int num;
    sscanf(s.c_str(),"/kinect%d",&num);
    return num;
};

int UserManager::findNumTracker(std::string s)
{
    int num;
    sscanf(s.c_str(),"/kinect_tracker_%d",&num);
    return num;
};

cv::Mat UserManager::convertDepthTo8BitImage(cv::Mat img)
{ 
    double min_range = .5;
    double max_range = 5.5;
    cv::Mat img8(img.rows, img.cols, CV_8UC1);
    for(int i = 0; i < img.rows; i++){
        float* Di = img.ptr<float>(i);
        char* Ii = img8.ptr<char>(i);
        for(int j = 0; j < img.cols; j++){
            Ii[j] = (char) (255*((Di[j]-min_range)/(max_range-min_range)));
        }
    }
    return img8;
}

/*// Helpful for Debugging ////////////////////////////////////////////
  // std::cerr<<frame_id<<std::endl;
  // std::cerr<<findNum(frame_id)<<std::endl;



// KinectSkeletonMap UserManager::getSkelsWorld(){return skels_world_;}

// KinectSkeletonMap UserManager::getSkels(int kinect_id){return skels_kinect_[kinect_id];}

// cv::Mat UserManager::getDepthImageFused(){return depth_image_fused_;}

// cv::Mat UserManager::getDepthImage(int kinect_id){return depth_images_[kinect_id];}

// cv::Mat UserManager::getRGBImageFused(){return rgb_image_fused_;}

// cv::Mat UserManager::getRGBImage(int kinect_id){return rgb_images_[kinect_id];}


// bool UserManager::usersAvailable(){return users_available_;}



  // tf_listener_.getFrameStrings(tf_frame_names);

  // for(int i = 0;i<frames.size();i++){
  //   tf::StampedTransform trans;
  //   if(tf_listener_.frameExists(tf_frame_names[i])){





  //     if( strContains(tf_frame_names[i],"/kinect_tracker_") &&
  //         !strContains(tf_frame_names[i],"wall_frame") &&
  //         !strContains(tf_frame_names[i],"openni_depth_frame")) {
  //       std::string tracker_name, joint_name;
  //       tracker_name = tf_frame_names[i].substr(1,17);
  //       joint_name = tf_frame_names[i].substr(18);

  //       try{
  //         tf_listener_.lookupTransform("/wall_frame", tf_frame_names[i],
  //                                     ros::Time(0), trans);
  //       }catch(tf::TransformException ex){
  //         ROS_ERROR("%s",ex.what());
  //       }
  //     }
  //   }
  // }

  // for(int u = 0;u<num_users;u++){
  //   openni_msgs::User user_msg = userArray_msg->users[u];
  //   int id = user_msg.uid;
  //   int tracker_id = user_msg.header.frame_id;
  //   for(int j = 0;j<user_msgs.frames.size():j++){
  //     std::string frame_id = user_msgs.frames[j];
  //     std::string tf_frame_id = "/kinect_tracker_"

  //   }
  // }
  // for(int i = 0;i<users_msg->users.size();i++){
  //   openni_msgs::User user_msg = users_msg->users[i];
  //   int id = user_msg.uid;
  //   if(id < 6){
  //     for(int j = 0;j<user_msg.frames.size();j++){
  //       skels_k0[id-1].exists = true;
  //       skels_k0[id-1].ids[0] = id;
  //       skels_k0[id-1].joints[j].conf = user_msg.confs[j];
  //       tf::Transform t;
  //       tf::transformMsgToTF(user_msg.transforms[j],t);
  //       Eigen::Affine3d te;
  //       tf::TransformTFToEigen(t,te);
  //       skels_k0[id-1].joints[j].translation = te.translation();
  //       skels_k0[id-1].joints[j].affine = te;
  //     }
  //   }
  // }

*/
