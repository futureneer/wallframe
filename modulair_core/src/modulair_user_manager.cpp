#include <modulair_common/modulair_user_manager.h>

namespace modulair{

std::string GetEnv( const std::string & var ) {
  const char * val = ::getenv( var.c_str() );
  if ( val == 0 ) {
    return "";
  }
  else {
    return val;
  }
}

void ModulairUserManager::mergeSkeletons(){
    switch(kinpacks_.size()){
    case 0:
        num_active_kinect_ = 0;
        break;
    case 1:
        num_active_kinect_ = 1;
        kinpack_fused_ = kinpacks_.begin()->second;
        kinpack_fused_->users_available_ = kinpack_fused_->users_.size();
        break;
    default:
        // Merge however many kinects are available > 1
        break;
    }
}

double ModulairUserManager::vectDist(Eigen::Vector3d a, Eigen::Vector3d b){
    Eigen::Vector3d c = a-b;
    return c.norm();
}

void ModulairUserManager::checkSkeletonsUpToDate()
{
    for(KinectPacketMap::iterator it = kinpacks_.begin();it!=kinpacks_.end();++it){
        int tracker_id = it->first;
        std::vector<int> to_delete;
        for( KinectSkeletonMap::iterator its = it->second->users_.begin();
             its != it->second->users_.end();
             ++its){

            // If a skeleton has not been updated for more than 20 seconds, erase it.
            if(its->second.center_of_mass == Eigen::Vector3d(0,0,0)) {
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

ModulairUserManager::UserCallback(const openni_msgs::UserArrayConstPtr &userArray_msg){
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

ModulairUserManager::ModulairUserManager(ros::NodeHandle n){

  // Set Node Handle and Subscribe
  node_ = n;
  user_sub_ = node_.subscribe("/users",1,&tracker_fusor::userCallback, this;

}


int ModulairUserManager::findNumKinect(std::string s){
    int num;
    sscanf(s.c_str(),"/kinect%d",&num);
    return num;
};

int ModulairUserManager::findNumTracker(std::string s){
    int num;
    sscanf(s.c_str(),"/kinect_tracker_%d",&num);
    return num;
};

} //namespace