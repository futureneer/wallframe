#ifndef modulair_app_base_qt_h
#define modulair_app_base_qt_h
// STD
// #include <deque>
// QT 
#include "QtGui"
#include <QtGui/QApplication>
#include <QObject>
// ROS
// #include <ros/ros.h>
// #include <ros/node_handle.h>
// #include <std_msgs/String.h>
// MODULAIR
#include <modulair_core/modulair_app_base.h>
// #include <modulair_msgs/ModulairUserEvent.h>
// #include <modulair_msgs/ModulairUser.h>
// TF and EIGEN
// #include <tf_conversions/tf_eigen.h>

namespace modulair{

  // class AppUser{
  // public:
  //   AppUser(){};
  //   ~AppUser(){};

  //   Eigen::Vector3d jtPosByName(std::string j){ return joint_positions_[j]; };
  //   Eigen::Vector3d jtPosBodyByName(std::string j){ return joint_body_positions_[j]; };
  //   Eigen::Vector3d jtPosById(int id){ return joint_positions_[joint_names_[id]]; };
  //   Eigen::Vector3d jtPosBodyById(int id){ return joint_positions_[joint_names_[id]]; };
  //   std::string jtNameById(int id){ return joint_names_[id]; }
  //   int jtIdByName(std::string id){
  //       for(unsigned int i=0;i<joint_names_.size();i++){
  //           if(joint_names_[i] == id){
  //               return i;
  //           } 
  //       }
  //       return -1;
  //   }

  //   std::map<std::string, Eigen::Vector3d> joint_positions_;
  //   std::map<std::string, Eigen::Vector3d> joint_body_positions_;
  //   std::vector<std::string> joint_names_;
  //   int user_id;
  //   bool focused;
  //   bool leaving;
  //   bool joined;
  //   bool hands_together;
  //   bool hands_on_head;
  //   bool right_elbow_click;
  //   bool left_elbow_click;
  //   bool right_in_front;
  //   bool left_in_front;
  //   bool outside_workspace;
  // };

  // typedef std::map<int,AppUser> AppUserMap;

  class ModulairAppBaseQt : public QWidget, public ModulairAppBase {
    Q_OBJECT // must include to use Qt signals and slots
  public:
    ModulairAppBaseQt(std::string app_name, ros::NodeHandle nh, int event_deque_size);
    ~ModulairAppBaseQt(){};

    // void userStateCallback(const modulair_msgs::ModulairUserArrayConstPtr &user_packet);
    // void userEventCallback(const modulair_msgs::ModulairUserEventConstPtr &user_event);
    // void modulairEventCallback(const std_msgs::StringConstPtr &modulair_event);
    // void updateUserData();
    // bool getFocusedUser(AppUser& u);
    // Virtual Methods
    virtual bool build() = 0;
    virtual bool start() = 0;
    virtual bool stop() = 0;
    virtual bool pause() = 0;
    virtual bool resume() = 0;

  public Q_SLOTS:
    void checkRosOk();
  
  private:
    // bool initBaseQtApp();

  protected:
    // ros::NodeHandle node_;
    // int x_,y_,width_,height_;
    // double height_perc_;
    // unsigned int deque_size_;
    std::string name_;
    QString asset_path_;
    // int num_users_, focused_user_id_;
    // AppUserMap users_;
    // std::vector<int> active_user_ids_;
    // ros::Subscriber user_state_subscriber_;
    // ros::Subscriber user_event_subscriber_;
    // ros::Subscriber modulair_event_subscriber_;
    // ros::Publisher debug_publisher_;
    // ros::Publisher toast_publisher_;
    QWidget tooltip_;
    // QWidget container_widget_;
    QTimer __ros_ok_timer;
    // modulair_msgs::ModulairUserArray current_user_packet_;
    // std::vector<modulair_msgs::ModulairUser> user_data_;
    // modulair_msgs::ModulairUserEvent current_user_event_;
    // std::deque<modulair_msgs::ModulairUserEvent> user_event_deque_;
  };

}
#endif