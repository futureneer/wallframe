#ifndef modulair_app_base_h
#define modulair_app_base_h
// STD
#include <deque>
// QT 
#include "QtGui"
#include <QtGui/QApplication>
#include <QObject>
// ROS
#include <ros/ros.h>
#include <ros/node_handle.h>
#include <std_msgs/String.h>
// MODULAIR
#include <modulair_msgs/ModulairUserArray.h>
#include <modulair_msgs/ModulairUserEvent.h>
#include <modulair_msgs/ModulairUser.h>
#include <modulair_core/modulair_core.h>
// TF and EIGEN
#include <tf_conversions/tf_eigen.h>

namespace modulair{

	class ModulairAppBase : public QWidget{
		Q_OBJECT // must include to use Qt signals and slots
  public:
    ModulairAppBase(QString app_name, ros::NodeHandle nh, int event_deque_size);
    ~ModulairAppBase(){};

    void userStateCallback(const modulair_msgs::ModulairUserArrayConstPtr &user_packet);
    void userEventCallback(const modulair_msgs::ModulairUserEventConstPtr &user_event);
    void modulairEventCallback(const std_msgs::StringConstPtr &modulair_event);
    virtual bool build() = 0;
    virtual bool start() = 0;
    virtual bool stop() = 0;


  public Q_SLOTS:
    void checkRosOk();
  
  private:
    bool initBaseApp();

  protected:
    ros::NodeHandle node_;
    int x_,y_,width_,height_;
    int deque_size_;
    QString name_;
    QString asset_path_;
    ros::Subscriber user_state_subscriber_;
    ros::Subscriber user_event_subscriber_;
    ros::Subscriber modulair_event_subscriber_;
    ros::Publisher debug_publisher_;
    ros::Publisher toast_publisher_;
    QWidget tooltip_;
    QWidget container_widget_;
    QTimer __ros_ok_timer;
    modulair_msgs::ModulairUserArray current_user_data_;
    modulair_msgs::ModulairUserEvent current_user_event_;
    std::deque<modulair_msgs::ModulairUserEvent> user_event_deque_;
	};

}
#endif