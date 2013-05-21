#ifndef MODULAIR_MODULAIR_WIDGET_H
#define MODULAIR_MODULAIR_WIDGET_H

#include "QtGui"
#include <QtGui/QApplication>
#include <QObject>

#include <ros/ros.h>
#include <ros/node_handle.h>

#include <modulair_common/modulair_common.h>

namespace modulair{

  class ModulairWidget : public QWidget{
    Q_OBJECT // must include this if you use Qt signals/slots
  public:
    ModulairWidget(QWidget* parent, QString name, ros::NodeHandle n);
    ~ModulairWidget(){};
    UserCallback();
  private:
    ros::NodeHandle node_;
    int x_,y_,width_,height_;
    QString name_;
    QString asset_path_;
    ros::Subscriber sub; = n.subscribe("modulair_users", 1000, UserCallback);
  };

}


#endif
