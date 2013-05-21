#ifndef MODULAIR_MODULAIR_COMMON_H
#define MODULAIR_MODULAIR_COMMON_H

#include "QtGui"
#include <QtGui/QApplication>
#include <QObject>

#include <ros/ros.h>
#include <ros/node_handle.h>

namespace modulair{

  class ModulairTools{
  public:
    ModulairTools(){};
    ~ModulairTools(){};
  };

  class ModulairMainWindow : public QWidget{
    Q_OBJECT // must include this if you use Qt signals/slots
  public:
    ModulairMainWindow(QWidget* parent, QString name, ros::NodeHandle n);
    ~ModulairMainWindow(){};
    bool build();
  private:
    ros::NodeHandle node_;
    int x_,y_,width_,height_;
    QString name_;
    QString asset_path_;
  };

}


#endif
