#ifndef MODULAIR_MODULAIR_MENU_H
#define MODULAIR_MODULAIR_MENU_H

#include <modulair_common/modulair_common.h>

#include "QtGui"
#include <QtGui/QApplication>
#include <QObject>

#include <ros/ros.h>

namespace modulair{
  class ModulairMenu{
    public:
      ModulairMenu(ros::NodeHandle n);
      ~ModulairMenu();

      void build();
      void start(){};
      void stop(){};
  private:
  	  ros::NodeHandle node_;
      ModulairMainWindow* window_;
  };
}

#endif
