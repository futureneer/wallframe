/*********************************************************************
* Software License Agreement (BSD License)
*
* Copyright (c) 2013, Johns Hopkins University
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above
* copyright notice, this list of conditions and the following
* disclaimer in the documentation and/or other materials provided
* with the distribution.
* * Neither the name of the Johns Hopkins University nor the names of its
* contributors may be used to endorse or promote products derived
* from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/*
 * Author: Kelleher Guerin, futureneer@gmail.com, Johns Hopkins University
 */

#include "wallframe_core/wallframe_app_base_qt.h"

namespace wallframe{
  //! Base application for using Qt with wallframe in c++
  /*!
    This class inherits the WallframeAppBase and also provides a parent Qt widget which will be resized to the specified size of the wall in the launch file
  */
  WallframeAppBaseQt::WallframeAppBaseQt(std::string app_name, ros::NodeHandle nh, int event_deque_size = 10): WallframeAppBase(app_name,nh,event_deque_size)
  {  
    // Build Widget
    this->setWindowFlags(Qt::FramelessWindowHint);
    this->setAutoFillBackground(true);
    this->setStyleSheet("background-color:#222222;");
    this->move(x_,y_);
    this->resize(width_,height_);
    this->setFocus();
    this->show();

    connect( &__ros_ok_timer, SIGNAL(timeout()), this, SLOT(checkRosOk()) );
    __ros_ok_timer.start(15);
  }
  //! Method to catch TERM sent to ROS and to close down the Qt thread as well
  void WallframeAppBaseQt::checkRosOk(){
    if(!ros::ok()){
      qApp->quit();
    }else{
      ros::spinOnce();
    }
  }

} // end namespace wallframe