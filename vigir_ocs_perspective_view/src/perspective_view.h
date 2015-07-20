/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013-2015, Team ViGIR ( TORC Robotics LLC, TU Darmstadt, Virginia Tech, Oregon State University, Cornell University, and Leibniz University Hanover )
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Team ViGIR, TORC Robotics, nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
//@TODO_ADD_AUTHOR_INFO
/*
 * RobotModel class definition.
 * 
 * Author: Felipe Bacim.
 * 
 * Based on librviz_tutorials and the .
 * 
 * Latest changes (12/08/2012):
 * - created class
 */

#ifndef PERSPECTIVE_VIEW_H
#define PERSPECTIVE_VIEW_H

#include <QWidget>
#include <QElapsedTimer>

#include "base_3d_view.h"

namespace rviz
{
class RenderPanelCustom;
class ViewController;
class VisualizationManager;
class FPSViewController;
class OrbitViewController;
class VectorProperty;
class ViewsPanel;
}

namespace vigir_ocs
{
// Class "PerspectiveView" implements the RobotModel class with joint manipulation that can be added to any QT application.
class PerspectiveView: public Base3DView
{
    Q_OBJECT
public:
    PerspectiveView( QWidget *parent, Base3DView *copy_from = NULL, std::string base_frame = "/world" , std::string widget_name = "PerspectiveView" );
    PerspectiveView( Base3DView *copy_from = NULL, std::string base_frame = "/world" , std::string widget_name = "PerspectiveView", QWidget *parent = NULL );
    virtual ~PerspectiveView();

protected:
    virtual void timerEvent(QTimerEvent *event);
    virtual rviz::ViewController* getCurrentViewController();
    ros::Subscriber camera_transform_sub_;
    /**
      * ROS Callback: receives current camera pose
      */
    void processCameraTransform(const geometry_msgs::Pose::ConstPtr msg);

private:
    void init();
    rviz::ViewsPanel* views_panel_;
    QElapsedTimer view_change_timer_;
};
}
#endif // PERSPECTIVE_VIEW_H
