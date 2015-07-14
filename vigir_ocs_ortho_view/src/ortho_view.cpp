/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013-2015, TORC Robotics, LLC ( Team ViGIR )
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
@TODO_ADD_AUTHOR_INFO
/*
 * OrthoView class implementation.
 *
 * Author: Felipe Bacim.
 *
 * Based on librviz_tutorials.
 *
 * Latest changes (12/08/2012):
 */

#include <QLabel>
#include <QVBoxLayout>
#include <QHBoxLayout>

#include <OGRE/OgreRenderWindow.h>

#include "rviz/visualization_manager.h"
#include "rviz/view_controller.h"
#include "rviz/tool_manager.h"
#include "rviz/view_manager.h"
#include "rviz/display.h"
#include "ortho_view_controller_custom.h"
#include <render_panel_custom.h>

#include <vigir_perception_msgs/EnvironmentRegionRequest.h>

#include "ortho_view.h"

namespace vigir_ocs
{
// Constructor for OrthoView.  This does most of the work of the class.
OrthoView::OrthoView( Base3DView* copy_from, std::string base_frame, std::string widget_name, QWidget* parent )
    : Base3DView( copy_from, base_frame, widget_name, parent )
    , setting_pose_(false)
{
    // set the camera to our own ortho view controller; needs to be initialized
    if(!copy_from)
    {
        // if this is the first view to be created for this context, we can set the viewcontroller as usual
        rviz::ViewManager* view_man_ = manager_->getViewManager();
        ortho_view_controller_ = view_man_->create( "rviz/OrthoViewControllerCustom" );
        ((rviz::OrthoViewControllerCustom*)ortho_view_controller_)->initialize(manager_, render_panel_);
        view_man_->setCurrentFrom(ortho_view_controller_);
        render_panel_->setViewController(ortho_view_controller_);
    }
    else
    {
        // if it already has a context, we don't overwrite the viewcontroller
        ortho_view_controller_ = new rviz::OrthoViewControllerCustom();
        ((rviz::OrthoViewControllerCustom*)ortho_view_controller_)->initialize(manager_, render_panel_);
        render_panel_->setViewController(ortho_view_controller_);
    }

    // make sure we're still able to cancel set goal pose
    QObject::connect(render_panel_, SIGNAL(signalKeyPressEvent(QKeyEvent*)), this, SLOT(keyPressEvent(QKeyEvent*)));

    previous_tool_ = manager_->getToolManager()->getCurrentTool();
}

// Destructor.
OrthoView::~OrthoView()
{
}

void OrthoView::timerEvent(QTimerEvent *event)
{
    // call the base3dview version of the timerevent
    Base3DView::timerEvent(event);

    ortho_view_controller_->update(0,0);

//    float lastFPS, avgFPS, bestFPS, worstFPS;

//    render_panel_->getRenderWindow()->getStatistics( lastFPS, avgFPS, bestFPS, worstFPS );
//    std::cout << "Camera (" << /*ortho_view_controller_->subProp( "View Plane" )->getValue(). <<*/ "): " << lastFPS << ", " << avgFPS << ", " << bestFPS << ", " << worstFPS << std::endl;

    //std::cout << "camera position: " << render_panel_->getCamera()->getPosition().x << ", " << render_panel_->getCamera()->getPosition().z << ", " << render_panel_->getCamera()->getPosition().z << std::endl;
    //std::cout << "camera direction: " << render_panel_->getCamera()->getDirection().x << ", " << render_panel_->getCamera()->getDirection().z << ", " << render_panel_->getCamera()->getDirection().z << std::endl;
}

void OrthoView::setViewPlane(const QString& view_plane)
{
    //rviz::ViewManager* view_man_ = manager_->getViewManager();
    ortho_view_controller_->subProp( "View Plane" )->setValue( view_plane.toStdString().c_str() );
    //view_man_->setCurrentFrom( ortho_view_controller_ );

    // set the camera position
    //if("XY")
    //render_panel_->getCamera()->setPosition(0,0,5);
    //render_panel_->getCamera()->setDirection(0,0,-1);
}

void OrthoView::defineFootstepGoal()
{
    //ROS_ERROR("vector pressed in map");
//    ((rviz::RenderPanelCustom*)render_panel_)->setEventFilters(rviz::RenderPanelCustom::MOUSE_PRESS_EVENT,false,Qt::NoModifier,Qt::RightButton);
//    ((rviz::RenderPanelCustom*)render_panel_)->setEventFilters(rviz::RenderPanelCustom::MOUSE_RELEASE_EVENT,false,Qt::NoModifier,Qt::RightButton);
//    ((rviz::RenderPanelCustom*)render_panel_)->setEventFilters(rviz::RenderPanelCustom::MOUSE_MOVE_EVENT,false,Qt::NoModifier,Qt::RightButton);
    previous_tool_ = manager_->getToolManager()->getCurrentTool();
    manager_->getToolManager()->setCurrentTool( set_goal_tool_ );
    setting_pose_ = true;
}


void OrthoView::processGoalPose(const geometry_msgs::PoseStamped::ConstPtr &pose)
{
    Base3DView::processGoalPose( pose );
    //ROS_ERROR("goal processed in map");
//    ((rviz::RenderPanelCustom*)render_panel_)->setEventFilters(rviz::RenderPanelCustom::MOUSE_PRESS_EVENT,false,Qt::NoModifier,Qt::LeftButton | Qt::RightButton);
//    ((rviz::RenderPanelCustom*)render_panel_)->setEventFilters(rviz::RenderPanelCustom::MOUSE_RELEASE_EVENT,false,Qt::NoModifier,Qt::LeftButton | Qt::RightButton);
//    ((rviz::RenderPanelCustom*)render_panel_)->setEventFilters(rviz::RenderPanelCustom::MOUSE_MOVE_EVENT,false,Qt::NoModifier,Qt::LeftButton | Qt::RightButton);
    manager_->getToolManager()->setCurrentTool( previous_tool_ );
    setting_pose_ = false;
}

void OrthoView::keyPressEvent( QKeyEvent* event )
{
    if(event->key() == Qt::Key_Escape)
    {
        // block events and change to camera tool
//        ((rviz::RenderPanelCustom*)render_panel_)->setEventFilters(rviz::RenderPanelCustom::MOUSE_PRESS_EVENT,false,Qt::NoModifier,Qt::LeftButton | Qt::RightButton);
//        ((rviz::RenderPanelCustom*)render_panel_)->setEventFilters(rviz::RenderPanelCustom::MOUSE_RELEASE_EVENT,false,Qt::NoModifier,Qt::LeftButton | Qt::RightButton);
//        ((rviz::RenderPanelCustom*)render_panel_)->setEventFilters(rviz::RenderPanelCustom::MOUSE_MOVE_EVENT,false,Qt::NoModifier,Qt::LeftButton | Qt::RightButton);
        manager_->getToolManager()->setCurrentTool( previous_tool_ );
        setting_pose_ = false;
    }
}

rviz::ViewController* OrthoView::getCurrentViewController()
{
    return ortho_view_controller_;
}
}

