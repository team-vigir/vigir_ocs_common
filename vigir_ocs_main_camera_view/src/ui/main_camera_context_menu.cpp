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
#include "camera_view_widget.h"
#include "main_camera_context_menu.h"


MainCameraContextMenu::MainCameraContextMenu(MainCameraViewWidget* main_camera_view)
{
    main_camera_view_ = main_camera_view;
    sys_command_pub_ = nh_.advertise<std_msgs::String>("/syscommand",1,false);
    createContextMenu();
}

MainCameraContextMenu::~MainCameraContextMenu()
{
}

void MainCameraContextMenu::createContextMenu()
{
    //grab 4 views
    std::map<std::string,QWidget*> views_list = main_camera_view_->getViewsList();
    vigir_ocs::Base3DView* base_view =  ((vigir_ocs::Base3DView*) ((CameraViewWidget*)views_list["Top Left"])->getCameraView());
    ContextMenuManager * context_menu_manager = base_view->getContextMenuManager();

    context_menu_manager->addActionItem("Request Point Cloud",boost::bind(&vigir_ocs::Base3DView::publishPointCloudWorldRequest,
                                                                                    base_view), NULL);
    context_menu_manager->addSeparatorItem();

    contextMenuItem * systemCommands = context_menu_manager->addMenuItem("System Commands");

    context_menu_manager->addActionItem("Reset World Model",boost::bind(&MainCameraContextMenu::systemCommandContext,this, "reset"), systemCommands);
    context_menu_manager->addActionItem("Reset Pose Est",boost::bind(&MainCameraContextMenu::systemCommandContext,this, "reset_pose_estimation"), systemCommands);
    context_menu_manager->addActionItem("Save Octomap",boost::bind(&MainCameraContextMenu::systemCommandContext,this,"save_octomap"), systemCommands);
    context_menu_manager->addActionItem("Save Pointcloud",boost::bind(&MainCameraContextMenu::systemCommandContext,this,"save_pointcloud"), systemCommands);
    context_menu_manager->addActionItem("Save Image Head",boost::bind(&MainCameraContextMenu::systemCommandContext,this,"save_image_left_eye"), systemCommands);
    context_menu_manager->addActionItem("Save Left Hand Image",boost::bind(&MainCameraContextMenu::systemCommandContext,this,"save_image_left_hand"), systemCommands);
    context_menu_manager->addActionItem("Save Right Hand Image",boost::bind(&MainCameraContextMenu::systemCommandContext,this,"save_image_right_hand"), systemCommands);
}

//CALLBACKS/////////////////////

void MainCameraContextMenu::systemCommandContext(std::string command)
{
    sysCmdMsg.data = command;
    sys_command_pub_.publish(sysCmdMsg);
}



