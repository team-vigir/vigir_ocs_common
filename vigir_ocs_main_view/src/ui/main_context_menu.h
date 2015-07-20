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

#ifndef MAIN_CONTEXT_MENU_H
#define MAIN_CONTEXT_MENU_H

#include <QObject>
#include <ros/ros.h>
#include <string>
#include <boost/bind.hpp>
#include <map>
#include <stdlib.h>
#include <QTreeWidget>
#include <std_msgs/String.h>
#include "context_menu_manager.h"
#include "ui/ghost_control_widget.h"

namespace ui
{
    class MainViewWidget;
}
class MainViewWidget;


class MainViewContextMenu : public QObject
{
    friend class MainViewWidget;

    Q_OBJECT

public:
    MainViewContextMenu(MainViewWidget* main_view);
    virtual ~MainViewContextMenu();
    //contextMenuItem* getContextMenuItem();
    void setItemCheckState(std::string name, bool checkable);
    //void setAllCheckable();

public Q_SLOTS:
    //void updateContextMenu();

private:   
    MainViewWidget* main_view_;
    void createContextMenu();

    ros::NodeHandle n_;
    std_msgs::String sysCmdMsg;
    ros::Publisher sys_command_pub_;

    vigir_ocs::Base3DView* base_view_;
    ContextMenuManager* context_menu_manager_;
    GhostControlWidget * ghost_control_widget_;

    std::map<std::string, contextMenuItem *> context_menu_items_map_;

   //CALLBACKS///////////////////////////////////
    void systemCommandContext(std::string command);
    void contextToggleWindow(int window);
    void setTemplateMode();
    void setLeftArmMode();
    void setRightArmMode();
    void setCameraMode();
    void setWorldMode();
   //END CALLBACKS///////////////////////////

};

#endif //MAIN_CONTEXT_MENU_H
