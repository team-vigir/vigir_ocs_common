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
//@TODO_ADD_AUTHOR_INFO
#ifndef JOINTLIST_H
#define JOINTLIST_H

#include <QMainWindow>
#include <QWidget>
#include <QTreeWidget>
#include <QFile>
#include <QtGui>

#include <vector>
#include <algorithm>

#include <ros/ros.h>
#include <ros/subscriber.h>

#include <sensor_msgs/JointState.h>

#include <vigir_ocs_msgs/OCSKeyEvent.h>

#include "robot_state_manager.h"


namespace rviz
{
class Display;
class RenderPanel;
class VisualizationManager;
class FrameManager;
}

class jointList : public QWidget
{
   Q_OBJECT

public:
   explicit jointList(QWidget *parent = 0);
   ~jointList();   
   int getNumWarn();
   int getNumError();
   void processNewKeyEvent(const vigir_ocs_msgs::OCSKeyEvent::ConstPtr pose);


private:
   ros::Subscriber joint_states;   
   std::vector<srdf::Model::Group> groups;
   std::vector<QTreeWidgetItem*> joints_;
   QTreeWidgetItem* tree;         
   QTreeWidget* jointTable;
   int warnCount;
   int errorCount;

   std::vector<int> keys_pressed_list_;

   ros::NodeHandle nh_;

   ros::Subscriber key_event_sub_;

   void setUpTable();

   /**
     * The groups recieved by the srdf have overlapping groups.  This function picks out the necessary groups.
     */
   std::vector<srdf::Model::Group> findValidGroups(std::vector<srdf::Model::Group> groups);


Q_SIGNALS:
   void sendJointData(int,QString);
};

#endif // JOINTLIST_H
