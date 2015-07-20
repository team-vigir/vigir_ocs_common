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
#include "jointList.h"
#include <QVBoxLayout>
#include <QRegExp>
#include <ros/ros.h>
#include <urdf/model.h>
#include <QDebug>

jointList::jointList(QWidget *parent) :
   QWidget(parent)
{
   groups = findValidGroups(RobotStateManager::Instance()->getRobotStateSingleton()->getGroups());

   this->setWindowTitle("Joint_Lists");
   this->setMinimumSize(425,120);
   jointTable = new QTreeWidget();
   jointTable->setColumnCount(4);
   QVBoxLayout* main_layout = new QVBoxLayout;

   main_layout->addWidget(jointTable);
   std::cout << "Adding layout..." << std::endl;
   setLayout(main_layout);

   QStringList columns;
   columns.push_back("Joint");
   columns.push_back("Position");
   columns.push_back("Velocity");
   columns.push_back("Effort");

   jointTable->setHeaderLabels(columns);
   jointTable->setColumnWidth(0,150);

   setUpTable();  

   key_event_sub_ = nh_.subscribe( "/flor/ocs/key_event", 5, &jointList::processNewKeyEvent, this );

}
void jointList::setUpTable()
{
    for(int i = 0; i < groups.size(); i++)
    {
        QTreeWidgetItem* tree = new QTreeWidgetItem(jointTable);
        tree->setText(0, groups[i].name_.c_str());

        for(int m = 0; m < groups[i].joints_.size(); m++)
        {
            QTreeWidgetItem *joint = new QTreeWidgetItem(tree);
            joint->setText(0, groups[i].joints_[m].c_str());
            joints_.push_back(joint);
        }
    }
}

//The groups recieved by the srdf (atleast in atlas) have overlapping groups.  This function picks out the necessary exclusive groups.
//Joint categories in the table are based on groups. ex l_arm_group contains all joints for left arm.
std::vector<srdf::Model::Group> jointList::findValidGroups(std::vector<srdf::Model::Group> groups)
{
    //ex.  group 1 contains joints[A B C]   group 2 contains joints [D E]  group 3 contains joints [A B C D E]
    // only want to return group 1 and group 2 to build jointlist table.

    //stores "bad" groups which contain the joints of multiple groups
    std::map<int, int> badIndexes;

    for(int i = 0; i < groups.size(); i++)
    {
        if(badIndexes.count(i) == 1)//Check if this is a bad group        
            continue;        
        for(int j = i + 1; j < groups.size(); j++)//Compare every group with every other group
        {
            //Note: compilers can be confused by continues and breaks nearby?
            if(badIndexes.count(j) == 1)//Check if this is a bad group
                continue;

            if(badIndexes.count(i) == 1)//this group could be marked bad inside
                break;

            int larger;
            int smaller;
            if(groups[i].joints_.size() > groups[j].joints_.size())//Pick a smaller or larger vector
            {
                larger = i;
                smaller = j;
            }
            else
            {                
                larger = j;
                smaller = i;
            }

            //Compare the two subsets
            bool isSubset = true;
            for(int m = 0; m < groups[smaller].joints_.size(); m++)
            {
                bool hasM = false;
                for(int n = 0; n < groups[larger].joints_.size(); n++)
                {
                    if(groups[smaller].joints_[m].compare(groups[larger].joints_[n]) == 0)
                    {
                        hasM = true;
                        break;
                    }
                }                
                if(!hasM)
                {
                    isSubset = false;
                    break;
                }
            }
            //Check if either list is empty
            if(groups[smaller].joints_.empty())
            {
                badIndexes[smaller] = smaller;
            }
            else if(groups[larger].joints_.empty())
            {
                badIndexes[larger] = larger;
            }
            else if(isSubset)
            {
                badIndexes[larger] = larger;                
            }
        }
    }

    //Create the new group vector with the badIndexes excluded
    std::vector<srdf::Model::Group> updatedGroups;
    for(int i = 0; i < groups.size(); i++)
    {
        if(badIndexes.count(i) == 0)
        {
            updatedGroups.push_back(groups[i]);
        }
    }

    return updatedGroups;
}


jointList::~jointList()
{
   //delete ui;
}
int jointList::getNumError()
{
   return errorCount;
}
int jointList::getNumWarn()
{
   return warnCount;
}


//?
void jointList::processNewKeyEvent(const vigir_ocs_msgs::OCSKeyEvent::ConstPtr key_event)
{
   // store key state
   if(key_event->state)
       keys_pressed_list_.push_back(key_event->keycode);
   else
       keys_pressed_list_.erase(std::remove(keys_pressed_list_.begin(), keys_pressed_list_.end(), key_event->keycode), keys_pressed_list_.end());

   // process hotkeys
   std::vector<int>::iterator key_is_pressed;

   key_is_pressed = std::find(keys_pressed_list_.begin(), keys_pressed_list_.end(), 37);
   /*if(key_event->keycode == 17 && key_event->state && key_is_pressed != keys_pressed_list_.end()) // ctrl+8
   {
       if(this->isVisible())
       {
           this->hide();
       }
       else
       {
           //this->move(QPoint(key_event->cursor_x+5, key_event->cursor_y+5));
           this->show();
       }
   }*/
}
