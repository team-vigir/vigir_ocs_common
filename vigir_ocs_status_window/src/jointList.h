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
