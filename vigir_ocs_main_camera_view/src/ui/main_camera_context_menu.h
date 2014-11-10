#ifndef MAIN_CAMERA_CONTEXT_MENU_H
#define MAIN_CAMERA_CONTEXT_MENU_H

#include <QObject>

#include <ros/ros.h>

#include <string>
#include <boost/bind.hpp>
#include <stdlib.h>
#include <QTreeWidget>
#include "context_menu_manager.h"
#include "main_camera_view_widget.h"


class MainCameraContextMenu: public QObject
{
    Q_OBJECT
public:
    MainCameraContextMenu(MainCameraViewWidget* main_camera_view);
    virtual ~MainCameraContextMenu();

private:
    MainCameraViewWidget* main_camera_view_;
    void createContextMenu();

    //for system cmd callback
    ros::Publisher sys_command_pub_;
    ros::NodeHandle nh_;
    std_msgs::String sysCmdMsg;

   //CALLBACKS///////////////////////////////////
    void systemCommandContext(std::string command);


   //END CALLBACKS///////////////////////////

};


#endif //MAIN_CAMERA_CONTEXT_MENU_H
