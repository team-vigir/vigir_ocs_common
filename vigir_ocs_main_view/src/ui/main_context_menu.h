
#ifndef MAIN_CONTEXT_MENU_H
#define MAIN_CONTEXT_MENU_H

#include <QObject>
#include <ros/ros.h>
#include <string>
#include <boost/bind.hpp>
#include <vector>
#include <map>
#include <stdlib.h>
#include <QTreeWidget>
#include "context_menu_manager.h"
#include "main_view_widget.h"


class MainViewContextMenu : public QObject
{
    Q_OBJECT

public:
    MainViewContextMenu(MainViewWidget* main_view);
    virtual ~MainViewContextMenu();

public Q_SLOTS:
    void updateContextMenu();

private:   
    MainViewWidget* main_view_;
    void createContextMenu();

    ros::NodeHandle n_;
    std_msgs::String sysCmdMsg;
    ros::Publisher sys_command_pub_;

    //context item references for checking/unchecking in context menu
    contextMenuItem * joystickContext;
    contextMenuItem * positionContext;
    contextMenuItem * graspContext;
    contextMenuItem * jointControlContext;
    contextMenuItem * pelvisContext;
    contextMenuItem * ghostContext;
    contextMenuItem * plannerContext;
    contextMenuItem * footBasicContext;
    contextMenuItem * footAdvancedContext;
    contextMenuItem * footParameterContext;
    contextMenuItem * objectContext;
    contextMenuItem * worldContext;
    contextMenuItem * cameraContext;

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
