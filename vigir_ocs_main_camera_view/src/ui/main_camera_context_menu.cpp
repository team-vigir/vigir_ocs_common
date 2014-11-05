
#include "main_camera_context_menu.h"


MainCameraContextMenu::MainCameraContextMenu(MainCameraViewWidget main_camera_view)
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

    ContextMenuManager::Instance()->addActionItem("Request Point Cloud",boost::bind(&vigir_ocs::Base3DView::publishPointCloudWorldRequest,((vigir_ocs::Base3DView*) ((CameraViewWidget*)views_list_["Top Left"])->getCameraView())), NULL);

    ContextMenuManager::Instance()->addSeperator();

    contextMenuItem * systemCommands = ContextMenuManager::Instance()->addMenuItem("System Commands");

    ContextMenuManager::Instance()->addActionItem("Reset World Model",boost::bind(&MainCameraContextMenu::systemCommandContext,this, "reset"), systemCommands);
    ContextMenuManager::Instance()->addActionItem("Save Octomap",boost::bind(&MainCameraContextMenu::systemCommandContext,this,"save_octomap"), systemCommands);
    ContextMenuManager::Instance()->addActionItem("Save Pointcloud",boost::bind(&MainCameraContextMenu::systemCommandContext,this,"save_pointcloud"), systemCommands);
    ContextMenuManager::Instance()->addActionItem("Save Image Head",boost::bind(&MainCameraContextMenu::systemCommandContext,this,"save_image_left_eye"), systemCommands);
    ContextMenuManager::Instance()->addActionItem("Save Left Hand Image",boost::bind(&MainCameraContextMenu::systemCommandContext,this,"save_image_left_hand"), systemCommands);
    ContextMenuManager::Instance()->addActionItem("Save Right Hand Image",boost::bind(&MainCameraContextMenu::systemCommandContext,this,"save_image_right_hand"), systemCommands);
}

//CALLBACKS/////////////////////

void MainCameraContextMenu::systemCommandContext(std::string command)
{
    sysCmdMsg.data = command;
    sys_command_pub_.publish(sysCmdMsg);
}



