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



