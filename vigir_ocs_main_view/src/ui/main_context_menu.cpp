#include "main_context_menu.h"
#include "main_view_widget.h"

MainViewContextMenu::MainViewContextMenu(MainViewWidget *main_view)
{
    main_view_ = main_view;
    base_view_ = main_view_->getPrimaryView();
    context_menu_manager_ = base_view_->getContextMenuManager();    

    //connect context menu elements to be updated by ui
    connect(context_menu_manager_,SIGNAL(updateMainViewItems()),main_view_,SLOT(updateContextMenu()));
    createContextMenu();
    sys_command_pub_ = n_.advertise<std_msgs::String>("/syscommand",1,false);

}

MainViewContextMenu::~MainViewContextMenu()
{
}

void MainViewContextMenu::setItemCheckState(std::string name, bool checkable)
{    
    //contains?
    if(context_menu_items_map_.find(name) != context_menu_items_map_.end())
    {
        //must enable the action to be checkable
        context_menu_items_map_[name]->action->setCheckable(checkable);
        //can still set checked even if not checkable, no result though.
        context_menu_items_map_[name]->action->setChecked(checkable);
    }
    else
        ROS_ERROR("Context item in main view not found: %s\n", name.c_str());
}

void MainViewContextMenu::createContextMenu()
{
    //create Menu items,
    //the order in which they are created matters
    //must do parent objects before children
    //and in the order you want them to show up in the context menu

    //need to build a map as we add certain items becuase main view references the ui to update isChecked property

    context_menu_manager_->addActionItem("Request Point Cloud",boost::bind(&vigir_ocs::Base3DView::publishPointCloudWorldRequest,base_view_), NULL);

    context_menu_manager_->addSeparatorItem();

    //manage windows-------------
    contextMenuItem * windowVisibility = context_menu_manager_->addMenuItem("Window Visibility");

    context_menu_items_map_["Joystick"] = context_menu_manager_->addActionItem("Joystick",boost::bind(&MainViewWidget::contextToggleWindow,main_view_, WINDOW_JOYSTICK),windowVisibility);

    context_menu_items_map_["Grasp"]= context_menu_manager_->addActionItem("Grasp",boost::bind(&MainViewWidget::graspWidgetToggle,main_view_), windowVisibility );

    context_menu_items_map_["Position Mode"]= context_menu_manager_->addActionItem("Position Mode",boost::bind(&MainViewWidget::contextToggleWindow,main_view_,WINDOW_POSITION_MODE), windowVisibility);

    //elements from joint control toolbar
    context_menu_items_map_["Joint Control"] = context_menu_manager_->addActionItem("Joint Control",boost::bind(&MainViewWidget::contextToggleWindow,main_view_,WINDOW_JOINT_CONTROL), windowVisibility);
    context_menu_items_map_["Pelvis Pose"] = context_menu_manager_->addActionItem("Pelvis Pose",boost::bind(&MainViewWidget::contextToggleWindow,main_view_,WINDOW_BDI_PELVIS_POSE), windowVisibility);
    context_menu_items_map_["Ghost Control"]= context_menu_manager_->addActionItem("Ghost Control",boost::bind(&MainViewWidget::contextToggleWindow,main_view_,WINDOW_GHOST_CONFIG), windowVisibility);
    context_menu_items_map_["Planner Configuration"]= context_menu_manager_->addActionItem("Planner Configuration",boost::bind(&MainViewWidget::contextToggleWindow,main_view_,WINDOW_PLANNER_CONFIG), windowVisibility);

    //elements from footstep control toolbar
    context_menu_items_map_["Basic Footstep Interface"] = context_menu_manager_->addActionItem("Basic Footstep Interface",boost::bind(&MainViewWidget::contextToggleWindow,main_view_,WINDOW_FOOTSTEP_BASIC), windowVisibility);
    context_menu_items_map_["Advanced Footstep Interface"] = context_menu_manager_->addActionItem("Advanced Footstep Interface",boost::bind(&MainViewWidget::contextToggleWindow,main_view_,WINDOW_FOOTSTEP_ADVANCED), windowVisibility);
    context_menu_items_map_["Footstep Parameter Control"] = context_menu_manager_->addActionItem("Footstep Parameter Control",boost::bind(&MainViewWidget::contextToggleWindow,main_view_,WINDOW_FOOTSTEP_PARAMETER), windowVisibility);

    //Manipulation modes
    context_menu_manager_->addSeparatorItem();

    contextMenuItem* manipulationModes = context_menu_manager_->addMenuItem("Manipulation Mode");

    context_menu_items_map_["Object"] = context_menu_manager_->addActionItem("Object",boost::bind(&MainViewWidget::setObjectManipulationMode,main_view_), manipulationModes);
    context_menu_items_map_["World"] = context_menu_manager_->addActionItem("World",boost::bind(&MainViewWidget::setWorldMode,main_view_), manipulationModes);
    context_menu_items_map_["Camera"]= context_menu_manager_->addActionItem("Camera",boost::bind(&MainViewWidget::setCameraMode,main_view_), manipulationModes);

    contextMenuItem * objectModes = context_menu_manager_->addMenuItem("Object Mode");

    context_menu_manager_->addActionItem("Template",boost::bind(&MainViewContextMenu::setTemplateMode,this), objectModes);
    context_menu_manager_->addActionItem("Left Arm",boost::bind(&MainViewContextMenu::setLeftArmMode,this), objectModes);
    context_menu_manager_->addActionItem("Right Arm",boost::bind(&MainViewContextMenu::setRightArmMode,this), objectModes);

    context_menu_manager_->addSeparatorItem();

    //System commands
    contextMenuItem * systemCommands = context_menu_manager_->addMenuItem("System Commands");

    context_menu_manager_->addActionItem("Reset World Model",boost::bind(&MainViewContextMenu::systemCommandContext,this, "reset"), systemCommands);
    context_menu_manager_->addActionItem("Reset Pose Est",boost::bind(&MainViewContextMenu::systemCommandContext,this, "reset_pose_estimation"), systemCommands);
    context_menu_manager_->addActionItem("Save Octomap",boost::bind(&MainViewContextMenu::systemCommandContext,this,"save_octomap"), systemCommands);
    context_menu_manager_->addActionItem("Save Pointcloud",boost::bind(&MainViewContextMenu::systemCommandContext,this,"save_pointcloud"), systemCommands);

    //ghost widget functions
    context_menu_manager_->addSeparatorItem();

    context_menu_manager_->addActionItem("Snap Ghost to Robot",boost::bind(&MainViewWidget::snapGhostContextMenu,main_view_), NULL);
    context_menu_items_map_["Use Torso"] = context_menu_manager_->addActionItem("Use Torso",boost::bind(&MainViewWidget::useTorsoContextMenu,main_view_), NULL);
}
//CALLBACKS/////////////////////

void MainViewContextMenu::systemCommandContext(std::string command)
{
    sysCmdMsg.data = command;
    sys_command_pub_.publish(sysCmdMsg);
}
void MainViewContextMenu::setTemplateMode()
{
    main_view_->setObjectMode(0);
}
void MainViewContextMenu::setLeftArmMode()
{
    main_view_->setObjectMode(1);
}
void MainViewContextMenu::setRightArmMode()
{
    main_view_->setObjectMode(2);
}
//END CALLBACKS////////////////////////////



