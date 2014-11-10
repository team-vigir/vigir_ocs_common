
#include "main_context_menu.h"

MainViewContextMenu::MainViewContextMenu(MainViewWidget *main_view)
{
    main_view_ = main_view;
    createContextMenu();

    sys_command_pub_ = n_.advertise<std_msgs::String>("/syscommand",1,false);
}

MainViewContextMenu::~MainViewContextMenu()
{
}

void MainViewContextMenu::updateContextMenu()
{
    //change default checkable values of context items.. must as they are created with menu
    joystickContext->action->setCheckable(true);
    positionContext->action->setCheckable(true);
    graspContext->action->setCheckable(true);
    jointControlContext->action->setCheckable(true);
    pelvisContext->action->setCheckable(true);
    ghostContext->action->setCheckable(true);
    plannerContext->action->setCheckable(true);
    footBasicContext->action->setCheckable(true);
    footAdvancedContext->action->setCheckable(true);
    footParameterContext->action->setCheckable(true);
    objectContext->action->setCheckable(true);
    worldContext->action->setCheckable(true);
    cameraContext->action->setCheckable(true);

    //update context menu elements with checks
    if(!main_view_->getUi()->joystickBtn->isChecked())
        joystickContext->action->setChecked(false);
    else
        joystickContext->action->setChecked(true);

    if(!main_view_->getUi()->jointControlBtn->isChecked())
        jointControlContext->action->setChecked(false);
    else
        jointControlContext->action->setChecked(true);

    if(!main_view_->getUi()->pelvisControlBtn->isChecked())
        pelvisContext->action->setChecked(false);
    else
        pelvisContext->action->setChecked(true);

    if(!main_view_->getUi()->basicStepBtn->isChecked())
        footBasicContext->action->setChecked(false);
    else
        footBasicContext->action->setChecked(true);

    if(!main_view_->getUi()->stepBtn->isChecked())
        footAdvancedContext->action->setChecked(false);
    else
        footAdvancedContext->action->setChecked(true);

    if(!main_view_->getUi()->footstepParamBtn->isChecked())
        footParameterContext->action->setChecked(false);
    else
        footParameterContext->action->setChecked(true);

    if(!main_view_->getUi()->ghostControlBtn->isChecked())
        ghostContext->action->setChecked(false);
    else
        ghostContext->action->setChecked(true);

    if(!main_view_->getUi()->positionModeBtn->isChecked())
         positionContext->action->setChecked(false);
    else
         positionContext->action->setChecked(true);

    if(!main_view_->getUi()->plannerConfigBtn->isChecked())
       plannerContext->action->setChecked(false);
    else
       plannerContext->action->setChecked(true);

    switch(main_view_->getUi()->modeBox->currentIndex())
    {
    case 0:
        objectContext->action->setChecked(true);
        worldContext->action->setChecked(false);
        cameraContext->action->setChecked(false);
        break;
    case 1:
        objectContext->action->setChecked(false);
        worldContext->action->setChecked(true);
        cameraContext->action->setChecked(false);
        break;
    case 2:
        objectContext->action->setChecked(false);
        worldContext->action->setChecked(false);
        cameraContext->action->setChecked(true);
        break;
    }
}

void MainViewContextMenu::createContextMenu()
{
    //create Menu items,
    //the order in which they are created matters
    //must do parent objects before children
    //and in the order you want them to show up in the context menu


    ContextMenuManager::Instance()->addActionItem("Request Point Cloud",boost::bind(&vigir_ocs::Base3DView::publishPointCloudWorldRequest,main_view_->getPrimaryView()), NULL);

    ContextMenuManager::Instance()->addSeperator();

    //manage windows-------------
    contextMenuItem * windowVisibility = ContextMenuManager::Instance()->addMenuItem("Window Visibility");

    joystickContext =  ContextMenuManager::Instance()->addActionItem("Joystick",boost::bind(&MainViewWidget::contextToggleWindow,this, WINDOW_JOYSTICK));

    graspContext = ContextMenuManager::Instance()->addActionItem("Grasp",boost::bind(&MainViewWidget::graspWidgetToggle,this), windowVisibility );

    positionContext = ContextMenuManager::Instance()->addActionItem("Position Mode",boost::bind(&MainViewWidget::contextToggleWindow,this,WINDOW_POSITION_MODE), windowVisibility);

    //elements from joint control toolbar
    jointControlContext = ContextMenuManager::Instance()->addActionItem("Joint Control",boost::bind(&MainViewWidget::contextToggleWindow,this,WINDOW_JOINT_CONTROL), windowVisibility);
    pelvisContext = ContextMenuManager::Instance()->addActionItem("Pelvis Pose",boost::bind(&MainViewWidget::contextToggleWindow,this,WINDOW_BDI_PELVIS_POSE), windowVisibility);
    ghostContext = ContextMenuManager::Instance()->addActionItem("Ghost Control",boost::bind(&MainViewWidget::contextToggleWindow,this,WINDOW_GHOST_CONFIG), windowVisibility);
    plannerContext = ContextMenuManager::Instance()->addActionItem("Planner Configuration",boost::bind(&MainViewWidget::contextToggleWindow,this,WINDOW_PLANNER_CONFIG), windowVisibility);

    //elements from footstep control toolbar
    footBasicContext = ContextMenuManager::Instance()->addActionItem("Basic Footstep Interface",boost::bind(&MainViewWidget::contextToggleWindow,this,WINDOW_FOOTSTEP_BASIC), windowVisibility);
    footAdvancedContext = ContextMenuManager::Instance()->addActionItem("Advanced Footstep Interface",boost::bind(&MainViewWidget::contextToggleWindow,this,WINDOW_FOOTSTEP_ADVANCED), windowVisibility);
    footParameterContext = ContextMenuManager::Instance()->addActionItem("Footstep Parameter Control",boost::bind(&MainViewWidget::contextToggleWindow,this,WINDOW_FOOTSTEP_PARAMETER), windowVisibility);

    //------------------------
    ContextMenuManager::Instance()->addSeperator();

    contextMenuItem * manipulationModes = ContextMenuManager::Instance()->addMenuItem("Manipulation Mode");

    objectContext = ContextMenuManager::Instance()->addActionItem("Object",boost::bind(&MainViewWidget::setObjectManipulationMode,this), manipulationModes);
    worldContext = ContextMenuManager::Instance()->addActionItem("World",boost::bind(&MainViewWidget::setWorldMode,this), manipulationModes);
    cameraContext = ContextMenuManager::Instance()->addActionItem("Camera",boost::bind(&MainViewWidget::setCameraMode,this), manipulationModes);

    contextMenuItem * objectModes = ContextMenuManager::Instance()->addMenuItem("Object Mode");

    ContextMenuManager::Instance()->addActionItem("Template",boost::bind(&MainViewWidget::setTemplateMode,this), objectModes);
    ContextMenuManager::Instance()->addActionItem("Left Arm",boost::bind(&MainViewWidget::setLeftArmMode,this), objectModes);
    ContextMenuManager::Instance()->addActionItem("Right Arm",boost::bind(&MainViewWidget::setRightArmMode,this), objectModes);

    ContextMenuManager::Instance()->addSeperator();

    contextMenuItem * systemCommands = ContextMenuManager::Instance()->addMenuItem("System Commands");

    ContextMenuManager::Instance()->addActionItem("Reset World Model",boost::bind(&MainViewWidget::systemCommandContext,this, "reset"), systemCommands);
    ContextMenuManager::Instance()->addActionItem("Save Octomap",boost::bind(&MainViewWidget::systemCommandContext,this,"save_octomap"), systemCommands);
    ContextMenuManager::Instance()->addActionItem("Save Pointcloud",boost::bind(&MainViewWidget::systemCommandContext,this,"save_pointcloud"), systemCommands);
    //**/

    //std::map<std::string,QWidget*> views_list = main_view_->getViewsList();

//    //add all context menu items to each view (base has its own vector of context items, this adds to that vector)
//    ((vigir_ocs::Base3DView*) views_list["Top Left"])->getBaseContextMenu()->addToContextMenuExternally(context_menu_items_);
//    ((vigir_ocs::Base3DView*) views_list["Top Right"])->getBaseContextMenu()->addToContextMenuExternally(context_menu_items_);
//    ((vigir_ocs::Base3DView*) views_list["Bottom Left"])->getBaseContextMenu()->addToContextMenuExternally(context_menu_items_);
//    ((vigir_ocs::Base3DView*) views_list["Bottom Right"])->getBaseContextMenu()->addToContextMenuExternally(context_menu_items_);

}


//CALLBACKS/////////////////////

void MainViewContextMenu::systemCommandContext(std::string command)
{
    sysCmdMsg.data = command;
    sys_command_pub_.publish(sysCmdMsg);
}

void MainViewContextMenu::contextToggleWindow(int window)
{
    switch(window)
    {
    case WINDOW_JOYSTICK:
        if(!main_view_->getUi()->joystickBtn->isChecked())
            main_view_->getUi()->joystickBtn->setChecked(true);
        else
            main_view_->getUi()->joystickBtn->setChecked(false);
        break;
    case WINDOW_JOINT_CONTROL:
        if(!main_view_->getUi()->jointControlBtn->isChecked())
            main_view_->getUi()->jointControlBtn->setChecked(true);
        else
            main_view_->getUi()->jointControlBtn->setChecked(false);
        break;
    case WINDOW_BDI_PELVIS_POSE:
        if(!main_view_->getUi()->pelvisControlBtn->isChecked())
            main_view_->getUi()->pelvisControlBtn->setChecked(true);
        else
            main_view_->getUi()->pelvisControlBtn->setChecked(false);
        break;
    case WINDOW_FOOTSTEP_BASIC:
        if(!main_view_->getUi()->basicStepBtn->isChecked())
            main_view_->getUi()->basicStepBtn->setChecked(true);
        else
            main_view_->getUi()->basicStepBtn->setChecked(false);
        break;
    case WINDOW_FOOTSTEP_ADVANCED:
        if(!main_view_->getUi()->stepBtn->isChecked())
            main_view_->getUi()->stepBtn->setChecked(true);
        else
            main_view_->getUi()->stepBtn->setChecked(false);
        break;
    case WINDOW_FOOTSTEP_PARAMETER:
        if(!main_view_->getUi()->footstepParamBtn->isChecked())
            main_view_->getUi()->footstepParamBtn->setChecked(true);
        else
            main_view_->getUi()->footstepParamBtn->setChecked(false);
        break;
    case WINDOW_GHOST_CONFIG:
        if(!main_view_->getUi()->ghostControlBtn->isChecked())
            main_view_->getUi()->ghostControlBtn->setChecked(true);
        else
            main_view_->getUi()->ghostControlBtn->setChecked(false);
        break;
    case WINDOW_POSITION_MODE:
        if(!main_view_->getUi()->positionModeBtn->isChecked())
            main_view_->getUi()->positionModeBtn->setChecked(true);
        else
            main_view_->getUi()->positionModeBtn->setChecked(false);
        break;
    case WINDOW_PLANNER_CONFIG:
        if(!main_view_->getUi()->plannerConfigBtn->isChecked())
            main_view_->getUi()->plannerConfigBtn->setChecked(true);
        else
            main_view_->getUi()->plannerConfigBtn->setChecked(false);
        break;
    }
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
void MainViewContextMenu::setCameraMode()
{
    main_view_->getUi()->modeBox->setCurrentIndex(2);
    main_view_->setManipulationMode(2);

}
void MainViewContextMenu::setWorldMode()
{
    main_view_->getUi()->modeBox->setCurrentIndex(1);
    main_view_->setManipulationMode(1);
}



