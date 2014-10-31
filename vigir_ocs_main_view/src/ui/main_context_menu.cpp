
#include "main_context_menu.h"

namespace vigir_ocs
{

MainViewContextMenu::MainViewContextMenu(MainViewWidget main_view)
{
    main_view_ = main_view;
    createContextMenu();
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
    if(!ui->joystickBtn->isChecked())
        joystickContext->action->setChecked(false);
    else
        joystickContext->action->setChecked(true);

    if(!ui->jointControlBtn->isChecked())
        jointControlContext->action->setChecked(false);
    else
        jointControlContext->action->setChecked(true);

    if(!ui->pelvisControlBtn->isChecked())
        pelvisContext->action->setChecked(false);
    else
        pelvisContext->action->setChecked(true);

    if(!ui->basicStepBtn->isChecked())
        footBasicContext->action->setChecked(false);
    else
        footBasicContext->action->setChecked(true);

    if(!ui->stepBtn->isChecked())
        footAdvancedContext->action->setChecked(false);
    else
        footAdvancedContext->action->setChecked(true);

    if(!ui->footstepParamBtn->isChecked())
        footParameterContext->action->setChecked(false);
    else
        footParameterContext->action->setChecked(true);

    if(!ui->ghostControlBtn->isChecked())
        ghostContext->action->setChecked(false);
    else
        ghostContext->action->setChecked(true);

    if(!ui->positionModeBtn->isChecked())
         positionContext->action->setChecked(false);
    else
         positionContext->action->setChecked(true);

    if(!ui->plannerConfigBtn->isChecked())
       plannerContext->action->setChecked(false);
    else
       plannerContext->action->setChecked(true);

    switch(ui->modeBox->currentIndex())
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


    .addActionItem("Request Point Cloud",boost::bind(&vigir_ocs::Base3DView::publishPointCloudWorldRequest,main_view_->getPrimaryView()), NULL);

    addSeperator();
    /**
    //manage windows-------------
    contextMenuItem * windowVisibility = addMenuItem"Window Visibility");

    joystickContext =  addActionItem("Joystick",boost::bind(&MainViewWidget::contextToggleWindow,this, WINDOW_JOYSTICK));

    graspContext = addActionItem("Grasp",boost::bind(&MainViewWidget::graspWidgetToggle,this), windowVisibility );

    positionContext = addActionItem("Position Mode",boost::bind(&MainViewWidget::contextToggleWindow,this,WINDOW_POSITION_MODE), windowVisibility);

    //elements from joint control toolbar
    jointControlContext = addActionItem("Joint Control",boost::bind(&MainViewWidget::contextToggleWindow,this,WINDOW_JOINT_CONTROL), windowVisibility);
    pelvisContext = addActionItem("Pelvis Pose",boost::bind(&MainViewWidget::contextToggleWindow,this,WINDOW_BDI_PELVIS_POSE), windowVisibility);
    ghostContext = addActionItem("Ghost Control",boost::bind(&MainViewWidget::contextToggleWindow,this,WINDOW_GHOST_CONFIG), windowVisibility);
    plannerContext = addActionItem("Planner Configuration",boost::bind(&MainViewWidget::contextToggleWindow,this,WINDOW_PLANNER_CONFIG), windowVisibility);

    //elements from footstep control toolbar
    footBasicContext = addActionItem("Basic Footstep Interface",boost::bind(&MainViewWidget::contextToggleWindow,this,WINDOW_FOOTSTEP_BASIC), windowVisibility);
    footAdvancedContext = addActionItem("Advanced Footstep Interface",boost::bind(&MainViewWidget::contextToggleWindow,this,WINDOW_FOOTSTEP_ADVANCED), windowVisibility);
    footParameterContext = addActionItem("Footstep Parameter Control",boost::bind(&MainViewWidget::contextToggleWindow,this,WINDOW_FOOTSTEP_PARAMETER), windowVisibility);

    //------------------------
    addSeperator();

    contextMenuItem * manipulationModes = addMenuItem("Manipulation Mode");

    objectContext = vigir_ocs::Base3DView::makeContextChild("Object",boost::bind(&MainViewWidget::setObjectManipulationMode,this), manipulationModes);
    worldContext = vigir_ocs::Base3DView::makeContextChild("World",boost::bind(&MainViewWidget::setWorldMode,this), manipulationModes);
    cameraContext = vigir_ocs::Base3DView::makeContextChild("Camera",boost::bind(&MainViewWidget::setCameraMode,this), manipulationModes);

    contextMenuItem * objectModes = vigir_ocs::Base3DView::makeContextParent("Object Mode", contextMenuElements);

    addActionItem("Template",boost::bind(&MainViewWidget::setTemplateMode,this), objectModes);
    addActionItem("Left Arm",boost::bind(&MainViewWidget::setLeftArmMode,this), objectModes);
    addActionItem("Right Arm",boost::bind(&MainViewWidget::setRightArmMode,this), objectModes);

    addSeperator();

    contextMenuItem * systemCommands = addMenuItem("System Commands");

    addActionItem("Reset World Model",boost::bind(&MainViewWidget::systemCommandContext,this, "reset"), systemCommands);
    addActionItem("Save Octomap",boost::bind(&MainViewWidget::systemCommandContext,this,"save_octomap"), systemCommands);
    addActionItem("Save Pointcloud",boost::bind(&MainViewWidget::systemCommandContext,this,"save_pointcloud"), systemCommands);
    **/

    std::map<std::string,QWidget*> views_list = main_view_->getViewsList();

    //add all context menu items to each view (base has its own vector of context items, this adds to that vector)
    ((vigir_ocs::Base3DView*) views_list["Top Left"])->getBaseContextMenu()->addToContextMenuExternally(context_menu_items_);
    ((vigir_ocs::Base3DView*) views_list["Top Right"])->getBaseContextMenu()->addToContextMenuExternally(context_menu_items_);
    ((vigir_ocs::Base3DView*) views_list["Bottom Left"])->getBaseContextMenu()->addToContextMenuExternally(context_menu_items_);
    ((vigir_ocs::Base3DView*) views_list["Bottom Right"])->getBaseContextMenu()->addToContextMenuExternally(context_menu_items_);

}


//CALLBACKS/////////////////////

//void MainViewContextMenu::publishPointCloudWorldRequest()

}

