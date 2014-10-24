
#include "main_context_menu.h"

MainViewContextMenu::MainViewContextMenu()
{
    createContextMenu();
}

MainViewContextMenu::~BaseContextMenu()
{
}

void MainViewContextMenu::createContextMenu()
{
    //can tell context menu to add a separator when this item is added
    contextMenuItem * separator = new contextMenuItem();
    separator->name = "Separator";

    //create Menu items,
    //the order in which they are created matters
    //must do parent objects before children
    //and in the order you want them to show up in the context menu

    vigir_ocs::Base3DView::makeContextChild("Request Point Cloud",boost::bind(&vigir_ocs::Base3DView::publishPointCloudWorldRequest,(vigir_ocs::Base3DView*)views_list["Top Left"]), NULL);

    contextMenuElements.push_back(separator);

    //manage windows-------------
    contextMenuItem * windowVisibility = vigir_ocs::Base3DView::makeContextParent("Window Visibility");

    joystickContext =  vigir_ocs::Base3DView::makeContextChild("Joystick",boost::bind(&MainViewWidget::contextToggleWindow,this, WINDOW_JOYSTICK), windowVisibility);

    graspContext = vigir_ocs::Base3DView::makeContextChild("Grasp",boost::bind(&MainViewWidget::graspWidgetToggle,this), windowVisibility , contextMenuElements);

    positionContext = vigir_ocs::Base3DView::makeContextChild("Position Mode",boost::bind(&MainViewWidget::contextToggleWindow,this,WINDOW_POSITION_MODE), windowVisibility, contextMenuElements);

    //elements from joint control toolbar
    jointControlContext = vigir_ocs::Base3DView::makeContextChild("Joint Control",boost::bind(&MainViewWidget::contextToggleWindow,this,WINDOW_JOINT_CONTROL), windowVisibility, contextMenuElements);
    pelvisContext = vigir_ocs::Base3DView::makeContextChild("Pelvis Pose",boost::bind(&MainViewWidget::contextToggleWindow,this,WINDOW_BDI_PELVIS_POSE), windowVisibility, contextMenuElements);
    ghostContext = vigir_ocs::Base3DView::makeContextChild("Ghost Control",boost::bind(&MainViewWidget::contextToggleWindow,this,WINDOW_GHOST_CONFIG), windowVisibility, contextMenuElements);
    plannerContext = vigir_ocs::Base3DView::makeContextChild("Planner Configuration",boost::bind(&MainViewWidget::contextToggleWindow,this,WINDOW_PLANNER_CONFIG), windowVisibility, contextMenuElements);

    //elements from footstep control toolbar
    footBasicContext = vigir_ocs::Base3DView::makeContextChild("Basic Footstep Interface",boost::bind(&MainViewWidget::contextToggleWindow,this,WINDOW_FOOTSTEP_BASIC), windowVisibility, contextMenuElements);
    footAdvancedContext = vigir_ocs::Base3DView::makeContextChild("Advanced Footstep Interface",boost::bind(&MainViewWidget::contextToggleWindow,this,WINDOW_FOOTSTEP_ADVANCED), windowVisibility, contextMenuElements);
    footParameterContext = vigir_ocs::Base3DView::makeContextChild("Footstep Parameter Control",boost::bind(&MainViewWidget::contextToggleWindow,this,WINDOW_FOOTSTEP_PARAMETER), windowVisibility, contextMenuElements);

    //------------------------
    contextMenuElements.push_back(separator);

    contextMenuItem * manipulationModes = vigir_ocs::Base3DView::makeContextParent("Manipulation Mode", contextMenuElements);

    objectContext = vigir_ocs::Base3DView::makeContextChild("Object",boost::bind(&MainViewWidget::setObjectManipulationMode,this), manipulationModes, contextMenuElements);
    worldContext = vigir_ocs::Base3DView::makeContextChild("World",boost::bind(&MainViewWidget::setWorldMode,this), manipulationModes, contextMenuElements);
    cameraContext = vigir_ocs::Base3DView::makeContextChild("Camera",boost::bind(&MainViewWidget::setCameraMode,this), manipulationModes, contextMenuElements);

    contextMenuItem * objectModes = vigir_ocs::Base3DView::makeContextParent("Object Mode", contextMenuElements);

    vigir_ocs::Base3DView::makeContextChild("Template",boost::bind(&MainViewWidget::setTemplateMode,this), objectModes, contextMenuElements);
    vigir_ocs::Base3DView::makeContextChild("Left Arm",boost::bind(&MainViewWidget::setLeftArmMode,this), objectModes, contextMenuElements);
    vigir_ocs::Base3DView::makeContextChild("Right Arm",boost::bind(&MainViewWidget::setRightArmMode,this), objectModes, contextMenuElements);

    contextMenuElements.push_back(separator);

    contextMenuItem * systemCommands = vigir_ocs::Base3DView::makeContextParent("System Commands", contextMenuElements);

    vigir_ocs::Base3DView::makeContextChild("Reset World Model",boost::bind(&MainViewWidget::systemCommandContext,this, "reset"), systemCommands, contextMenuElements);
    vigir_ocs::Base3DView::makeContextChild("Save Octomap",boost::bind(&MainViewWidget::systemCommandContext,this,"save_octomap"), systemCommands, contextMenuElements);
    vigir_ocs::Base3DView::makeContextChild("Save Pointcloud",boost::bind(&MainViewWidget::systemCommandContext,this,"save_pointcloud"), systemCommands, contextMenuElements);

    //add all context menu items to each view (base has its own vector of context items, this adds to that vector)
    ((vigir_ocs::Base3DView*) views_list["Top Left"])->addToBaseContextMenu(context_menu_items_);
    ((vigir_ocs::Base3DView*) views_list["Top Right"])->addToBaseContextMenu(context_menu_items_);
    ((vigir_ocs::Base3DView*) views_list["Bottom Left"])->addToBaseContextMenu(context_menu_items_context_menu_items_);
    ((vigir_ocs::Base3DView*) views_list["Bottom Right"])->addToBaseContextMenu(context_menu_items_);


}

//send context items to base_3d_view's context menu
void MainViewContextMenu::sendContextMenu()
{

}



