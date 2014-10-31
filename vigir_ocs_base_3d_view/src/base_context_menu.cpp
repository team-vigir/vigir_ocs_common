
#include "base_context_menu.h"
namespace vigir_ocs
{

BaseContextMenu::BaseContextMenu(vigir_ocs::Base3DView* base_3d_view, FootstepVisManager* footstep_vis_manager)
{
    base_3d_view_ = base_3d_view;
    footstep_vis_manager_ = footstep_vis_manager;
    initializing_context_menu_ = 0;
    //init vector with elements from base3dview
    createContextMenuItems();
}

BaseContextMenu::~BaseContextMenu()
{
}




void BaseContextMenu::addToContextMenuExternally(std::vector<contextMenuItem* > external_context_menu)
{
    for(int i=0;i<external_context_menu.size();i++)
    {
        context_menu_items_.push_back(external_context_menu[i]);
    }
}

void BaseContextMenu::resetMenu()
{
    context_menu_->clear();
    context_menu_->setTitle("Base Menu");
    context_menu_->setStyleSheet("font-size:11px;");
}

//called from main view
void BaseContextMenu::setTemplateTree(QTreeWidget * root)
{
    if(root != NULL)
    {
        template_root_ = root;
        addTemplatesToContext();
    }
}

void BaseContextMenu::addTemplatesToContext()
{
    QTreeWidgetItemIterator it(template_root_);
    while (*it)
    {
        contextMenuItem * localParent;
        if(!(*it)->text(0).contains(".mesh")) //only templates have .mesh
        {
            //will be traversed before children, can be used locally for children as well
            //next parent won't be called until after all children, preorder traversal
            //need to make manually as we have a parent with a parent(not handled by built-in method)
            localParent = new contextMenuItem();
            localParent->hasChildren = true;
            localParent->name = (*it)->text(0);
            localParent->parent = insertTemplateMenu;
            context_menu_items_.push_back(localParent);
        }
        else
        {
            //build path to template for insertion
            QString path = localParent->name + "/" + (*it)->text(0);
            // ROS_ERROR("path %s",qPrintable(path));
            addActionItem((*it)->text(0), boost::bind(&BaseContextMenu::contextInsertTemplate,this,path),localParent);
        }
        ++it;
    }
}

void BaseContextMenu::createContextMenuItems()
{
    //creates all menu items for context menu in a base view, visibility is set on create

    // selection items
    selectTemplateMenu = addActionItem("Select Template",boost::bind(&Base3DView::selectTemplate,base_3d_view_),NULL);

    /**
    leftArmMenu = addActionItem("Select Left Arm",boost::bind(&BaseContextMenu::selectLeftArm,this),NULL);
    rightArmMenu = addActionItem("Select Right Arm",boost::bind(&BaseContextMenu::selectRightArm,this),NULL);

    selectFootstepGoalMenu = addActionItem("Select Footstep",boost::bind(&BaseContextMenu::selectFootstepGoal,this),NULL);
    selectFootstepMenu = addActionItem("Select Footstep",boost::bind(&BaseContextMenu::selectFootstep,this),NULL);
    lockFootstepMenu = addActionItem("Lock Footstep",boost::bind(&BaseContextMenu::lockFootstep,this),NULL);
    unlockFootstepMenu = addActionItem("Unlock Footstep",boost::bind(&BaseContextMenu::unlockFootstep,this),NULL);
    removeFootstepMenu = addActionItem("Remove Footstep",boost::bind(&BaseContextMenu::removeFootstep,this),NULL);
    selectStartFootstepMenu = addActionItem("Set Starting Footstep",boost::bind(&BaseContextMenu::setStartingFootstep,this),NULL);
    clearStartFootstepMenu = addActionItem("Clear Starting Footstep",boost::bind(&BaseContextMenu::clearStartingFootstep,this),NULL);
    stitchFootstepMenu = addActionItem("Stitch Plans",boost::bind(&BaseContextMenu::stitchFootstepPlans,this),NULL);

    addSeperator();

    snapHandMenu = addActionItem("Snap Hand to Ghost",boost::bind(&BaseContextMenu::snapHandGhost,this),NULL);

    addToContextVector(separator);

    footstepGoalMenu = addActionItem("Create Step Plan Goal",boost::bind(&vigir_ocs::BaseContextMenu::defineFootstepGoal,this), NULL);
    defaultFootstepRequestMenu = addActionItem("Request Step Plan",boost::bind(&vigir_ocs::BaseContextMenu::requestFootstepPlan,this,flor_ocs_msgs::OCSFootstepPlanRequest::NEW_PLAN), NULL);
    customFootstepRequestMenu = addActionItem("Request Step Plan...",boost::bind(&vigir_ocs::BaseContextMenu::requestFootstepPlan,this,flor_ocs_msgs::OCSFootstepPlanRequest::NEW_PLAN), NULL);
    executeFootstepPlanMenu = addActionItem(QString("Execute Step Plan"),boost::bind(&BaseContextMenu::executeFootstepPlanContextMenu,this),NULL);
    undoFootstepMenu = addActionItem("Undo Step Change",boost::bind(&FootstepVisManager::requestFootstepListUndo,footstep_vis_manager_),NULL);
    redoFootstepMenu = addActionItem("Redo Step Change",boost::bind(&FootstepVisManager::requestFootstepListRedo,footstep_vis_manager_),NULL);

    addSeperator();

    insertTemplateMenu = addMenuItem("Insert Template");
    removeTemplateMenu = addActionItem("Remove Template",boost::bind(&BaseContextMenu::removeTemplateContextMenu,this),NULL);

    addSeperator();

    lockLeftMenu = addActionItem("Lock Left Arm to Template",boost::bind(&BaseContextMenu::setTemplateGraspLock,this,flor_ocs_msgs::OCSObjectSelection::LEFT_ARM),NULL);
    lockRightMenu = addActionItem("Lock Right Arm to Template",boost::bind(&BaseContextMenu::setTemplateGraspLock,this,flor_ocs_msgs::OCSObjectSelection::RIGHT_ARM),NULL);
    unlockArmsMenu = addActionItem("Unlock Arms",boost::bind(&BaseContextMenu::setTemplateGraspLock,this,-1),NULL);

    addSeperator();

    cartesianMotionMenu = addMenuItem("Cartesian Motion");

    createCartesianMarkerMenu = addActionItem("Create Cartesian Motion Marker",boost::bind(&BaseContextMenu::createCartesianContextMenu,this),cartesianMotionMenu);
    removeCartesianMarkerMenu = addActionItem("Remove All Markers",boost::bind(&BaseContextMenu::removeCartesianContextMenu,this),cartesianMotionMenu);

    circularMotionMenu = addMenuItem("Circular Motion");

    createCircularMarkerMenu = addActionItem("Create Circular Motion Marker",boost::bind(&BaseContextMenu::createCircularContextMenu,this),circularMotionMenu);
    removeCircularMarkerMenu = addActionItem("Remove marker",boost::bind(&BaseContextMenu::removeCircularContextMenu,this),circularMotionMenu);

    addSeperator();
    **/
}





//CALLBACKS///////

//void BaseContextMenu::selectTemplate()
//{
//    base_3d_view_->selectTemplate();
//}

void BaseContextMenu::contextInsertTemplate(QString path)
{
    base_3d_view_->insertTemplate(path);
}

}


