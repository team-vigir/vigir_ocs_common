
#include "base_context_menu.h"
namespace vigir_ocs
{

BaseContextMenu::BaseContextMenu(vigir_ocs::Base3DView* base_3d_view, FootstepVisManager* footstep_vis_manager)
{
    base_3d_view_ = base_3d_view;
    footstep_vis_manager_ = footstep_vis_manager;    
    //init vector with elements from base3dview
    createContextMenuItems();
}

BaseContextMenu::~BaseContextMenu()
{
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
            ContextMenuManager::Instance()->addCustomItem(localParent);
        }
        else
        {
            //build path to template for insertion
            QString path = localParent->name + "/" + (*it)->text(0);
            // ROS_ERROR("path %s",qPrintable(path));
            ContextMenuManager::Instance()->addActionItem((*it)->text(0), boost::bind(&BaseContextMenu::contextInsertTemplate,this,path),localParent);
        }
        ++it;
    }
}

void BaseContextMenu::createContextMenuItems()
{
    //creates all menu items for context menu in a base view, visibility is set on create

    // selection items
    //selectTemplateMenu = addActionItem("Select Template",boost::bind(&Base3DView::selectTemplate,base_3d_view_),NULL);
    //menu_items_.insert(std::make_pair<std::string,contextMenuItem*>("Select Template",ContextMenuManager::Instance()->addActionItem("Select Template",boost::bind(&Base3DView::selectTemplate,base_3d_view_),NULL)));

    ContextMenuManager::Instance()->addActionItem("Select Template",boost::bind(&Base3DView::selectTemplate,base_3d_view_),NULL);
    /**
    ContextMenuManager::Instance()->addActionItem("Select Left Arm",boost::bind(&BaseContextMenu::selectLeftArm,this),NULL);
    ContextMenuManager::Instance()->addActionItem("Select Right Arm",boost::bind(&BaseContextMenu::selectRightArm,this),NULL);

    ContextMenuManager::Instance()->addActionItem("Select Footstep",boost::bind(&BaseContextMenu::selectFootstepGoal,this),NULL);
    ContextMenuManager::Instance()->addActionItem("Select Footstep",boost::bind(&BaseContextMenu::selectFootstep,this),NULL);
    ContextMenuManager::Instance()->addActionItem("Lock Footstep",boost::bind(&BaseContextMenu::lockFootstep,this),NULL);
    ContextMenuManager::Instance()->addActionItem("Unlock Footstep",boost::bind(&BaseContextMenu::unlockFootstep,this),NULL);
    ContextMenuManager::Instance()->addActionItem("Remove Footstep",boost::bind(&BaseContextMenu::removeFootstep,this),NULL);
    ContextMenuManager::Instance()->addActionItem("Set Starting Footstep",boost::bind(&BaseContextMenu::setStartingFootstep,this),NULL);
    ContextMenuManager::Instance()->addActionItem("Clear Starting Footstep",boost::bind(&BaseContextMenu::clearStartingFootstep,this),NULL);
    ContextMenuManager::Instance()->addActionItem("Stitch Plans",boost::bind(&BaseContextMenu::stitchFootstepPlans,this),NULL);

    ContextMenuManager::Instance()->addSeperator();

    ContextMenuManager::Instance()->addActionItem("Snap Hand to Ghost",boost::bind(&BaseContextMenu::snapHandGhost,this),NULL);


    ContextMenuManager::Instance()->addActionItem("Create Step Plan Goal",boost::bind(&vigir_ocs::BaseContextMenu::defineFootstepGoal,this), NULL);
    ContextMenuManager::Instance()->addActionItem("Request Step Plan",boost::bind(&vigir_ocs::BaseContextMenu::requestFootstepPlan,this,flor_ocs_msgs::OCSFootstepPlanRequest::NEW_PLAN), NULL);
    ContextMenuManager::Instance()->addActionItem("Request Step Plan...",boost::bind(&vigir_ocs::BaseContextMenu::requestFootstepPlan,this,flor_ocs_msgs::OCSFootstepPlanRequest::NEW_PLAN), NULL);
    ContextMenuManager::Instance()->addActionItem(QString("Execute Step Plan"),boost::bind(&BaseContextMenu::executeFootstepPlanContextMenu,this),NULL);
    ContextMenuManager::Instance()->addActionItem("Undo Step Change",boost::bind(&FootstepVisManager::requestFootstepListUndo,footstep_vis_manager_),NULL);
    ContextMenuManager::Instance()->addActionItem("Redo Step Change",boost::bind(&FootstepVisManager::requestFootstepListRedo,footstep_vis_manager_),NULL);

    ContextMenuManager::Instance()->addSeperator();

    insertTemplateMenu = addMenuItem("Insert Template");
    removeTemplateMenu = addActionItem("Remove Template",boost::bind(&BaseContextMenu::removeTemplateContextMenu,this),NULL);

    ContextMenuManager::Instance()->addSeperator();

    ContextMenuManager::Instance()->addActionItem("Lock Left Arm to Template",boost::bind(&BaseContextMenu::setTemplateGraspLock,this,flor_ocs_msgs::OCSObjectSelection::LEFT_ARM),NULL);
    ContextMenuManager::Instance()->addActionItem("Lock Right Arm to Template",boost::bind(&BaseContextMenu::setTemplateGraspLock,this,flor_ocs_msgs::OCSObjectSelection::RIGHT_ARM),NULL);
    ContextMenuManager::Instance()->addActionItem("Unlock Arms",boost::bind(&BaseContextMenu::setTemplateGraspLock,this,-1),NULL);

    ContextMenuManager::Instance()->addSeperator();

    cartesianMotionMenu = ContextMenuManager::Instance()->addMenuItem("Cartesian Motion");

    ContextMenuManager::Instance()->addActionItem("Create Cartesian Motion Marker",boost::bind(&BaseContextMenu::createCartesianContextMenu,this),cartesianMotionMenu);
    ContextMenuManager::Instance()->addActionItem("Remove All Markers",boost::bind(&BaseContextMenu::removeCartesianContextMenu,this),cartesianMotionMenu);

    circularMotionMenu = ContextMenuManager::Instance()->addMenuItem("Circular Motion");

    ContextMenuManager::Instance()->addActionItem("Create Circular Motion Marker",boost::bind(&BaseContextMenu::createCircularContextMenu,this),circularMotionMenu);
    ContextMenuManager::Instance()->addActionItem("Remove marker",boost::bind(&BaseContextMenu::removeCircularContextMenu,this),circularMotionMenu);

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


