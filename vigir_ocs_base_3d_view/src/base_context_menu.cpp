
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
            localParent->parent = insertTemplateMenu_;
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

    ContextMenuManager::Instance()->addActionItem("Select Left Arm",boost::bind(&Base3DView::selectLeftArm,base_3d_view_),NULL);
    ContextMenuManager::Instance()->addActionItem("Select Right Arm",boost::bind(&Base3DView::selectRightArm,base_3d_view_),NULL);

    ContextMenuManager::Instance()->addActionItem("Select Footstep",boost::bind(&Base3DView::selectFootstepGoal,base_3d_view_),NULL);
    ContextMenuManager::Instance()->addActionItem("Select Footstep",boost::bind(&Base3DView::selectFootstep,base_3d_view_),NULL);
    ContextMenuManager::Instance()->addActionItem("Lock Footstep",boost::bind(&Base3DView::lockFootstep,base_3d_view_),NULL);
    ContextMenuManager::Instance()->addActionItem("Unlock Footstep",boost::bind(&Base3DView::unlockFootstep,base_3d_view_),NULL);
    ContextMenuManager::Instance()->addActionItem("Remove Footstep",boost::bind(&Base3DView::removeFootstep,base_3d_view_),NULL);
    ContextMenuManager::Instance()->addActionItem("Set Starting Footstep",boost::bind(&Base3DView::setStartingFootstep,base_3d_view_),NULL);
    ContextMenuManager::Instance()->addActionItem("Clear Starting Footstep",boost::bind(&Base3DView::clearStartingFootstep,base_3d_view_),NULL);
    ContextMenuManager::Instance()->addActionItem("Stitch Plans",boost::bind(&Base3DView::stitchFootstepPlans,base_3d_view_),NULL);


    ContextMenuManager::Instance()->addSeperator();

    ContextMenuManager::Instance()->addActionItem("Snap Hand to Ghost",boost::bind(&Base3DView::snapHandGhost,this),NULL);


    ContextMenuManager::Instance()->addActionItem("Create Step Plan Goal",boost::bind(&Base3DView::defineFootstepGoal,base_3d_view_), NULL);
    ContextMenuManager::Instance()->addActionItem("Request Step Plan",boost::bind(&Base3DView::requestFootstepPlan,base_3d_view_,flor_ocs_msgs::OCSFootstepPlanRequest::NEW_PLAN), NULL);
    ContextMenuManager::Instance()->addActionItem("Request Step Plan...",boost::bind(&Base3DView::requestFootstepPlan,base_3d_view_,flor_ocs_msgs::OCSFootstepPlanRequest::NEW_PLAN), NULL);
    ContextMenuManager::Instance()->addActionItem(QString("Execute Step Plan"),boost::bind(&Base3DView::executeFootstepPlanContextMenu,base_3d_view_),NULL);
    ContextMenuManager::Instance()->addActionItem("Undo Step Change",boost::bind(&FootstepVisManager::requestFootstepListUndo,footstep_vis_manager_),NULL);
    ContextMenuManager::Instance()->addActionItem("Redo Step Change",boost::bind(&FootstepVisManager::requestFootstepListRedo,footstep_vis_manager_),NULL);

    ContextMenuManager::Instance()->addSeperator();

    insertTemplateMenu_ = ContextMenuManager::Instance()->addMenuItem("Insert Template");
    ContextMenuManager::Instance()->addActionItem("Remove Template",boost::bind(&BaseContextMenu::removeTemplateContextMenu,this),NULL);

    ContextMenuManager::Instance()->addSeperator();

    ContextMenuManager::Instance()->addActionItem("Lock Left Arm to Template",boost::bind(&Base3DView::setTemplateGraspLock,base_3d_view_,flor_ocs_msgs::OCSObjectSelection::LEFT_ARM),NULL);
    ContextMenuManager::Instance()->addActionItem("Lock Right Arm to Template",boost::bind(&Base3DView::setTemplateGraspLock,base_3d_view_,flor_ocs_msgs::OCSObjectSelection::RIGHT_ARM),NULL);
    ContextMenuManager::Instance()->addActionItem("Unlock Arms",boost::bind(&Base3DView::setTemplateGraspLock,base_3d_view_,-1),NULL);

    ContextMenuManager::Instance()->addSeperator();

    contextMenuItem * cartesianMotionMenu = ContextMenuManager::Instance()->addMenuItem("Cartesian Motion");

    ContextMenuManager::Instance()->addActionItem("Create Cartesian Motion Marker",boost::bind(&Base3DView::createCartesianContextMenu,base_3d_view_),cartesianMotionMenu);
    ContextMenuManager::Instance()->addActionItem("Remove All Markers",boost::bind(&Base3DView::removeCartesianContextMenu,base_3d_view_),cartesianMotionMenu);

    contextMenuItem * circularMotionMenu = ContextMenuManager::Instance()->addMenuItem("Circular Motion");

    ContextMenuManager::Instance()->addActionItem("Create Circular Motion Marker",boost::bind(&Base3DView::createCircularContextMenu,base_3d_view_),circularMotionMenu);
    ContextMenuManager::Instance()->addActionItem("Remove marker",boost::bind(&Base3DView::removeCircularContextMenu,base_3d_view_),circularMotionMenu);

    ContextMenuManager::Instance()->addSeperator();

}





//CALLBACKS///////
void BaseContextMenu::contextInsertTemplate(QString path)
{
    base_3d_view_->insertTemplate(path);
}

void BaseContextMenu::removeTemplateContextMenu()
{
    std::string active_context_name = ContextMenuManager::Instance()->getCurrentActiveContext();
    int start = active_context_name.find(" ")+1;
    int end = active_context_name.find(".");
    QString template_number(active_context_name.substr(start, end-start).c_str());
    //ROS_INFO("%d %d %s",start,end,template_number.toStdString().c_str());
    bool ok;
    int t = template_number.toInt(&ok);
    if(ok) base_3d_view_->removeTemplate(t);
}

}


