
#include "base_context_menu.h"
namespace vigir_ocs
{

BaseContextMenu::BaseContextMenu(vigir_ocs::Base3DView* base_3d_view)
{
    base_3d_view_ = base_3d_view;
    footstep_vis_manager_ = base_3d_view_->getFootstepVisManager();
    //init vector with elements from base3dview
    context_menu_manager_ = base_3d_view_->getContextMenuManager();
    //add all other context items
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
        //want insert template to be first context item
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
            context_menu_manager_->addCustomItem(localParent);
        }
        else
        {
            //build path to template for insertion
            QString path = localParent->name + "/" + (*it)->text(0);
            // ROS_ERROR("path %s",qPrintable(path));
            context_menu_manager_->addActionItem((*it)->text(0), boost::bind(&BaseContextMenu::contextInsertTemplate,this,path),localParent);
        }
        ++it;
    }
}

void BaseContextMenu::createContextMenuItems()
{
    //creates all menu items for context menu in a base view, visibility is set on create

    context_menu_manager_->addActionItem("Select Template",boost::bind(&Base3DView::selectTemplate,base_3d_view_),NULL);

    context_menu_manager_->addActionItem("Select Left Arm",boost::bind(&Base3DView::selectLeftArm,base_3d_view_),NULL);
    context_menu_manager_->addActionItem("Select Right Arm",boost::bind(&Base3DView::selectRightArm,base_3d_view_),NULL);

    context_menu_manager_->addActionItem("Select Footstep Goal",boost::bind(&Base3DView::selectFootstepGoal,base_3d_view_),NULL);
    context_menu_manager_->addActionItem("Select Footstep",boost::bind(&Base3DView::selectFootstep,base_3d_view_),NULL);
    //context_menu_manager_->addActionItem("Lock Footstep",boost::bind(&Base3DView::lockFootstep,base_3d_view_),NULL);
    //context_menu_manager_->addActionItem("Unlock Footstep",boost::bind(&Base3DView::unlockFootstep,base_3d_view_),NULL);
    //context_menu_manager_->addActionItem("Remove Footstep",boost::bind(&Base3DView::removeFootstep,base_3d_view_),NULL);
    context_menu_manager_->addActionItem("Set Starting Footstep",boost::bind(&Base3DView::setStartingFootstep,base_3d_view_),NULL);
    context_menu_manager_->addActionItem("Clear Starting Footstep",boost::bind(&FootstepVisManager::clearStartingFootstep,footstep_vis_manager_),NULL);

    context_menu_manager_->addSeparatorItem();

    context_menu_manager_->addActionItem("Create Step Plan Goal",boost::bind(&Base3DView::defineFootstepGoal,base_3d_view_), NULL);
    //context_menu_manager_->addActionItem("Request Step Plan",boost::bind(&FootstepVisManager::requestStepPlan,footstep_vis_manager_), NULL);
                                                                                        //will be custom plan?
    //context_menu_manager_->addActionItem("Request Step Plan...",boost::bind(&FootstepVisManager::requestStepPlan,footstep_vis_manager_), NULL);
    context_menu_manager_->addActionItem("Stitch Plans",boost::bind(&FootstepVisManager::requestStitchFootstepPlans,footstep_vis_manager_),NULL);
    context_menu_manager_->addActionItem("Validate Step Plan",boost::bind(&FootstepVisManager::requestValidateStepPlan,footstep_vis_manager_),NULL);
    context_menu_manager_->addActionItem("Execute Step Plan",boost::bind(&FootstepVisManager::requestExecuteStepPlan,footstep_vis_manager_),NULL);
    context_menu_manager_->addActionItem("Undo Step Change",boost::bind(&FootstepVisManager::requestFootstepListUndo,footstep_vis_manager_),NULL);
    context_menu_manager_->addActionItem("Redo Step Change",boost::bind(&FootstepVisManager::requestFootstepListRedo,footstep_vis_manager_),NULL);

    context_menu_manager_->addSeparatorItem();

    insertTemplateMenu_ = context_menu_manager_->addMenuItem("Insert Template");
    context_menu_manager_->addActionItem("Remove Template",boost::bind(&BaseContextMenu::removeTemplateContextMenu,this),NULL);

    context_menu_manager_->addSeparatorItem();

    context_menu_manager_->addActionItem("Snap Hand to Ghost",boost::bind(&Base3DView::snapHandGhost,base_3d_view_),NULL);
    context_menu_manager_->addActionItem("Lock Left Arm to Template",boost::bind(&Base3DView::setTemplateGraspLock,base_3d_view_,flor_ocs_msgs::OCSObjectSelection::LEFT_ARM),NULL);
    context_menu_manager_->addActionItem("Lock Right Arm to Template",boost::bind(&Base3DView::setTemplateGraspLock,base_3d_view_,flor_ocs_msgs::OCSObjectSelection::RIGHT_ARM),NULL);
    context_menu_manager_->addActionItem("Unlock Arms",boost::bind(&Base3DView::setTemplateGraspLock,base_3d_view_,-1),NULL);

    context_menu_manager_->addSeparatorItem();

    contextMenuItem * cartesianMotionMenu = context_menu_manager_->addMenuItem("Cartesian Motion");

    context_menu_manager_->addActionItem("Create Cartesian Motion Marker",boost::bind(&Base3DView::createCartesianContextMenu,base_3d_view_),cartesianMotionMenu);
    context_menu_manager_->addActionItem("Remove All Markers",boost::bind(&Base3DView::removeCartesianContextMenu,base_3d_view_),cartesianMotionMenu);

    contextMenuItem * circularMotionMenu = context_menu_manager_->addMenuItem("Circular Motion");

    context_menu_manager_->addActionItem("Create Circular Motion Marker",boost::bind(&Base3DView::createCircularContextMenu,base_3d_view_),circularMotionMenu);
    context_menu_manager_->addActionItem("Remove marker",boost::bind(&Base3DView::removeCircularContextMenu,base_3d_view_),circularMotionMenu);

    context_menu_manager_->addSeparatorItem();
}

//CALLBACKS///////
void BaseContextMenu::contextInsertTemplate(QString path)
{
    base_3d_view_->insertTemplate(path);
}

void BaseContextMenu::removeTemplateContextMenu()
{
    std::string active_context_name = base_3d_view_->getActiveContext();
    int start = active_context_name.find(" ")+1;
    int end = active_context_name.find(".");
    QString template_number(active_context_name.substr(start, end-start).c_str());
    //ROS_INFO("%d %d %s",start,end,template_number.toStdString().c_str());
    bool ok;
    int t = template_number.toInt(&ok);
    if(ok) base_3d_view_->removeTemplate(t);
}
//END CALLBACKS////////////////////////////////////////////

}


