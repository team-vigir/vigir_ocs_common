
#include "base_context_menu.h"
namespace vigir_ocs
{

BaseContextMenu::BaseContextMenu(vigir_ocs::Base3DView* base_3d_view, FootstepVisManager* footstep_vis_manager)
{
    base_3d_view_ = base_3d_view;
    footstep_vis_manager_ = footstep_vis_manager;
    initializing_context_menu_ = 0;
}

BaseContextMenu::~BaseContextMenu()
{
}


void BaseContextMenu::setItemVisibility(QString name, bool visibility)
{
    contextMenuItem * item;
    //find context menu item in vector
    for(int i=0;i<context_menu_items_.size();i++)
    {
        if(context_menu_items_[i]->name == name)
            item = context_menu_items_[i];
    }
    //can only remove actions?
    if(!item->hasChildren)
        context_menu_->removeAction(item->action);
}


void BaseContextMenu::processContextMenuVector(QAction* context_menu_selected_item)
{
    for(int i =0; i<context_menu_items_.size();i++)
    {
        //check parent if it exists?
        if(context_menu_items_[i]->parent != NULL && ((QMenu*)context_menu_selected_item->parent())->title() == context_menu_items_[i]->parent->name )
        {
            //check actual item
            if( context_menu_selected_item->text() == context_menu_items_[i]->name)
            {
                if(context_menu_items_[i]->function != NULL)
                {
                    context_menu_items_[i]->function(); //call binded function
                }
            }
        }
        else // no parent, must still check item
        {
            if( context_menu_selected_item->text() == context_menu_items_[i]->name)
            {
                if(context_menu_items_[i]->function != NULL)
                {
                    context_menu_items_[i]->function(); //call binded function
                }
            }
        }
    }
}

void BaseContextMenu::addToContextMenuExternally(std::vector<contextMenuItem* > external_context_menu)
{
    for(int i=0;i<external_context_menu.size();i++)
    {
        context_menu_items_.push_back(external_context_menu[i]);
    }
}

//QMenu * BaseContextMenu::getContextMenu()
//{
//    if(!initialized_)
//    {
//        createContextMenu();
//        buildContextMenu();
//        initialized_ = true;
//    }
//    return context_menu_;
//}

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
            addActionItem((*it)->text(0), localParent, boost::bind(&BaseContextMenu::contextInsertTemplate,this,path));
        }
        ++it;
    }
}

//builds entire context menu including specific widgets
void BaseContextMenu::buildContextMenuHeirarchy()
{
    for(int i=0;i<context_menu_items_.size();i++)
    {
        if(context_menu_items_[i]->name == "Separator")
        {
            context_menu_.addSeparator();
            continue;
        }
        //top level menu item
        if(context_menu_items_[i]->parent == NULL)
        {
            if(context_menu_items_[i]->hasChildren)
            {
                QMenu * menu = context_menu_.addMenu(context_menu_items_[i]->name);
                context_menu_items_[i]->menu = menu;
            }
            else //no children, must be action
            {
                QAction * action = context_menu_.addAction(context_menu_items_[i]->name);
                context_menu_items_[i]->action = action;
            }
        }
        else // can guarantee parent has already been added provided elements were added in correct order to vector
        {
            if(context_menu_items_[i]->hasChildren)
            {
                QMenu * menu = context_menu_items_[i]->parent->menu->addMenu(context_menu_items_[i]->name);
                context_menu_items_[i]->menu = menu;
            }
            else
            {
                QAction * action = context_menu_items_[i]->parent->menu->addAction(context_menu_items_[i]->name);
                context_menu_items_[i]->action = action;
            }
        }
    }
    Q_EMIT updateMainViewItems();
}

void BaseContextMenu::createContextMenuItems()
{
    //creates all menu items for context menu in a base view, visibility is set on create

    // selection items
    selectTemplateMenu = addActionItem("Select Template",boost::bind(&BaseContextMenu::selectTemplate,this),NULL);

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


void BaseContextMenu::createContextMenu(bool, int x, int y)
{
    initializing_context_menu_++;

    context_menu_selected_item_ = NULL;

    resetMenu();

    // first we need to query the 3D scene to retrieve the context
    Q_EMIT base_3d_view_.queryContext(x,y);
    // context is stored in the active_context_ variable
    std::cout << "Active context: " << active_context_ << std::endl;


    //context_menu_ = base_context_menu_->getContextMenu();


    //insert stuff in constructor
    //have special case for empty vector item. insert separator when found
    //context_menu_.addAction("Insert Template");

    //ROS_ERROR("CONTEXT: %s",active_context_name_.c_str());

    //toggle visibility of context items for a base view

    //arms selection  only show appropriate arm
    if(active_context_name_.find("LeftArm") != std::string::npos)
    {
        base_context_menu_->setItemVisibility("Select Right Arm",false);
        //context_menu_->setItemVisibility("Select Right Arm",false);
    }
/**
    else if(active_context_name_.find("RightArm") != std::string::npos)
    {
        context_menu_->removeAction(leftArmMenu->action);
    }
    else //neither arm selected
    {
        context_menu_->removeAction(rightArmMenu->action);
        context_menu_->removeAction(leftArmMenu->action);
    }

    //remove footstep-related items if context is not footstep
    if(active_context_name_.find("footstep") == std::string::npos || active_context_name_.find("footstep goal") != std::string::npos)
    {

        context_menu_->removeAction(selectFootstepMenu->action);
        context_menu_->removeAction(lockFootstepMenu->action);
        context_menu_->removeAction(unlockFootstepMenu->action);
        context_menu_->removeAction(removeFootstepMenu->action);
        context_menu_->removeAction(selectStartFootstepMenu->action);
    }

    //footstep goal is still technically a footstep but need seperate case
    if(active_context_name_.find("footstep goal") == std::string::npos)
    {
        //remove context items as not needed
        context_menu_->removeAction(selectFootstepGoalMenu->action);
    }

    //cannot request footstep plan without goal
    //if(!footstep_vis_manager_->hasGoal())
    //{
    //    context_menu_.removeAction(defaultFootstepRequestMenu->action);
        context_menu_->removeAction(customFootstepRequestMenu->action);
    //}

    //cannot execute without footstep plan
    //if(!footstep_vis_manager_->hasValidStepPlan())
    //{
    //    context_menu_.removeAction(executeFootstepPlanMenu->action);
    //}

    if(!footstep_vis_manager_->hasStartingFootstep())
    {
        context_menu_.removeAction(clearStartFootstepMenu->action);
    }

    // context is stored in the active_context_ variable
    //lock/unlock arms context items
    if(active_context_name_.find("template") == std::string::npos)
    {
        //remove context items as not needed
        context_menu_.removeAction(removeTemplateMenu->action);
        context_menu_.removeAction(selectTemplateMenu->action);
        context_menu_.removeAction(lockLeftMenu->action);
        context_menu_.removeAction(lockRightMenu->action);
    }

    if((ghost_pose_source_[flor_ocs_msgs::OCSObjectSelection::RIGHT_ARM] && ghost_world_lock_[flor_ocs_msgs::OCSObjectSelection::RIGHT_ARM]) || (ghost_pose_source_[flor_ocs_msgs::OCSObjectSelection::LEFT_ARM] && ghost_world_lock_[flor_ocs_msgs::OCSObjectSelection::LEFT_ARM]))
    {
        //show only unlock
        context_menu_.removeAction(lockLeftMenu->action);
        context_menu_.removeAction(lockRightMenu->action);
    }
    else
    {
        //dont show unlock.. both arms are free and ready to be locked
        context_menu_.removeAction(unlockArmsMenu->action);
    }

//    if(flor_atlas_current_mode_ == 0 || flor_atlas_current_mode_ == 100)
//    {
//        executeFootstepPlanMenu->action->setEnabled(true);
//    }
//    else
//    {
//        context_menu_.removeAction(executeFootstepPlanMenu->action);
//    }

    if(cartesian_marker_list_.size() == 0)
    {
        cartesianMotionMenu->menu->removeAction(removeCartesianMarkerMenu->action);
        //removeCartesianMarkerMenu->action->setEnabled(false);
    }
    else
        removeCartesianMarkerMenu->action->setEnabled(true);

    if(circular_marker_ != NULL)
    {
        createCircularMarkerMenu->action->setEnabled(false);
        removeCircularMarkerMenu->action->setEnabled(true);
    }
    else if(circular_marker_ == NULL)
    {
        createCircularMarkerMenu->action->setEnabled(true);
        circularMotionMenu->menu->removeAction(removeCircularMarkerMenu->action);
        //removeCircularMarkerMenu->action->setEnabled(false);
    }

**/
    if(initializing_context_menu_ == 1)
        processContextMenu(x, y);

    initializing_context_menu_--;
}

void BaseContextMenu::setActiveContext(std::string name)
{
     active_context_name_ = name;
}


void BaseContextMenu::processContextMenu(int x, int y)
{
    QPoint globalPos = this->mapToGlobal(QPoint(x,y));
    context_menu_selected_item_ = base_context_menu_->getContextMenu()->exec(globalPos);

    //std::cout << selectedItem << std::endl;
    if(context_menu_selected_item_ != NULL)
    {
        base_context_menu_->processContextMenuVector(context_menu_selected_item_);
    }
}

//CALLBACKS///////

void BaseContextMenu::selectTemplate()
{
    base_3d_view_->selectTemplate();
}

}


