
#include "context_menu_manager.h"

namespace vigir_ocs
{

ContextMenuManager* ContextMenuManager::instance = 0;


ContextMenuManager* ContextMenuManager::Instance()
{
   if (!instance)   // Only allow one instance of class to be generated.
       instance = new ContextMenuManager();

   return instance;
}
ContextMenuManager::ContextMenuManager()
{

}

contextMenuItem * ContextMenuManager::addMenuItem(QString name)
{
    contextMenuItem * parent = new contextMenuItem();
    parent->name = name;
    parent->hasChildren = true;
    context_menu_items_.push_back(parent);
    return parent;
}


contextMenuItem *  ContextMenuManager::addActionItem(QString name, boost::function<void()> function, contextMenuItem * parent)
{
    contextMenuItem * child = new contextMenuItem();
    child->name = name;
    child->function = function;
    child->parent = parent;
    child->hasChildren = false;
    context_menu_items_.push_back(child);
    return child;
}

void ContextMenuManager::addSeperator()
{
    contextMenuItem * child = new contextMenuItem();
    child->name = "Seperator";
    context_menu_items_.push_back(child);
}

//builds entire context menu including specific widgets
void ContextMenuManager::buildContextMenuHeirarchy()
{
    for(int i=0;i<context_menu_items_.size();i++)
    {
        if(context_menu_items_[i]->name == "Separator")
        {
            context_menu_->addSeparator();
            continue;
        }
        //top level menu item
        if(context_menu_items_[i]->parent == NULL)
        {
            if(context_menu_items_[i]->hasChildren)
            {
                QMenu * menu = context_menu_->addMenu(context_menu_items_[i]->name);
                context_menu_items_[i]->menu = menu;
            }
            else //no children, must be action
            {
                QAction * action = context_menu_->addAction(context_menu_items_[i]->name);
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
    //Q_EMIT updateMainViewItems();
}

void ContextMenuManager::createContextMenu(bool, int x, int y)
{
    initializing_context_menu_++;

    context_menu_selected_item_ = NULL;

    resetMenu();

    // first we need to query the 3D scene to retrieve the context
    base_3d_view_->emitQueryContext(x,y);
    // context is stored in the active_context_ variable
    //std::cout << "Active context: " << active_context_ << std::endl;

    //toggle visibility of context items for a base view

    //arms selection  only show appropriate arm
    if(active_context_name_.find("LeftArm") != std::string::npos)
    {
        setItemVisibility("Select Right Arm",false);
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


void ContextMenuManager::setActiveContext(std::string name,int num)
{
    active_context_ = num; //necessary?
    active_context_name_ = name;
}


void ContextMenuManager::processContextMenu(int x, int y)
{
    QPoint globalPos = base_3d_view_->mapToGlobal(QPoint(x,y));
    context_menu_selected_item_ = context_menu_->exec(globalPos);

    //std::cout << selectedItem << std::endl;
    if(context_menu_selected_item_ != NULL)
    {
        processContextMenuVector(context_menu_selected_item_);
    }
}

void ContextMenuManager::setItemVisibility(QString name, bool visibility)
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


void ContextMenuManager::processContextMenuVector(QAction* context_menu_selected_item)
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







}
