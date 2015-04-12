
#include "context_menu_manager.h"
#include "base_3d_view.h"

ContextMenuManager::ContextMenuManager(vigir_ocs::Base3DView *base_view)
{
    initializing_context_menu_ = 0;    
    base_3d_view_ = base_view;
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

void ContextMenuManager::addCustomItem(contextMenuItem* item)
{
    context_menu_items_.push_back(item);
}

void ContextMenuManager::addSeparatorItem()
{
    contextMenuItem * child = new contextMenuItem();
    child->name = "Separator";
    context_menu_items_.push_back(child);
}

//builds entire context menu including specific widgets
void ContextMenuManager::buildContextMenuHierarchy()
{
    for(int i=0;i<context_menu_items_.size();i++)
    {
        if(context_menu_items_[i]->name.contains("Separator"))
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
        else // can guarantee parent has already been added provided elements must be added in correct order to vector
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
    //update main view ischecked properties if needed
    Q_EMIT updateMainViewItems();
}

void ContextMenuManager::resetMenu()
{
    context_menu_.clear();
    context_menu_.setTitle("Base Menu");
    context_menu_.setStyleSheet("font-size:11px;");    
}

void ContextMenuManager::createContextMenu(bool, int x, int y)
{
    initializing_context_menu_++;

    context_menu_selected_item_ = NULL;

    resetMenu();

    // first we need to query the 3D scene to retrieve the context
    base_3d_view_->emitQueryContext(x,y);

    buildContextMenuHierarchy();

    //toggle visibility of context items for a base view

    //arms selection  only show appropriate arm
    if(base_3d_view_->getActiveContext().find("LeftArm") != std::string::npos)
    {
        setItemVisibility("Select Right Arm",false);

    }
    else if(base_3d_view_->getActiveContext().find("RightArm") != std::string::npos)
    {
        setItemVisibility("Select Left Arm",false);
    }
    else //neither arm selected
    {
        setItemVisibility("Select Right Arm",false);
        setItemVisibility("Select Left Arm",false);
    }

    //remove footstep-related items if context is not footstep
    if(base_3d_view_->getActiveContext().find("footstep") == std::string::npos || base_3d_view_->getActiveContext().find("footstep goal") != std::string::npos)
    {
        setItemVisibility("Set Starting Footstep",false);
        setItemVisibility("Select Footstep",false);
    }

    //setItemVisibility("Lock Footstep",false);
    //setItemVisibility("Unlock Footstep",false);
    //setItemVisibility("Remove Footstep",false);

    //footstep goal is still technically a footstep but need seperate case
    if(base_3d_view_->getActiveContext().find("footstep goal") == std::string::npos)
    {
        setItemVisibility("Select Footstep Goal",false);
    }

    //cannot request footstep plan without goal
    //if(!base_3d_view_->getFootstepVisManager()->hasGoal())
    //{
        //setItemVisibility("Request Step Plan",false);
        //setItemVisibility("Request Step Plan...",false);
    //}

    //cannot execute without footstep plan
    if(!base_3d_view_->getFootstepVisManager()->hasValidStepPlan())
    {
        setItemVisibility("Execute Step Plan",false);
    }

    if(!base_3d_view_->getFootstepVisManager()->hasStartingFootstep())
    {
        setItemVisibility("Clear Starting Footstep",false);
    }

    // context is stored in the active_context_ variable
    //lock/unlock arms context items
    if(base_3d_view_->getActiveContext().find("template") == std::string::npos)
    {
        //remove context items as not needed
        setItemVisibility("Remove Template",false);
        setItemVisibility("Select Template",false);
        setItemVisibility("Lock Left Arm to Template",false);
        setItemVisibility("Lock Right Arm to Template",false);
    }

    if((base_3d_view_->getGhostPoseSource()[flor_ocs_msgs::OCSObjectSelection::RIGHT_ARM] && base_3d_view_->getGhostWorldLock()[flor_ocs_msgs::OCSObjectSelection::RIGHT_ARM]) || (base_3d_view_->getGhostPoseSource()[flor_ocs_msgs::OCSObjectSelection::LEFT_ARM] && base_3d_view_->getGhostWorldLock()[flor_ocs_msgs::OCSObjectSelection::LEFT_ARM]))
    {
        //show only unlock        
        setItemVisibility("Lock Left Arm to Template",false);
        setItemVisibility("Lock Right Arm to Template",false);
    }
    else
    {
        //dont show unlock.. both arms are free and ready to be locked        
        setItemVisibility("Unlock Arms",false);
    }
//    if(flor_atlas_current_mode_ == 0 || flor_atlas_current_mode_ == 100)
//    {
//        executeFootstepPlanMenu->action->setEnabled(true);
//    }
//    else
//    {
//        context_menu_.removeAction(executeFootstepPlanMenu->action);
//    }

    if(base_3d_view_->getCartesianMarkerList().size() == 0)
    {
        //remove cartesian marker menu
        setItemVisibility("Remove All Markers",false);
    }
    else
        setItemVisibility("Remove All Markers",true);


    if(base_3d_view_->getCircularMarker() != NULL)
    {
        setItemVisibility("Create Circular Motion Marker",false);
        setItemVisibility("Remove marker",true);
    }
    else if(base_3d_view_->getCircularMarker() == NULL)
    {
        setItemVisibility("Create Circular Motion Marker",true);
        setItemVisibility("Remove marker",false);
    }

    if(initializing_context_menu_ == 1)
        processContextMenu(x, y);

    initializing_context_menu_--;
}


void ContextMenuManager::processContextMenu(int x, int y)
{
    //tells base3dview to send globalpos   
    QPoint globalPos = base_3d_view_->mapToGlobal(QPoint(x,y));
    context_menu_selected_item_ = context_menu_.exec(globalPos);

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
        context_menu_.removeAction(item->action);
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



