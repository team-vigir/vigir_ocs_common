
#include "base_context_menu.h"
namespace vigir_ocs
{

BaseContextMenu::BaseContextMenu(vigir_ocs::Base3DView* base_3d_view, FootstepVisManager* footstep_vis_manager)
{
    base_3d_view_ = base_3d_view;
    footstep_vis_manager_ = footstep_vis_manager;

}

BaseContextMenu::~BaseContextMenu()
{
}

void BaseContextMenu::processContextMenuVector(QAction* context_menu_selected_item)
{
    for(int i =0; i<context_menu_items_.size();i++)
    {
        //check parent if it exists?
        if(context_menu_items_[i]->parent != NULL && ((QMenu*)context_menu_selected_item->parent())->title() == context_menu_items_[i]->parent->name )
        {
            //check actual item
            if( context_menu_selected_item->text() == contextMenuItems[i]->name)
            {
                if(context_menu_items_[i]->function != NULL)
                {
                    context_menu_items_[i]->function(); //call binded function
                }
            }
        }
        else // no parent, must still check item
        {
            if( context_menu_selected_item->text() == contextMenuItems[i]->name)
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

void BaseContextMenu::getContextMenu()
{
    createContextMenu();
    buildContextMenu();
    return context_menu_;
}


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
    QTreeWidgetItemIterator it(templateRoot);
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
            addActionItem((*it)->text(0), localParent, boost::bind(&Base3DView::contextInsertTemplate,this,path));
        }
        ++it;
    }
}

void BaseContextMenu::buildContextMenu()
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
                contextMenuItems[i]->menu = menu;
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

void BaseContextMenu::createContextMenu()
{

}

}


