
#include "context_menu_manager.h"

namespace vigir_ocs
{

ContextMenuManager::ContextMenuManager()
{

}

ContextMenuManager::~ContextMenuManager()
{

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

void ContextMenuManager::addMenuItem(QString name)
{
    contextMenuItem * parent = new contextMenuItem();
    parent->name = name;
    parent->hasChildren = true;
    context_menu_items_.push_back(parent);
}


void ContextMenuManager::addActionItem(QString name, contextMenuItem * parent, boost::function<void()> function)
{
    contextMenuItem * child = new contextMenuItem();
    child->name = name;
    child->function = function;
    child->parent = parent;
    child->hasChildren = false;
    context_menu_items_.push_back(child);
}

void ContextMenuManager::addSeperator()
{
    contextMenuItem * child = new contextMenuItem();
    child->name = "Seperator";
    context_menu_items_.push_back(child);
}







}
