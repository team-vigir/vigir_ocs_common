
#include "context_menu_manager.h"

namespace vigir_ocs
{

ContextMenuManager::ContextMenuManager()
{

}

ContextMenuManager::~ContextMenuManager()
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







}
