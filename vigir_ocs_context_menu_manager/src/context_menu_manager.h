/*
 * Context menu Manager class definition.
 *
 * Author: Brian Wright.
 */

#ifndef CONTEXT_MENU_MANAGER_H
#define CONTEXT_MENU_MANAGER_H

#include <QObject>

#include <ros/ros.h>

#include <string>
#include <boost/bind.hpp>
#include <vector>
#include <map>
#include <stdlib.h>
#include "QMenu"
#include "QAction"



namespace vigir_ocs
{

struct contextMenuItem
{
    QString name;
    //callback function of this item, sometimes null for parent items
    boost::function<void()> function;
    struct contextMenuItem * parent;
    //menu associated with this item, for children to add to menu
    QMenu* menu;
    //can only have action or menu. never both
    QAction* action;
    //tells whether to make an action or a menu object
    bool hasChildren;
};

class ContextMenuManager: public QObject
{
    Q_OBJECT
public:

    ContextMenuManager();
    virtual ~ContextMenuManager();


protected:

    // specific to every type of menu_manager, -set your own build order etc..
    //void createContextMenu();
    contextMenuItem *addMenuItem(QString name);
    contextMenuItem *addActionItem(QString name, boost::function<void()> function, contextMenuItem * parent);
    void addSeperator();
    //visibility handled after construction in associated widget
    void setItemVisibility(QString name, bool visibility);
    std::vector<contextMenuItem*> context_menu_items_;




};

}
#endif // CONTEXT_MENU_MANAGER_H
