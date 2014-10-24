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

namespace vigir_ocs
{
class ContextMenuManager: public QObject
{
    Q_OBJECT
public:

    ContextMenuManager();
    virtual ~ContextMenuManager();

    //visibility handled after construction in associated widget
    void setItemVisibility(QString name, bool visibility);

private:  

    // specific to every type of menu_manager, -set your own build order etc..
    void createContextMenu();
    void addMenuItem(QString name);
    void addActionItem(QString name, contextMenuItem * parent, boost::function<void()> function);
    void addSeperator();
    std::vector<contextMenuItem*> context_menu_items_;

    QMenu context_menu_;
    QAction* context_menu_selected_item_;

};

}
#endif // CONTEXT_MENU_MANAGER_H
