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
#include <stdlib.h>
#include "QMenu"
#include "QAction"


/**
 * WARN
 * Do not construct context items with duplicate names as visibility won't be able to be handled correctly.
 * -could add extra QString to contextMenuItem to handle item name rather than title name
 */

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

//forward declare to use in source file
namespace vigir_ocs
{
    class Base3DView;
}

class ContextMenuManager: public QObject
{
    //only initialized by base
    friend class vigir_ocs::Base3DView;

    Q_OBJECT
public:
    static ContextMenuManager* Instance();
    contextMenuItem *addMenuItem(QString name);
    contextMenuItem *addActionItem(QString name, boost::function<void()> function, contextMenuItem * parent);
    void addSeparatorItem();
    void setGlobalPos(QPoint globalPos);    
    void addCustomItem(contextMenuItem* item);

protected:
    ContextMenuManager(vigir_ocs::Base3DView* base_view);  // Only called by base3dview

private:    
    ContextMenuManager(ContextMenuManager const&){};             // copy constructor is private
    ContextMenuManager& operator=(ContextMenuManager const&){};  // assignment operator is private    

    void buildContextMenuHeirarchy();
    void resetMenu();

    void processContextMenu(int x, int y);
    void processContextMenuVector(QAction* context_menu_selected_item);

    //visibility handled after construction in associated widget
    void setItemVisibility(QString name, bool visibility);

    vigir_ocs::Base3DView* base_3d_view_;

    //stores heirarchy of the context menu to be constructed
    std::vector<contextMenuItem*> context_menu_items_;
    QMenu context_menu_;
    QAction* context_menu_selected_item_;
    int initializing_context_menu_;


public Q_SLOTS:
    void createContextMenu(bool, int x, int y);

Q_SIGNALS:
    void updateMainViewItems();

};

#endif // CONTEXT_MENU_MANAGER_H
