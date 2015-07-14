/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013-2015, TORC Robotics, LLC ( Team ViGIR )
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Team ViGIR, TORC Robotics, nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
@TODO_ADD_AUTHOR_INFO
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
#include <QMenu>
#include <QAction>
#include <QToolTip>


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
    void setToolTip(QString name, QString tooltip);

protected:
    ContextMenuManager(vigir_ocs::Base3DView* base_view);  // Only called by base3dview

private:    
    ContextMenuManager(ContextMenuManager const&){};             // copy constructor is private
    ContextMenuManager& operator=(ContextMenuManager const&){};  // assignment operator is private    

   // bool event(QEvent * evt);
    void buildContextMenuHierarchy();
    void resetMenu();

    void processContextMenu(int x, int y);
    void processContextMenuVector(QAction* context_menu_selected_item);

    //visibility handled after construction in associated widget
    void setItemVisibility(QString name, bool visibility);
    //grey things out if necessary
    void setItemEnabled(QString name, bool enabled);

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
