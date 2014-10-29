
#ifndef BASE_CONTEXT_MENU_H
#define BASE_CONTEXT_MENU_H

#include <QObject>

#include <ros/ros.h>

#include <string>
#include <boost/bind.hpp>
#include <vector>
#include <map>
#include <stdlib.h>
#include <QTreeWidget>
#include "context_menu_manager.h"
#include "main_view_widget.h"


namespace vigir_ocs
{
class MainViewContextMenu: public ContextMenuManager
{
    Q_OBJECT
public:

    MainViewContextMenu(MainViewWidget* main_view);
    virtual ~MainViewContextMenu();
   
Q_SIGNALS:

public Q_SLOTS;
    void updateContextMenu();
private:   
    MainViewWidget* main_view_;
    void createContextMenu();

    //context item references for checking/unchecking in context menu
    contextMenuItem * joystickContext;
    contextMenuItem * positionContext;
    contextMenuItem * graspContext;
    contextMenuItem * jointControlContext;
    contextMenuItem * pelvisContext;
    contextMenuItem * ghostContext;
    contextMenuItem * plannerContext;
    contextMenuItem * footBasicContext;
    contextMenuItem * footAdvancedContext;
    contextMenuItem * footParameterContext;
    contextMenuItem * objectContext;
    contextMenuItem * worldContext;
    contextMenuItem * cameraContext;
   //CALLBACKS///////////////////////////////////



   //END CALLBACKS///////////////////////////

};

}
#endif BASE_CONTEXT_MENU_H
