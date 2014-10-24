
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

    MainViewContextMenu();
    virtual ~MainViewContextMenu();
   
Q_SIGNALS:


private:   
    void createContextMenu();


   //CALLBACKS///////////////////////////////////



   //END CALLBACKS///////////////////////////

};

}
#endif BASE_CONTEXT_MENU_H
