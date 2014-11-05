
#ifndef MAP_CONTEXT_MENU_H
#define MAP_CONTEXT_MENU_H

#include <QObject>

#include <ros/ros.h>

#include <string>
#include <boost/bind.hpp>
#include <vector>
#include <map>
#include <stdlib.h>
#include <QTreeWidget>
#include "context_menu_manager.h"
#include "map_view.h"


namespace vigir_ocs
{
class MapViewContextMenu: public QObject
{
    Q_OBJECT
public:

    MapViewContextMenu(MapView* map_view);
    virtual ~MapViewContextMenu();

Q_SIGNALS:

public Q_SLOTS;
    void updateContextMenu();
private:
    MapView* map_view_;
    void createContextMenu();

    //context item references for checking/unchecking in context menu
    contextMenuItem * blockRegion;
    contextMenuItem * clearRegion;
    contextMenuItem * pointCloudMenu;
   //CALLBACKS///////////////////////////////////



   //END CALLBACKS///////////////////////////

};

}
#endif //MAIN_CONTEXT_MENU_H
