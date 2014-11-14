#ifndef MAP_CONTEXT_MENU_H
#define MAP_CONTEXT_MENU_H

#include <QObject>
#include <ros/ros.h>
#include <string>
#include <boost/bind.hpp>
#include <stdlib.h>
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

private:
    MapView* map_view_;
    void createContextMenu();

    ros::NodeHandle nh_;
    //copied from map_view
    ros::Publisher augment_grid_map_pub_;

   //CALLBACKS///////////////////////////////////
    void requestAreaMapContext();
    void requestOctomapContext();

   //END CALLBACKS///////////////////////////

};

}
#endif //MAIN_CONTEXT_MENU_H
