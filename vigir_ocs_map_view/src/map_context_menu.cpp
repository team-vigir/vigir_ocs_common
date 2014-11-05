
#include "map_context_menu.h"

namespace vigir_ocs
{

MapViewContextMenu::MapViewContextMenu(MapView* map_view)
{
    map_view_ = map_view;
    createContextMenu();
}

MapViewContextMenu::~MapViewContextMenu()
{
}


void MapViewContextMenu::createContextMenu()
{
    //request point cloud from tools section
    ContextMenuManager::Instance()->addActionItem("Request Point Cloud",boost::bind(&MapView::publishPointCloudWorldRequest,map_view_), NULL);

    ContextMenuManager::Instance()->addSeperator();
    /**

    addActionItem("Request Area Map",boost::bind(&MapView::requestAreaMapContext,this), NULL);

    ContextMenuManager::Instance()->addSeperator();

    ContextMenuManager::Instance()->addActionItem("Request Octomap",boost::bind(&MapView::requestOctomapContext,this), NULL);

    pointCloudMenu = ContextMenuManager::Instance()->addMenuItem("Request Point Clound Types");
    ContextMenuManager::Instance()->addActionItem("LIDAR filtered",boost::bind(&MapView::requestPointCloud,this,0), pointCloudMenu);
    ContextMenuManager::Instance()->addActionItem("LIDAR unfiltered",boost::bind(&MapView::requestPointCloud,this,1), pointCloudMenu);
    ContextMenuManager::Instance()->addActionItem("Stereo",boost::bind(&MapView::requestPointCloud,this,2), pointCloudMenu);
    ContextMenuManager::Instance()->addActionItem("Stereo Sandia",boost::bind(&MapView::requestPointCloud,this,3), pointCloudMenu);

    ContextMenuManager::Instance()->addSeperator();

    blockRegion = ContextMenuManager::Instance()->addMenuItem("Block Region");

    ContextMenuManager::Instance()->addActionItem("Axis-Aligned Rectangle",boost::bind(&MapView::blockRegionContext,this, 1), blockRegion);
    ContextMenuManager::Instance()->addActionItem("Line",boost::bind(&MapView::blockRegionContext,this, 0), blockRegion);

    clearRegion = ContextMenuManager::Instance()->addMenuItem("Clear Region");

    ContextMenuManager::Instance()->addActionItem("Axis-Aligned Rectangle",boost::bind(&MapView::clearRegionContext,this, 1), clearRegion);
    ContextMenuManager::Instance()->addActionItem("Line",boost::bind(&MapView::clearRegionContext,this, 0), clearRegion);

    ContextMenuManager::Instance()->addSeperator();
    **/

}

//CALLBACKS/////////////////////

//void MainViewContextMenu::publishPointCloudWorldRequest()

}

