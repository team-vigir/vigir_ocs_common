
#include "map_context_menu.h"
#include <vigir_ocs_msgs/OCSAugmentRegions.h>

namespace vigir_ocs
{

MapViewContextMenu::MapViewContextMenu(MapView* map_view)
{
    map_view_ = map_view;
    createContextMenu();

    // create publisher for augmentations on the grid map
    augment_grid_map_pub_ = nh_.advertise<vigir_ocs_msgs::OCSAugmentRegions>( "/flor/worldmodel/ocs_augmented_regions", 1, false );
}

MapViewContextMenu::~MapViewContextMenu()
{
}


void MapViewContextMenu::createContextMenu()
{
    ContextMenuManager* context_menu_manager = map_view_->getContextMenuManager();
    //request point cloud from tools section
    context_menu_manager->addActionItem("Request Point Cloud",boost::bind(&MapView::publishPointCloudWorldRequest,map_view_), NULL);

    context_menu_manager->addSeparatorItem();

    context_menu_manager->addActionItem("Request Area Map",boost::bind(&MapViewContextMenu::requestAreaMapContext,this), NULL);

    context_menu_manager->addSeparatorItem();

    context_menu_manager->addActionItem("Request Octomap",boost::bind(&MapViewContextMenu::requestOctomapContext,this), NULL);

    contextMenuItem * pointCloudMenu = context_menu_manager->addMenuItem("Request Point Clound Types");
    context_menu_manager->addActionItem("LIDAR filtered",boost::bind(&MapView::requestPointCloud,map_view_,0), pointCloudMenu);
    context_menu_manager->addActionItem("LIDAR unfiltered",boost::bind(&MapView::requestPointCloud,map_view_,1), pointCloudMenu);
    context_menu_manager->addActionItem("Stereo",boost::bind(&MapView::requestPointCloud,map_view_,2), pointCloudMenu);
    context_menu_manager->addActionItem("Stereo Sandia",boost::bind(&MapView::requestPointCloud,map_view_,3), pointCloudMenu);

    context_menu_manager->addSeparatorItem();

    contextMenuItem * blockRegion = context_menu_manager->addMenuItem("Block Region");

    context_menu_manager->addActionItem("Axis-Aligned Rectangle",boost::bind(&MapView::blockRegionContext,map_view_, 1), blockRegion);
    context_menu_manager->addActionItem("Line",boost::bind(&MapView::blockRegionContext,map_view_, 0), blockRegion);

    contextMenuItem * clearRegion = context_menu_manager->addMenuItem("Clear Region");

    context_menu_manager->addActionItem("Axis-Aligned Rectangle",boost::bind(&MapView::clearRegionContext,map_view_, 1), clearRegion);
    context_menu_manager->addActionItem("Line",boost::bind(&MapView::clearRegionContext,map_view_, 0), clearRegion);

    context_menu_manager->addSeparatorItem();
}

//CALLBACKS/////////////////////

void MapViewContextMenu::requestAreaMapContext()
{
    Q_EMIT map_view_->UIrequestAreaMap();
}

void MapViewContextMenu::requestOctomapContext()
{
    Q_EMIT map_view_->UIrequestOctomap();
}

//END CALLBACKS////////////////////////////////////////////

}

