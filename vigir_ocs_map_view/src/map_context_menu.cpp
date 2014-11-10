
#include "map_context_menu.h"
#include <flor_ocs_msgs/OCSAugmentRegions.h>

namespace vigir_ocs
{

MapViewContextMenu::MapViewContextMenu(MapView* map_view)
{
    map_view_ = map_view;
    createContextMenu();

    // create publisher for augmentations on the grid map
    augment_grid_map_pub_ = nh_.advertise<flor_ocs_msgs::OCSAugmentRegions>( "/flor/worldmodel/ocs_augmented_regions", 1, false );

}

MapViewContextMenu::~MapViewContextMenu()
{
}


void MapViewContextMenu::createContextMenu()
{
    //request point cloud from tools section
    ContextMenuManager::Instance()->addActionItem("Request Point Cloud",boost::bind(&MapView::publishPointCloudWorldRequest,map_view_), NULL);

    ContextMenuManager::Instance()->addSeperator();    

    ContextMenuManager::Instance()->addActionItem("Request Area Map",boost::bind(&MapViewContextMenu::requestAreaMapContext,this), NULL);

    ContextMenuManager::Instance()->addSeperator();

    ContextMenuManager::Instance()->addActionItem("Request Octomap",boost::bind(&MapViewContextMenu::requestOctomapContext,this), NULL);

    contextMenuItem * pointCloudMenu = ContextMenuManager::Instance()->addMenuItem("Request Point Clound Types");
    ContextMenuManager::Instance()->addActionItem("LIDAR filtered",boost::bind(&MapView::requestPointCloud,map_view_,0), pointCloudMenu);
    ContextMenuManager::Instance()->addActionItem("LIDAR unfiltered",boost::bind(&MapView::requestPointCloud,map_view_,1), pointCloudMenu);
    ContextMenuManager::Instance()->addActionItem("Stereo",boost::bind(&MapView::requestPointCloud,map_view_,2), pointCloudMenu);
    ContextMenuManager::Instance()->addActionItem("Stereo Sandia",boost::bind(&MapView::requestPointCloud,map_view_,3), pointCloudMenu);

    ContextMenuManager::Instance()->addSeperator();

    contextMenuItem * blockRegion = ContextMenuManager::Instance()->addMenuItem("Block Region");

    ContextMenuManager::Instance()->addActionItem("Axis-Aligned Rectangle",boost::bind(&MapView::blockRegionContext,map_view_, 1), blockRegion);
    ContextMenuManager::Instance()->addActionItem("Line",boost::bind(&MapView::blockRegionContext,map_view_, 0), blockRegion);

    contextMenuItem * clearRegion = ContextMenuManager::Instance()->addMenuItem("Clear Region");

    ContextMenuManager::Instance()->addActionItem("Axis-Aligned Rectangle",boost::bind(&MapView::clearRegionContext,map_view_, 1), clearRegion);
    ContextMenuManager::Instance()->addActionItem("Line",boost::bind(&MapView::clearRegionContext,map_view_, 0), clearRegion);

    ContextMenuManager::Instance()->addSeperator();
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

//void MapViewContextMenu::blockRegionContext(int boxType)
//{
//    flor_ocs_msgs::OCSAugmentRegions augmentation;
//    augmentation.header.frame_id = base_frame_;
//    augmentation.map_selection = 2;
//    Ogre::Vector3 min, max;
//    Q_EMIT map_view_->queryPosition(selected_area_[0],selected_area_[1],min);
//    Q_EMIT map_view_->queryPosition(selected_area_[2],selected_area_[3],max);
//    flor_ocs_msgs::TwoPoint box;
//    box.min[0] = min.x;
//    box.min[1] = min.y;
//    box.max[0] = max.x;
//    box.max[1] = max.y;
//    box.type = boxType;
//    augmentation.blocked.push_back(box);
//    augment_grid_map_pub_.publish(augmentation);
//}

//void MapViewContextMenu::clearRegionContext(int boxType)
//{
//    flor_ocs_msgs::OCSAugmentRegions augmentation;
//    augmentation.header.frame_id = base_frame_;
//    augmentation.map_selection = 2;
//    Ogre::Vector3 min, max;
//    Q_EMIT queryPosition(selected_area_[0],selected_area_[1],min);
//    Q_EMIT queryPosition(selected_area_[2],selected_area_[3],max);
//    flor_ocs_msgs::TwoPoint box;
//    box.min[0] = min.x;
//    box.min[1] = min.y;
//    box.max[0] = max.x;
//    box.max[1] = max.y;
//    box.type = boxType;
//    augmentation.cleared.push_back(box);
//    augment_grid_map_pub_.publish(augmentation);
//}
//END CALLBACKS////////////////////////////////////////////

}

