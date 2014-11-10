
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
    ContextMenuManager* context_menu_manager = map_view_->getContextMenuManager();
    //request point cloud from tools section
    context_menu_manager->addActionItem("Request Point Cloud",boost::bind(&MapView::publishPointCloudWorldRequest,map_view_), NULL);

    context_menu_manager->addSeperator();

    context_menu_manager->addActionItem("Request Area Map",boost::bind(&MapViewContextMenu::requestAreaMapContext,this), NULL);

    context_menu_manager->addSeperator();

    context_menu_manager->addActionItem("Request Octomap",boost::bind(&MapViewContextMenu::requestOctomapContext,this), NULL);

    contextMenuItem * pointCloudMenu = context_menu_manager->addMenuItem("Request Point Clound Types");
    context_menu_manager->addActionItem("LIDAR filtered",boost::bind(&MapView::requestPointCloud,map_view_,0), pointCloudMenu);
    context_menu_manager->addActionItem("LIDAR unfiltered",boost::bind(&MapView::requestPointCloud,map_view_,1), pointCloudMenu);
    context_menu_manager->addActionItem("Stereo",boost::bind(&MapView::requestPointCloud,map_view_,2), pointCloudMenu);
    context_menu_manager->addActionItem("Stereo Sandia",boost::bind(&MapView::requestPointCloud,map_view_,3), pointCloudMenu);

    context_menu_manager->addSeperator();

    contextMenuItem * blockRegion = context_menu_manager->addMenuItem("Block Region");

    context_menu_manager->addActionItem("Axis-Aligned Rectangle",boost::bind(&MapView::blockRegionContext,map_view_, 1), blockRegion);
    context_menu_manager->addActionItem("Line",boost::bind(&MapView::blockRegionContext,map_view_, 0), blockRegion);

    contextMenuItem * clearRegion = context_menu_manager->addMenuItem("Clear Region");

    context_menu_manager->addActionItem("Axis-Aligned Rectangle",boost::bind(&MapView::clearRegionContext,map_view_, 1), clearRegion);
    context_menu_manager->addActionItem("Line",boost::bind(&MapView::clearRegionContext,map_view_, 0), clearRegion);

    context_menu_manager->addSeperator();
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

