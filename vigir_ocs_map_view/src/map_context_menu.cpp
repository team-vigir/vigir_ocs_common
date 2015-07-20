/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013-2015, Team ViGIR ( TORC Robotics LLC, TU Darmstadt, Virginia Tech, Oregon State University, Cornell University, and Leibniz University Hanover )
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
//@TODO_ADD_AUTHOR_INFO

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

