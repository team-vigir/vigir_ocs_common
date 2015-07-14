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
//@TODO_ADD_AUTHOR_INFO
/*
 * MapView class definition.
 *
 * Author: Felipe Bacim.
 *
 * Based on librviz_tutorials and the .
 *
 * Latest changes (12/08/2012):
 */

#ifndef MAP_VIEW_H
#define MAP_VIEW_H

#include <QWidget>

#include <geometry_msgs/PoseStamped.h>

#include "ortho_view.h"
#include "interaction_tool_custom.h"
#include "notification_system.h"
#include <ui/footstep_config.h>



namespace rviz
{
class RenderPanelCustom;
}

namespace vigir_ocs
{

//forward declare to reference later
class MapViewContextMenu;

// Class "MapView" implements the RobotModel class with joint manipulation that can be added to any QT application.
class MapView: public OrthoView
{
    friend class MapViewContextMenu;
    Q_OBJECT
public:
    MapView( QWidget* parent = 0 );
    virtual ~MapView();

    void requestMap(double min_z, double max_z, double resolution);
    void requestOctomap(double min_z, double max_z, double resolution);
    void requestPointCloud(double min_z, double max_z, double resolution, int type, int aggregation_size);
    void requestPointCloud(int type);

    bool hasValidSelection();

Q_SIGNALS:
    void queryPosition( int, int, Ogre::Vector3& );
    void unHighlight();
    void UIrequestAreaMap();
    void UIrequestOctomap();

public Q_SLOTS:
    void enableSelectionTool(bool, int, int);
    void selectionToolToggle(bool);

private:
    rviz::Tool* selection_tool_;

    ros::Publisher grid_map_request_pub_;
    ros::Publisher augment_grid_map_pub_;
    ros::Publisher octomap_request_pub_;
    ros::Publisher point_cloud_request_pub_;

    bool selection_tool_enabled_;

    int selected_area_[4];

    vigir_perception_msgs::PointCloudTypeRegionRequest last_request_;   

    MapViewContextMenu * map_view_context_menu_;

protected:
    void blockRegionContext(int boxType);
    void clearRegionContext(int boxType);
    };
}
#endif // map_view_H
