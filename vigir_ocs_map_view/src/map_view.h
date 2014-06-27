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

namespace rviz
{
class RenderPanelCustom;
}

namespace vigir_ocs
{
// Class "MapView" implements the RobotModel class with joint manipulation that can be added to any QT application.
class MapView: public OrthoView
{
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

public Q_SLOTS:
    void enableSelectionTool(bool, int, int);
    void selectionToolToggle(bool);
    virtual void createContextMenu( bool, int, int );
    virtual void processContextMenu( int x, int y );

private:
    rviz::Tool* selection_tool_;

    ros::Publisher grid_map_request_pub_;
    ros::Publisher augment_grid_map_pub_;
    ros::Publisher octomap_request_pub_;
    ros::Publisher point_cloud_request_pub_;

    bool selection_tool_enabled_;

    int selected_area_[4];

    flor_perception_msgs::PointCloudTypeRegionRequest last_request_;

    void addContextMenu();
    std::vector<contextMenuItem *> contextMenuElements;
};
}
#endif // map_view_H
