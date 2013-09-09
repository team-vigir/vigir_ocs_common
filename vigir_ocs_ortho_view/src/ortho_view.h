/*
 * OrthoView class definition.
 *
 * Author: Felipe Bacim.
 *
 * Based on librviz_tutorials and the .
 *
 * Latest changes (12/08/2012):
 */

#ifndef ORTHO_VIEW_H
#define ORTHO_VIEW_H

#include <QWidget>

#include <geometry_msgs/PoseStamped.h>

#include "base_3d_view.h"

namespace rviz
{
class RenderPanelCustom;
}

namespace vigir_ocs
{
// Class "OrthoView" implements the RobotModel class with joint manipulation that can be added to any QT application.
class OrthoView: public Base3DView
{
    Q_OBJECT
public:
    OrthoView( QWidget* parent = 0 );
    virtual ~OrthoView();

    void requestMap(double min_z, double max_z, double resolution);
    void requestOctomap(double min_z, double max_z, double resolution);

    void processGoalPose( const geometry_msgs::PoseStamped::ConstPtr& pose );

Q_SIGNALS:
    void queryPosition( int, int, Ogre::Vector3& );
    void unHighlight();

public Q_SLOTS:
    void enableSelectionTool(bool, int, int);
    virtual void vectorPressed();
    void keyPressEvent( QKeyEvent* event );

private:
    rviz::Tool* selection_tool_;

    ros::Publisher grid_map_request_pub_;
    ros::Publisher octomap_request_pub_;
    ros::Subscriber set_goal_sub_;

    int selected_area_[4];

    bool setting_pose_;
};
}
#endif // ORTHO_VIEW_H
