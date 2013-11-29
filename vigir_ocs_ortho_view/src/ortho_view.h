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
class ViewController;
class VisualizationManager;
}

namespace vigir_ocs
{
// Class "OrthoView" implements the RobotModel class with joint manipulation that can be added to any QT application.
class OrthoView: public Base3DView
{
    Q_OBJECT
public:
    OrthoView( rviz::VisualizationManager* context = NULL, std::string base_frame = "/world", QWidget* parent = 0 );
    virtual ~OrthoView();

    void processGoalPose( const geometry_msgs::PoseStamped::ConstPtr& pose );
    
    void setViewPlane(const QString& view_plane);

Q_SIGNALS:
    void queryPosition( int, int, Ogre::Vector3& );

public Q_SLOTS:
    virtual void defineWalkPosePressed();
    virtual void defineStepPosePressed();
    void keyPressEvent( QKeyEvent* event );

protected:
    virtual void timerEvent(QTimerEvent *event);
    virtual rviz::ViewController* getCurrentViewController();

    rviz::Tool* previous_tool_;
    rviz::ViewController* ortho_view_controller_;

    ros::Subscriber set_walk_goal_sub_;
    ros::Subscriber set_step_goal_sub_;

    bool setting_pose_;
};
}
#endif // ORTHO_VIEW_H
