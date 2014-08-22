/*
 * RobotModel class definition.
 * 
 * Author: Felipe Bacim.
 * 
 * Based on librviz_tutorials and the .
 * 
 * Latest changes (12/08/2012):
 * - created class
 */

#ifndef PERSPECTIVE_VIEW_H
#define PERSPECTIVE_VIEW_H

#include <QWidget>
#include <QElapsedTimer>

#include "base_3d_view.h"

namespace rviz
{
class RenderPanelCustom;
class ViewController;
class VisualizationManager;
class FPSViewController;
class OrbitViewController;
class VectorProperty;
class ViewsPanel;
}

namespace vigir_ocs
{
// Class "PerspectiveView" implements the RobotModel class with joint manipulation that can be added to any QT application.
class PerspectiveView: public Base3DView
{
    Q_OBJECT
public:
    PerspectiveView( QWidget *parent, Base3DView *copy_from = NULL, std::string base_frame = "/world" , std::string widget_name = "PerspectiveView" );
    PerspectiveView( Base3DView *copy_from = NULL, std::string base_frame = "/world" , std::string widget_name = "PerspectiveView", QWidget *parent = NULL );
    virtual ~PerspectiveView();

protected:
    virtual void timerEvent(QTimerEvent *event);
    virtual rviz::ViewController* getCurrentViewController();
    ros::Subscriber camera_transform_sub_;
    /**
      * ROS Callback: receives current camera pose
      */
    void processCameraTransform(const geometry_msgs::Pose::ConstPtr& msg);

private:
    void init();
    rviz::ViewsPanel* views_panel_;
    QElapsedTimer view_change_timer_;
};
}
#endif // PERSPECTIVE_VIEW_H
