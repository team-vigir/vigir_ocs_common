/* 
 * CameraViewerCustom class definition.
 * 
 * Author: Felipe Bacim.
 * 
 * Based on librviz_tutorials.
 * 
 * Latest changes (12/04/2012):
 */

#ifndef CAMERA_VIEWER_H
#define CAMERA_VIEWER_H

#include <QWidget>
#include "rviz/selection/selection_manager.h"
#include <std_msgs/Float64.h>
#include <ros/ros.h>

#include "base_3d_view.h"

namespace rviz
{
class Display;
class RenderPanel;
class VisualizationManager;
class FrameManager;
class Tool;
}

namespace vigir_ocs
{
// Class "CameraViewerCustom" implements the QWidget that can be added to any QT application.
class CameraViewerCustom: public Base3DView
{
    Q_OBJECT
public:
    CameraViewerCustom( QWidget* parent = 0 );
    virtual ~CameraViewerCustom();

    void setCameraPitch( int );

Q_SIGNALS:
    void setFullImageResolution( int );
    void setCropImageResolution( int );
    void setCameraSpeed( int );

public Q_SLOTS:
    void changeCameraTopic( int );
    void changeFullImageResolution( int );
    void changeCropImageResolution( int );
    void changeCameraSpeed( int );
    void disableSelection( );
    void changeAlpha(int);
    void changeLayer(int);
    void changeZoom(int);


private Q_SLOTS:
    void select( int, int, int, int );

protected:
    void mouseMoveEvent(QMouseEvent *event);

private:
    bool okay;

    rviz::Display* camera_viewer_;

    rviz::Tool* selection_tool_;

    rviz::SelectionManager* select_manager_;

    ros::Publisher head_pitch_update_pub_;

};
}
#endif // CAMERA_VIEWER_H
