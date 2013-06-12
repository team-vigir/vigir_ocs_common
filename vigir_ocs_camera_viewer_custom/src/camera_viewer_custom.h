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
class ViewportMouseEvent;
}

namespace Ogre
{
class Viewport;
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
    void setCropCameraSpeed( int );
    void unHighlight();
    void publishCropImageRequest();
    void publishFullImageRequest();

public Q_SLOTS:
    void changeCameraTopic( int );
    void changeFullImageResolution( int );
    void changeCropImageResolution( int );
    void changeCameraSpeed( int );
    void changeCropCameraSpeed( int );
    void applyFeedChanges();
    void applyAreaChanges();
    void requestSingleFeedImage();
    void requestSingleAreaImage();
    void disableSelection( );
    void changeAlpha(int);
    void closeSelectedArea();
    void mouseMoved(int,int);
    void requestPointCloudROI();
    void updateImageFrame(std::string);

private Q_SLOTS:
    void select( int, int, int, int );

private:
    int selected_area_[4];
    std::string camera_frame_topic_;

    rviz::Display* camera_viewer_;

    rviz::Tool* selection_tool_;

    ros::Publisher head_pitch_update_pub_;
    ros::Publisher pointcloud_request_frame_pub_;

    int feed_rate_;
    int feed_resolution_;
    int area_rate_;
    int area_resolution_;
};
}
#endif // CAMERA_VIEWER_H
