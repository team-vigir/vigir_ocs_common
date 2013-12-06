/* 
 * CameraView class definition.
 * 
 * Author: Felipe Bacim.
 * 
 * Based on librviz_tutorials.
 * 
 * Latest changes (12/04/2012):
 */

#ifndef CAMERA_VIEW_H
#define CAMERA_VIEW_H

#include <QWidget>
#include <QSocketNotifier>

#include <ros/ros.h>

#include <std_msgs/Float64.h>

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
// Class "CameraView" implements the QWidget that can be added to any QT application.
class CameraView: public Base3DView
{
    Q_OBJECT
public:
    CameraView( QWidget* parent = 0, Base3DView* copy_from = 0 );
    virtual ~CameraView();

    void setCameraPitch( int );

    void processGoalPose( const geometry_msgs::PoseStamped::ConstPtr& pose );

    std::vector<std::string> getCameraNames();

    int getDefaultCamera();

Q_SIGNALS:
    void setFullImageResolution( int );
    void setCropImageResolution( int );
    void setCameraSpeed( int );
    void setCropCameraSpeed( int );
    void unHighlight();
    void publishCropImageRequest();
    void publishFullImageRequest();
    void setInitialized();

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
    void selectionToolToggle(bool);

    virtual void defineWalkPosePressed();
    virtual void defineStepPosePressed();
    void mouseEnterEvent( QEvent* event );
    void mouseMoveEvent( QMouseEvent* event );
    void keyPressEvent( QKeyEvent* event );

    virtual bool eventFilter( QObject * o, QEvent * e );

private Q_SLOTS:
    void select( int, int, int, int );

protected:
    virtual void timerEvent(QTimerEvent *event);

private:
    int selected_area_[4];
    int last_selected_area_[4];
    std::string camera_frame_topic_;

    rviz::Display* camera_viewer_;

    rviz::Tool* selection_tool_;
    rviz::Tool* previous_tool_;

    ros::Publisher head_pitch_update_pub_;
    ros::Publisher pointcloud_request_frame_pub_;

    ros::Subscriber set_walk_goal_sub_;
    ros::Subscriber set_step_goal_sub_;

    int feed_rate_;
    int feed_resolution_;
    int area_rate_;
    int area_resolution_;

    bool setting_pose_;

    void loadCameraTopics(std::string);
    typedef struct
    {
        std::string topic_prefix;
        std::string name;
        int width;
        int height;
    } Camera;
    std::vector<Camera> camera_;
    QPushButton* close_area_button_;
    bool selection_made_;
    bool initialized_;

    bool selection_tool_enabled_;
};
}
#endif // CAMERA_VIEWER_H
