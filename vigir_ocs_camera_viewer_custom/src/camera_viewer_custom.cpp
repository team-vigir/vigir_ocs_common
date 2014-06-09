/*
 * CameraViewerCustom class implementation.
 *
 * Author: Felipe Bacim.
 *
 * Based on librviz_tutorials.
 *
 * Latest changes (12/11/2012):
 */

#include <QLabel>
#include <QVBoxLayout>
#include <QMouseEvent>
#include <QPushButton>

#include <sys/types.h> /* See NOTES */
#include <sys/socket.h>

#include "rviz/visualization_manager.h"
#include "rviz/display.h"
#include "rviz/frame_manager.h"
#include "rviz/tool_manager.h"
#include "rviz/view_manager.h"
#include "camera_viewer_custom.h"
//#include "image_display_custom.h"
#include "camera_display_custom.h"
#include "image_selection_tool_custom.h"
#include "empty_view_controller.h"
#include "rviz/viewport_mouse_event.h"

// Constructor for CameraViewerCustom.  This does most of the work of the class.

QPushButton* xButton;
bool selectionMade = false;
namespace vigir_ocs
{

CameraViewerCustom::CameraViewerCustom( QWidget* parent )
    : Base3DView( NULL, "/world", "CameraViewerCustom", parent )
    , camera_frame_topic_("")
    , feed_rate_(0)
    , feed_resolution_(4)
    , area_rate_(0)
    , area_resolution_(0)
    , setting_pose_(false)
{

    // Create a camera/image display.
    camera_viewer_ = manager_->createDisplay( "rviz/CameraDisplayCustom", "Camera image", true ); // this would use the plugin instead of manually adding the display object to the manager
    ROS_ASSERT( camera_viewer_ != NULL );

    ((rviz::CameraDisplayCustom*)camera_viewer_)->setRenderPanel( render_panel_ );
    ((rviz::CameraDisplayCustom*)camera_viewer_)->setup();

    // Set image topic
    camera_viewer_->subProp( "Image Topic" )->setValue( "/l_image_full/image_raw" );
    camera_viewer_->setEnabled(false);
    camera_viewer_->setEnabled(true);

    // Add support for selection
    selection_tool_ = manager_->getToolManager()->addTool( "rviz/ImageSelectionToolCustom" );

    manager_->getToolManager()->setCurrentTool( selection_tool_ );

    // connect the selection tool select signal to this
    QObject::connect(selection_tool_, SIGNAL(select(int,int,int,int)), this, SLOT(select(int,int,int,int)));

    robot_model_->setEnabled(false);
    ghost_robot_model_->setEnabled(false);

    QObject::connect(this, SIGNAL(setFullImageResolution(int)), camera_viewer_, SLOT(changeFullImageResolution(int)));
    QObject::connect(this, SIGNAL(setCropImageResolution(int)), camera_viewer_, SLOT(changeCropImageResolution(int)));
    QObject::connect(this, SIGNAL(setCameraSpeed(int)), camera_viewer_, SLOT(changeCameraSpeed(int)));
    QObject::connect(this, SIGNAL(setCropCameraSpeed(int)), camera_viewer_, SLOT(changeCropCameraSpeed(int)));
    QObject::connect(this, SIGNAL(publishCropImageRequest()), camera_viewer_, SLOT(publishCropImageRequest()));
    QObject::connect(this, SIGNAL(publishFullImageRequest()), camera_viewer_, SLOT(publishFullImageRequest()));

    QObject::connect(this, SIGNAL(unHighlight()), selection_tool_, SLOT(unHighlight()));

    QObject::connect(camera_viewer_, SIGNAL(updateFrameID(std::string)), this, SLOT(updateImageFrame(std::string)));

    Q_EMIT unHighlight();

    // and advertise the head pitch update function
    head_pitch_update_pub_ = nh_.advertise<std_msgs::Float64>( "/atlas/pos_cmd/neck_ry", 1, false );

    rviz::EmptyViewController* camera_controller = new rviz::EmptyViewController();
    camera_controller->initialize( render_panel_->getManager() );
    render_panel_->setViewController( camera_controller );

    xButton = new QPushButton("X",this);
    QObject::connect(xButton, SIGNAL(clicked()), this, SLOT(closeSelectedArea()));
    xButton->hide();
    QObject::connect((selection_tool_), SIGNAL(mouseHasMoved(int,int)), this, SLOT(mouseMoved(int,int)));

    Q_EMIT setMarkerScale(0.001f);

    // advertise pointcloud request
    pointcloud_request_frame_pub_ = nh_.advertise<geometry_msgs::PointStamped>( "/flor/worldmodel/ocs/dist_query_pointcloud_request_frame", 1, false );

    //reset_view_button_->setParent(0);
    delete reset_view_button_;
    reset_view_button_ = NULL;

    // subscribe to goal pose so we can add filters back
    set_walk_goal_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>( "/goal_pose_walk", 5, &CameraViewerCustom::processGoalPose, this );
    set_step_goal_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>( "/goal_pose_step", 5, &CameraViewerCustom::processGoalPose, this );

    // make sure we're still able to cancel set goal pose
    QObject::connect(render_panel_, SIGNAL(signalKeyPressEvent(QKeyEvent*)), this, SLOT(keyPressEvent(QKeyEvent*)));
}

// Destructor.
CameraViewerCustom::~CameraViewerCustom()
{
}

void CameraViewerCustom::setCameraPitch( int degrees )
{
    std_msgs::Float64 cmd;
    cmd.data = degrees*0.0174532925;
    head_pitch_update_pub_.publish(cmd);
}

void CameraViewerCustom::select( int x1, int y1, int x2, int y2 )
{
    selected_area_[0] = x1;
    selected_area_[1] = y1;
    selected_area_[2] = x2;
    selected_area_[3] = y2;

    //select_manager_->highlight(render_panel_->getViewport(), x1, y1, x2, y2 );
}

void CameraViewerCustom::changeCameraTopic( int t )
{
    std::cout << "Camera topic changed:" << t << std::endl;
    switch( t )
    {
    case 0: // Head Left
    {
        camera_viewer_->subProp( "Image Topic" )->setValue( "/l_image_full/image_raw" );
        camera_viewer_->subProp( "Image Request Topic" )->setValue( "/l_image_full/image_request" );
        camera_viewer_->subProp( "Cropped Image Topic" )->setValue( "/l_image_cropped/image_raw" );
        camera_viewer_->subProp( "Cropped Image Request Topic" )->setValue( "/l_image_cropped/image_request" );
        break;
    }
    case 1: // Head Right
    {
        camera_viewer_->subProp( "Image Topic" )->setValue( "/r_image_full/image_raw" );
        camera_viewer_->subProp( "Image Request Topic" )->setValue( "/r_image_full/image_request" );
        camera_viewer_->subProp( "Cropped Image Topic" )->setValue( "/r_image_cropped/image_raw" );
        camera_viewer_->subProp( "Cropped Image Request Topic" )->setValue( "/r_image_cropped/image_request" );
        break;
    }
    case 2: // Left hand Left
    {
        camera_viewer_->subProp( "Image Topic" )->setValue( "/lhl_image_full/image_raw" );
        camera_viewer_->subProp( "Image Request Topic" )->setValue( "/lhl_image_full/image_request" );
        camera_viewer_->subProp( "Cropped Image Topic" )->setValue( "/lhl_image_cropped/image_raw" );
        camera_viewer_->subProp( "Cropped Image Request Topic" )->setValue( "/lhl_image_cropped/image_request" );
        break;
    }
    case 3: // Left hand Right
    {
        camera_viewer_->subProp( "Image Topic" )->setValue( "/lhr_image_full/image_raw" );
        camera_viewer_->subProp( "Image Request Topic" )->setValue( "/lhr_image_full/image_request" );
        camera_viewer_->subProp( "Cropped Image Topic" )->setValue( "/lhr_image_cropped/image_raw" );
        camera_viewer_->subProp( "Cropped Image Request Topic" )->setValue( "/lhr_image_cropped/image_request" );
        break;
    }
    case 4: // Right hand Left
    {
        camera_viewer_->subProp( "Image Topic" )->setValue( "/rhl_image_full/image_raw" );
        camera_viewer_->subProp( "Image Request Topic" )->setValue( "/rhl_image_full/image_request" );
        camera_viewer_->subProp( "Cropped Image Topic" )->setValue( "/rhl_image_cropped/image_raw" );
        camera_viewer_->subProp( "Cropped Image Request Topic" )->setValue( "/rhl_image_cropped/image_request" );
        break;
    }
    case 5: // Right hand Right
    {
        camera_viewer_->subProp( "Image Topic" )->setValue( "/rhr_image_full/image_raw" );
        camera_viewer_->subProp( "Image Request Topic" )->setValue( "/rhr_image_full/image_request" );
        camera_viewer_->subProp( "Cropped Image Topic" )->setValue( "/rhr_image_cropped/image_raw" );
        camera_viewer_->subProp( "Cropped Image Request Topic" )->setValue( "/rhr_image_cropped/image_request" );
        break;
    }
    }
    //applyFeedChanges();
    closeSelectedArea();
}

void CameraViewerCustom::changeFullImageResolution( int t )
{
    std::cout << "changed camera resolution: " << t << std::endl;
    //Q_EMIT setFullImageResolution( t );
    feed_resolution_ = t;
}

void CameraViewerCustom::changeCropImageResolution( int t )
{
    std::cout << "changed crop camera resolution: " << t << std::endl;
    //Q_EMIT setCropImageResolution( t );
    area_resolution_ = t;
}

void CameraViewerCustom::changeCameraSpeed( int t )
{
    std::cout << "changed camera update speed: " << t << std::endl;
    //Q_EMIT setCameraSpeed( t );
    feed_rate_ = t;
}

void CameraViewerCustom::changeCropCameraSpeed( int t )
{
    std::cout << "changed crop camera update speed: " << t << std::endl;
    //Q_EMIT setCropCameraSpeed( t );
    area_rate_ = t;
}

void CameraViewerCustom::applyFeedChanges()
{
    Q_EMIT setFullImageResolution( feed_resolution_ );
    Q_EMIT setCameraSpeed( feed_rate_ );
    Q_EMIT publishFullImageRequest();
}

void CameraViewerCustom::applyAreaChanges()
{
    disableSelection();
    Q_EMIT setCropImageResolution( area_resolution_ );
    Q_EMIT setCropCameraSpeed( area_rate_ );
    if(selected_area_[0] != selected_area_[2] || selected_area_[1] != selected_area_[3])
    {
        for(int i = 0; i < 4; i++)
            last_selected_area_[i] = selected_area_[i];

        Q_EMIT publishCropImageRequest();
    }
    else
        selectionMade = false;
}

void CameraViewerCustom::requestSingleFeedImage()
{
    feed_rate_ = 0;
    applyFeedChanges();
}

void CameraViewerCustom::requestSingleAreaImage()
{
    area_rate_ = 0;
    applyAreaChanges();
}

void CameraViewerCustom::disableSelection()
{
    Q_EMIT unHighlight();

    ((rviz::CameraDisplayCustom*)camera_viewer_)->selectionProcessed( selected_area_[0], selected_area_[1], selected_area_[2], selected_area_[3] );
    selectionMade = true;
    int rightSide = 0;
    int topSide = 0;
    if(selected_area_[0]<selected_area_[2])
    {
        rightSide = selected_area_[2];
    }
    else
    {
        rightSide = selected_area_[0];
    }

    if(selected_area_[1]<selected_area_[3])
    {
        topSide = selected_area_[1];
    }
    else
    {
        topSide = selected_area_[3];
    }
    xButton->setGeometry(rightSide-20, topSide, 20,20);
}

void CameraViewerCustom::requestPointCloudROI()
{
    Q_EMIT unHighlight();

    float win_width = render_panel_->width();
    float win_height = render_panel_->height();

    Ogre::Ray mouseRay = this->render_panel_->getCamera()->getCameraToViewportRay(((float)(selected_area_[0]+selected_area_[2])/2.0f)/win_width, (float)((float)(selected_area_[1]+selected_area_[3])/2.0f)/win_height);
    Ogre::Vector3 direction = mouseRay.getDirection();

    geometry_msgs::PointStamped cmd;

    cmd.header.stamp = ros::Time::now();
    cmd.header.frame_id = camera_frame_topic_;
    std::cout << "Requesting new pointcloud from " << camera_frame_topic_ << std::endl;

    Ogre::Vector3 frame_pos(0,0,0);
    Ogre::Quaternion frame_quat(1,0,0,0);

    transform( frame_pos, frame_quat, manager_->getFixedFrame().toUtf8().constData(), camera_frame_topic_.c_str() );

    // convert vision (Z-forward) frame to ogre frame (Z-out)
    //orientation = orientation * Ogre::Quaternion( Ogre::Degree( 180 ), Ogre::Vector3::UNIT_X );

    direction = (frame_quat * direction);// + frame_pos;

    cmd.point.x = direction.x;
    cmd.point.y = direction.y;
    cmd.point.z = direction.z;

    pointcloud_request_frame_pub_.publish(cmd);
}

void CameraViewerCustom::changeAlpha(int newAlpha)
{
    ((rviz::CameraDisplayCustom*)camera_viewer_)->setAlpha(1.0f-(newAlpha/100.0f));
}

void CameraViewerCustom::mouseMoved(int newX, int newY)
{
    if(((newX<last_selected_area_[0] && newX>last_selected_area_[2]) ||
       (newX>last_selected_area_[0] && newX<last_selected_area_[2])) &&
       ((newY<last_selected_area_[1] && newY>last_selected_area_[3]) ||
       (newY>last_selected_area_[1] && newY<last_selected_area_[3])) && selectionMade)
    {
        xButton->show();
    }
    else
    {
        xButton->hide();
    }
}

void CameraViewerCustom::closeSelectedArea()
{
    //std::cout<<"This gets hit"<<std::endl;
    ((rviz::CameraDisplayCustom*)camera_viewer_)->closeSelected();
    xButton->hide();
    selectionMade = false;
}

void CameraViewerCustom::updateImageFrame(std::string frame)
{
    camera_frame_topic_ = frame;
}

void CameraViewerCustom::defineWalkPosePressed()
{
    //ROS_ERROR("vector pressed in map");
    //set_goal_tool_->getPropertyContainer()->subProp( "Topic" )->setValue( "/goal_pose_walk" );
    manager_->getToolManager()->setCurrentTool( set_walk_goal_tool_ );
    setting_pose_ = true;
}

void CameraViewerCustom::defineStepPosePressed()
{
    //ROS_ERROR("vector pressed in map");
    //set_goal_tool_->getPropertyContainer()->subProp( "Topic" )->setValue( "/goal_pose_step" );
    manager_->getToolManager()->setCurrentTool( set_step_goal_tool_ );
    setting_pose_ = true;
}

void CameraViewerCustom::processGoalPose(const geometry_msgs::PoseStamped::ConstPtr &pose)
{
    //ROS_ERROR("goal processed in map");
    manager_->getToolManager()->setCurrentTool( selection_tool_ );
    setting_pose_ = false;
}

void CameraViewerCustom::keyPressEvent( QKeyEvent* event )
{
    // block events and change to camera tool
    manager_->getToolManager()->setCurrentTool( selection_tool_ );
    setting_pose_ = false;
}
}

