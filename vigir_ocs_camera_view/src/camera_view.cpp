/*
 * CameraView class implementation.
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
#include "camera_view.h"
//#include "image_display_custom.h"
#include "camera_display_custom.h"
#include "image_selection_tool_custom.h"
#include "empty_view_controller.h"
#include "rviz/viewport_mouse_event.h"

// Constructor for CameraView.  This does most of the work of the class.

namespace vigir_ocs
{

CameraView::CameraView( QWidget* parent, Base3DView* copy_from )
    : Base3DView( copy_from, "/world", "CameraView", parent )
    , camera_frame_topic_("")
    , feed_rate_(0)
    , feed_resolution_(4)
    , area_rate_(0)
    , area_resolution_(0)
    , setting_pose_(false)
    , selection_made_(false)
    , initialized_(false)
    , selection_tool_enabled_( true )
{
    this->setMouseTracking(true);

    // Load camera topics from parameter server
    loadCameraTopics("/flor/ocs/camera/atlas");
    loadCameraTopics("/flor/ocs/camera/left_hand");
    loadCameraTopics("/flor/ocs/camera/right_hand");

    // Make sure we have camera topics
    if(!camera_.size())
    {
        ROS_ERROR("Need to load camera topics in the parameter server. Try using:\n\troslaunch vigir_ocs vigir_ocs.launch\t\t- for the OCS UI\n\troslaunch vigir_ocs vigir_ocs_camera.launch\t- for the camera widgets standalone");
        exit(1);
    }

    // Create a camera/image display.
    camera_viewer_ = manager_->createDisplay( "rviz/CameraDisplayCustom", "Camera image", true ); // this would use the plugin instead of manually adding the display object to the manager
    ROS_ASSERT( camera_viewer_ != NULL );

    ((rviz::CameraDisplayCustom*)camera_viewer_)->setRenderPanel( render_panel_ );
    ((rviz::CameraDisplayCustom*)camera_viewer_)->setup();
    ((rviz::CameraDisplayCustom*)camera_viewer_)->setViewID(view_id_);

    // Add support for selection
    selection_tool_ = manager_->getToolManager()->addTool( "rviz/ImageSelectionToolCustom" );

    std::bitset<32> y(camera_viewer_->getVisibilityBits());
    std::cout << "camera vis bit: " << y << std::endl;
    uint32_t vis_bit = 0xFFFFFFFF;
    for(int i = 0; i < 4; i++)
    {
        std::bitset<32> a((uint32_t)pow(2,i));
        std::cout << "\tsetting: " << a << std::endl;

        if(view_id_ == i)
            vis_bit |= (uint32_t)pow(2,i); // send the viewport visilibity bit to the display and display will know if it should render there or not.
        else
            vis_bit &= ~(uint32_t)pow(2,i);
    }

    std::bitset<32> x(vis_bit);
    std::cout << "tool vis bit:   " << x << std::endl;
    ((rviz::ImageSelectionToolCustom*)selection_tool_)->setVisibilityBits(vis_bit);
    ((rviz::CameraDisplayCustom*)camera_viewer_)->setVisibilityBits(vis_bit);

    // connect the selection tool select signal to this
    QObject::connect(selection_tool_, SIGNAL(select(int,int,int,int)), this, SLOT(select(int,int,int,int)));
    manager_->getToolManager()->setCurrentTool( selection_tool_ );

    if(!copy_from)
    {
        robot_model_->setEnabled(false);
        ghost_robot_model_->setEnabled(false);
    }

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
    //head_pitch_update_pub_ = nh_.advertise<std_msgs::Float64>( "/atlas/pos_cmd/neck_ry", 1, false );
    head_pitch_update_traj_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory > ("/flor/neck_controller/trajectory",1,false);

    rviz::EmptyViewController* camera_controller = new rviz::EmptyViewController();
    camera_controller->initialize( render_panel_->getManager() );
    render_panel_->setViewController( camera_controller );

    close_area_button_ = new QPushButton("X",this);
    QObject::connect(close_area_button_, SIGNAL(clicked()), this, SLOT(closeSelectedArea()));
    close_area_button_->hide();
    close_area_button_->setStyleSheet(QString("QPushButton  { ") +
                                      " background-color: qlineargradient(spread:pad, x1:0, y1:0, x2:0, y2:1, stop:0 rgba(240, 240, 240, 255), stop:1 rgba(222, 222, 222, 255));" +
                                      " border-style: solid;" +
                                      " border-width: 1px;" +
                                      " border-radius: 1px;" +
                                      " border-color: gray;" +
                                      " padding: 0px;" +
                                      " font: \"Ubuntu\";"
                                      " font-size: 9px;"
                                      "}" +
                                      "QPushButton:pressed  {" +
                                      " padding-top:1px; padding-left:1px;" +
                                      " background-color: rgb(180,180,180);" +
                                      " border-style: inset;" +
                                      "}");
    QObject::connect((selection_tool_), SIGNAL(mouseHasMoved(int,int)), this, SLOT(mouseMoved(int,int)));

    // set scale of the selection marker
    selection_3d_display_->subProp("Scale")->setValue(0.001f);
    selection_3d_display_->subProp("Constant Visual Size")->setValue(false);

    // advertise pointcloud request
    pointcloud_request_frame_pub_ = nh_.advertise<geometry_msgs::PointStamped>( "/flor/worldmodel/ocs/dist_query_pointcloud_request_frame", 1, false );

    //reset_view_button_->setParent(0);
    //delete reset_view_button_;
    //reset_view_button_ = NULL;
    reset_view_button_->hide();

    // make sure we're still able to cancel set goal pose
    QObject::connect(render_panel_, SIGNAL(signalKeyPressEvent(QKeyEvent*)), this, SLOT(keyPressEvent(QKeyEvent*)));

    QObject::connect(render_panel_, SIGNAL(signalMouseEnterEvent(QEvent*)), this, SLOT(mouseEnterEvent(QEvent*)));
    QObject::connect(render_panel_, SIGNAL(signalMouseMoveEvent(QMouseEvent*)), this, SLOT(mouseMoveEvent(QMouseEvent*)));

    Q_FOREACH( QWidget* sp, findChildren<QWidget*>() ) {
        sp->installEventFilter( this );
        sp->setMouseTracking( true );
    }

    // Set image topic
    int default_cam = getDefaultCamera();
    camera_viewer_->subProp( "Image Topic" )->setValue( (camera_[default_cam].topic_prefix+"_full/image_raw").c_str() );
    camera_viewer_->subProp( "Image Request Topic" )->setValue( (camera_[default_cam].topic_prefix+"_full/image_request").c_str() );
    camera_viewer_->subProp( "Cropped Image Topic" )->setValue( (camera_[default_cam].topic_prefix+"_cropped/image_raw").c_str() );
    camera_viewer_->subProp( "Cropped Image Request Topic" )->setValue( (camera_[default_cam].topic_prefix+"_cropped/image_request").c_str() );
    camera_viewer_->setEnabled(false);
    camera_viewer_->setEnabled(true);

    selected_area_[0] = 0;
    selected_area_[1] = 0;
    selected_area_[2] = 0;
    selected_area_[3] = 0;

    previous_tool_ = selection_tool_;
}

// Destructor.
CameraView::~CameraView()
{
}

void CameraView::timerEvent(QTimerEvent *event)
{
    // call the base3dview version of the timerevent
    Base3DView::timerEvent(event);

    if(!initialized_ && ((rviz::CameraDisplayCustom*)camera_viewer_)->hasRenderedOnce())
    {
        std::cout << "initialized camera" << std::endl;
        initialized_ = true;
        Q_EMIT setInitialized();
    }
}

void CameraView::loadCameraTopics(std::string prefix)
{
    // get cameras parameters and setup local structure
    XmlRpc::XmlRpcValue camera_topic_prefix, camera_name, camera_width, camera_height;

    nh_.getParam(prefix+"/topic_prefix", camera_topic_prefix);
    nh_.getParam(prefix+"/name", camera_name);
    nh_.getParam(prefix+"/width", camera_width);
    nh_.getParam(prefix+"/height", camera_height);

    for(int i = 0; i < camera_topic_prefix.size(); i++)
    {
        Camera c;
        c.topic_prefix = static_cast<std::string>(camera_topic_prefix[i]);
        c.name = static_cast<std::string>(camera_name[i]);
        c.width = static_cast<int>(camera_width[i]);
        c.height = static_cast<int>(camera_height[i]);
        camera_.push_back(c);
    }
}

int CameraView::getDefaultCamera()
{
    int default_cam;
    if(view_id_ < camera_.size())
        default_cam = view_id_;
    else
        default_cam = camera_.size()-1;
    return default_cam;
}

void CameraView::setCurrentCameraPitch( int degrees )
{
    m_current_pitch = degrees;
}

void CameraView::setCameraPitch( int degrees )
{
	trajectory_msgs::JointTrajectory trajectory;

    trajectory.joint_names.push_back("neck_ry");

	trajectory.header.stamp = ros::Time::now();

	trajectory.points.push_back( trajectory_msgs::JointTrajectoryPoint() );
	trajectory.points.push_back( trajectory_msgs::JointTrajectoryPoint() );

	trajectory.points[0].positions.push_back( ((double)m_current_pitch)*0.0174532925 ); // current
	trajectory.points[1].positions.push_back( ((double)degrees)*0.0174532925); // next

	trajectory.points[0].time_from_start = ros::Duration(0.0); 
    trajectory.points[1].time_from_start = ros::Duration(0.25+fabs(((double)m_current_pitch)-((double)degrees))/(65.0+40.0)*1.5); //range from 0-> 3

	head_pitch_update_traj_pub_.publish( trajectory );
}

void CameraView::select( int x1, int y1, int x2, int y2 )
{
    selected_area_[0] = std::min(x1,x2);
    selected_area_[1] = std::min(y1,y2);
    selected_area_[2] = std::max(x1,x2);
    selected_area_[3] = std::max(y1,y2);

    //select_manager_->highlight(render_panel_->getViewport(), x1, y1, x2, y2 );
}

void CameraView::changeCameraTopic( int t )
{
    // check if it's within bounds
    if(t < 0 || t >= camera_.size())
        return;

    std::cout << "Camera topic changed:" << t << std::endl;
    closeSelectedArea();
    ((rviz::CameraDisplayCustom*)camera_viewer_)->closeFull();


    camera_viewer_->subProp( "Image Topic" )->setValue( (camera_[t].topic_prefix+"_full/image_raw").c_str() );
    camera_viewer_->subProp( "Image Request Topic" )->setValue( (camera_[t].topic_prefix+"_full/image_request").c_str() );
    camera_viewer_->subProp( "Cropped Image Topic" )->setValue( (camera_[t].topic_prefix+"_cropped/image_raw").c_str() );
    camera_viewer_->subProp( "Cropped Image Request Topic" )->setValue( (camera_[t].topic_prefix+"_cropped/image_request").c_str() );

    //applyFeedChanges();
}

void CameraView::changeFullImageResolution( int t )
{
    std::cout << "changed camera resolution: " << t << std::endl;
    //Q_EMIT setFullImageResolution( t );
    feed_resolution_ = t;
}

void CameraView::changeCropImageResolution( int t )
{
    std::cout << "changed crop camera resolution: " << t << std::endl;
    //Q_EMIT setCropImageResolution( t );
    area_resolution_ = t;
}

void CameraView::changeCameraSpeed( int t )
{
    std::cout << "changed camera update speed: " << t << std::endl;
    //Q_EMIT setCameraSpeed( t );
    feed_rate_ = t;
}

void CameraView::changeCropCameraSpeed( int t )
{
    std::cout << "changed crop camera update speed: " << t << std::endl;
    //Q_EMIT setCropCameraSpeed( t );
    area_rate_ = t;
}

void CameraView::applyFeedChanges()
{
    Q_EMIT setFullImageResolution( feed_resolution_ );
    Q_EMIT setCameraSpeed( feed_rate_ );
    Q_EMIT publishFullImageRequest();
}

void CameraView::applyAreaChanges()
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
        selection_made_ = false;
}

void CameraView::requestSingleFeedImage()
{
    feed_rate_ = 0;
    applyFeedChanges();
}

void CameraView::requestSingleAreaImage()
{
    area_rate_ = 0;
    applyAreaChanges();
}

void CameraView::disableSelection()
{
    Q_EMIT unHighlight();

    ((rviz::CameraDisplayCustom*)camera_viewer_)->selectionProcessed( selected_area_[0], selected_area_[1], selected_area_[2], selected_area_[3] );
    selection_made_ = true;
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
    close_area_button_->setGeometry(rightSide-12, topSide, 12, 12);
}

void CameraView::requestPointCloudROI()
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

void CameraView::changeAlpha(int newAlpha)
{
    ((rviz::CameraDisplayCustom*)camera_viewer_)->setAlpha(1.0f-(newAlpha/100.0f));
}

void CameraView::mouseMoved(int newX, int newY)
{
    if(((newX<last_selected_area_[0] && newX>last_selected_area_[2]) ||
       (newX>last_selected_area_[0] && newX<last_selected_area_[2])) &&
       ((newY<last_selected_area_[1] && newY>last_selected_area_[3]) ||
       (newY>last_selected_area_[1] && newY<last_selected_area_[3])) && selection_made_)
    {
        close_area_button_->show();
    }
    else
    {
        close_area_button_->hide();
    }
}

void CameraView::closeSelectedArea()
{
    //std::cout<<"This gets hit"<<std::endl;
    ((rviz::CameraDisplayCustom*)camera_viewer_)->closeSelected();
    close_area_button_->hide();
    selection_made_ = false;
}

void CameraView::updateImageFrame(std::string frame)
{
    camera_frame_topic_ = frame;
}

void CameraView::defineFootstepGoal()
{
    //ROS_ERROR("vector pressed in map");
    previous_tool_ = manager_->getToolManager()->getCurrentTool();
    manager_->getToolManager()->setCurrentTool( set_goal_tool_ );
    setting_pose_ = true;
}

void CameraView::processGoalPose(const geometry_msgs::PoseStamped::ConstPtr &pose)
{
    Base3DView::processGoalPose( pose );
    //ROS_ERROR("goal processed in map");
    manager_->getToolManager()->setCurrentTool( previous_tool_ );
    setting_pose_ = false;
}

std::vector<std::string> CameraView::getCameraNames()
{
    std::vector<std::string> names;
    for(int i = 0; i < camera_.size(); i++)
        names.push_back(camera_[i].name);
    return names;
}

void CameraView::selectionToolToggle(bool enable)
{
    selection_tool_enabled_ = enable;
}

void CameraView::mouseEnterEvent( QEvent* event )
{
    //std::cout << "mouse enter " << view_id_ << std::endl;
    if(selection_tool_enabled_ && manager_->getToolManager()->getCurrentTool() != selection_tool_)
        manager_->getToolManager()->setCurrentTool( selection_tool_ );
}

void CameraView::mouseMoveEvent( QMouseEvent* event )
{
    //std::cout << "mouse move " << view_id_ << std::endl;
    if(selection_tool_enabled_ && manager_->getToolManager()->getCurrentTool() != selection_tool_)
        manager_->getToolManager()->setCurrentTool( selection_tool_ );
}

bool CameraView::eventFilter( QObject * o, QEvent * e )
{
    if ( e->type() == QEvent::Enter )
        mouseEnterEvent(e);
    else if ( e->type() == QEvent::MouseMove )
        mouseMoveEvent((QMouseEvent*)e);
    return Base3DView::eventFilter( o, e );
}

void CameraView::keyPressEvent( QKeyEvent* event )
{
    // block events and change to camera tool
    manager_->getToolManager()->setCurrentTool( selection_tool_ );
    setting_pose_ = false;
}

void CameraView::processHotkeyRelayMessage(const flor_ocs_msgs::OCSHotkeyRelay::ConstPtr &msg)
{
    Base3DView::processHotkeyRelayMessage(msg);
    if(msg->relay_code == flor_ocs_msgs::OCSHotkeyRelay::CLEAR_IMAGE_SELECTED)
    {
        closeSelectedArea();
    }
}
}

