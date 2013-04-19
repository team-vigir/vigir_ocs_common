/*
 * Base3DView class implementation.
 *
 * Author: Felipe Bacim.
 *
 * Based on librviz_tutorials.
 *
 * Latest changes (12/08/2012):
 * - added support for joint manipulation?
 */

#include <QLabel>
#include <QVBoxLayout>
#include <QHBoxLayout>

#include "rviz/visualization_manager.h"
//#include "rviz/render_panel.h"
#include <render_panel_custom.h>
#include "rviz/display.h"
#include "rviz/frame_manager.h"
#include "rviz/tool_manager.h"
#include <selection_handler.h>
#include <template_display_custom.h>
#include "base_3d_view.h"

#include "flor_ocs_msgs/OCSTemplateAdd.h"
#include "flor_ocs_msgs/OCSWaypointAdd.h"
#include "flor_perception_msgs/EnvironmentRegionRequest.h"

namespace vigir_ocs
{
// Constructor for Base3DView.  This does most of the work of the class.
Base3DView::Base3DView( std::string base_frame, QWidget* parent )
    : QWidget( parent )
{
    base_frame_ = base_frame;

    // Create a new label for this widget.
    //QLabel* robot_model_label = new QLabel( "rviz/RobotModel" );

    // Construct and lay out render panel.
    render_panel_ = new rviz::RenderPanelCustom();
    QVBoxLayout* main_layout = new QVBoxLayout;
    //main_layout->addWidget( robot_model_label );
    main_layout->addWidget( render_panel_ );

    // Set the top-level layout for this MyViz widget.
    setLayout( main_layout );

    // Make signal/slot connections.
    //connect( collision_checkbox, SIGNAL( valueChanged( int )), this, SLOT( setCollision( bool )));

    // Next we initialize the main RViz classes.
    //
    // The VisualizationManager is the container for Display objects,
    // holds the main Ogre scene, holds the ViewController, etc.  It is
    // very central and we will probably need one in every usage of
    // librviz.
    manager_ = new rviz::VisualizationManager( render_panel_ );
    render_panel_->initialize( manager_->getSceneManager(), manager_ );
    manager_->initialize();
    manager_->startUpdate();

    // Create a RobotModel display.
    robot_model_ = manager_->createDisplay( "rviz/RobotModel", "Robot model", true );
    ROS_ASSERT( robot_model_ != NULL );

    // Set topic that will be used as 0,0,0 -> reference for all the other transforms
    // IMPORTANT: WITHOUT THIS, ALL THE DIFFERENT PARTS OF THE ROBOT MODEL WILL BE DISPLAYED AT 0,0,0
    manager_->setFixedFrame(base_frame_.c_str());

    // Add support for interactive markers
    interactive_markers_tool_ = manager_->getToolManager()->addTool( "rviz/Interact" );
    // Add support for selection
    selection_tool_ = manager_->getToolManager()->addTool( "rviz/Select" );
    // Add support for camera movement
    move_camera_tool_ = manager_->getToolManager()->addTool( "rviz/MoveCamera" );
    // Add support for goal specification/vector navigation
    set_goal_tool_ = manager_->getToolManager()->addTool( "rviz/SetGoal" );

    // Add interactive markers Stefan's markers and IK implementation
    interactive_marker_robot_[0] = manager_->createDisplay( "rviz/InteractiveMarkers", "Interactive marker 1", true );
    interactive_marker_robot_[0]->subProp( "Update Topic" )->setValue( "/l_arm_pose_marker/update" );
    interactive_marker_robot_[1] = manager_->createDisplay( "rviz/InteractiveMarkers", "Interactive marker 2", true );
    interactive_marker_robot_[1]->subProp( "Update Topic" )->setValue( "/l_leg_pose_marker/update" );
    interactive_marker_robot_[2] = manager_->createDisplay( "rviz/InteractiveMarkers", "Interactive marker 3", true );
    interactive_marker_robot_[2]->subProp( "Update Topic" )->setValue( "/r_arm_pose_marker/update" );
    interactive_marker_robot_[3] = manager_->createDisplay( "rviz/InteractiveMarkers", "Interactive marker 4", true );
    interactive_marker_robot_[3]->subProp( "Update Topic" )->setValue( "/r_leg_pose_marker/update" );

    // Make the move camera tool the currently selected one
    manager_->getToolManager()->setCurrentTool( move_camera_tool_ );

    // Create a LaserScan display.
    laser_scan_ = manager_->createDisplay( "rviz/LaserScan", "Laser Scan", false );
    ROS_ASSERT( laser_scan_ != NULL );

    laser_scan_->subProp( "Topic" )->setValue( "/multisense_sl/laser/scan" );
    laser_scan_->subProp( "Size (m)" )->setValue( 0.1 );
    laser_scan_->subProp( "Decay Time" )->setValue( 1 );

    // Create a MarkerArray display.
    //marker_array_ = manager_->createDisplay( "rviz/MarkerArray", "MarkerArray", true );
    octomap_ = manager_->createDisplay( "rviz/OctomapDisplayCustom", "Octomap", true );
    ROS_ASSERT( octomap_ != NULL );

    octomap_->subProp( "Marker Topic" )->setValue( "/worldmodel_main/occupied_cells_vis_array" );

    // Create a point cloud display.
    stereo_point_cloud_viewer_ = manager_->createDisplay( "rviz/PointCloud2", "Point Cloud", false );
    ROS_ASSERT( stereo_point_cloud_viewer_ != NULL );
    stereo_point_cloud_viewer_->subProp( "Style" )->setValue( "Points" );
    stereo_point_cloud_viewer_->subProp( "Topic" )->setValue( "/multisense_sl/camera/points2" );
    stereo_point_cloud_viewer_->subProp( "Size (Pixels)" )->setValue( 3 );

    lidar_point_cloud_viewer_ = manager_->createDisplay( "rviz/PointCloud2", "Point Cloud", true );
    ROS_ASSERT( lidar_point_cloud_viewer_ != NULL );
    lidar_point_cloud_viewer_->subProp( "Style" )->setValue( "Points" );
    lidar_point_cloud_viewer_->subProp( "Topic" )->setValue( "/scan_cloud_filtered" );
    lidar_point_cloud_viewer_->subProp( "Size (Pixels)" )->setValue( 3 );

    // Create a template display to display all templates listed by the template nodelet
    template_display_ = manager_->createDisplay( "rviz/TemplateDisplayCustom", "Template Display", true );
    ((rviz::TemplateDisplayCustom*)template_display_)->setVisualizationManager(manager_);

    // Create a display for 3D selection
    selection_3d_display_ = manager_->createDisplay( "rviz/Selection3DDisplayCustom", "3D Selection Display", true );
    
    // Create a display for waypoints
    waypoints_display_ = manager_->createDisplay( "rviz/PathDisplayCustom", "Path Display", true );
    waypoints_display_->subProp( "Topic" )->setValue( "/waypoint/list" );

    // Create another display for waypoints, this time the ones that have already been achieved
    achieved_waypoints_display_ = manager_->createDisplay( "rviz/PathDisplayCustom", "Path Display", true );
    achieved_waypoints_display_->subProp( "Topic" )->setValue( "/waypoint/achieved_list" );
    achieved_waypoints_display_->subProp( "Color" )->setValue( QColor( 150, 150, 255 ) );

    // connect the 3d selection tool to its display
    //QObject::connect(selection_3d_tool_, SIGNAL(select(int,int,int,int)), selection_3d_display_, SLOT(createMarker(int,int,int,int)));
    QObject::connect(this, SIGNAL(setRenderPanel(rviz::RenderPanel*)), selection_3d_display_, SLOT(setRenderPanel(rviz::RenderPanel*)));
    QObject::connect(selection_3d_display_, SIGNAL(newSelection(Ogre::Vector3)), this, SLOT(newSelection(Ogre::Vector3)));

    Q_EMIT setRenderPanel(this->render_panel_);
    
    // handles selections without rviz::tool
    selection_handler_ = new vigir_ocs::SelectionHandler();
    QObject::connect(render_panel_, SIGNAL(signalMousePressEvent(QMouseEvent*)), selection_handler_, SLOT(mousePressEvent(QMouseEvent*)));
    QObject::connect(selection_handler_, SIGNAL(select(int,int)), selection_3d_display_, SLOT(createMarker(int,int)));
    QObject::connect(selection_handler_, SIGNAL(selectROI(int,int)), selection_3d_display_, SLOT(createROISelection(int,int)));

    // create a publisher to add templates
    template_add_pub_   = n_.advertise<flor_ocs_msgs::OCSTemplateAdd>( "/template/add", 1, false );
    
	// create a publisher to add waypoints
    waypoint_add_pub_   = n_.advertise<flor_ocs_msgs::OCSWaypointAdd>( "/waypoint/add", 1, false );

    selection_position_ = Ogre::Vector3(0,0,0);

    // this will all go to a separate nodelet that processes octomaps

    //octomap_roi_ = manager_->createDisplay( "rviz/OctomapDisplayCustom", "Octomap", true );
    //ROS_ASSERT( octomap_roi_ != NULL );

    //octomap_roi_->subProp( "Marker Topic" )->setValue( "/worldmodel_main/occupied_cells_vis_array" );

    //octomap_roi_pub_ = n_.advertise<flor_perception_msgs::EnvironmentRegionRequest>( "/waypoint/add", 1, false );
}

// Destructor.
Base3DView::~Base3DView()
{
    delete manager_;
}

void Base3DView::robotModelToggled( bool selected )
{
    robot_model_->setEnabled( selected );
}

void Base3DView::lidarPointCloudToggled( bool selected )
{
    lidar_point_cloud_viewer_->setEnabled( selected );
}

void Base3DView::stereoPointCloudToggled( bool selected )
{
    stereo_point_cloud_viewer_->setEnabled( selected );
}

void Base3DView::laserScanToggled( bool selected )
{
    laser_scan_->setEnabled( selected );
}

void Base3DView::markerArrayToggled( bool selected )
{
    octomap_->setEnabled( selected );
}

void Base3DView::cameraToggled( bool selected )
{
    if(selected)
        manager_->getToolManager()->setCurrentTool( move_camera_tool_ );
}

void Base3DView::selectToggled( bool selected )
{
    if(selected)
        manager_->getToolManager()->setCurrentTool( selection_tool_ );
}

void Base3DView::select3DToggled( bool selected )
{
    //if(selected)
    //    manager_->getToolManager()->setCurrentTool( selection_3d_tool_ );
}

void Base3DView::markerRobotToggled( bool selected )
{
    if(selected)
    {
        manager_->getToolManager()->setCurrentTool( interactive_markers_tool_ );

        // enable robot IK markers
        for( int i = 0; i < 4; i++ )
        {
            interactive_marker_robot_[i]->setEnabled( true );
        }
        // disable template marker
        //interactive_marker_template_->setEnabled( false );
    }
}

void Base3DView::markerTemplateToggled( bool selected )
{
    if(selected)
    {
        manager_->getToolManager()->setCurrentTool( interactive_markers_tool_ );

        // disable robot IK markers
        for( int i = 0; i < 4; i++ )
        {
            interactive_marker_robot_[i]->setEnabled( false );
        }
        // enable template markers
        //interactive_marker_template_->setEnabled( true );
    }
}

void Base3DView::vectorToggled( bool selected )
{
    if(selected)
        manager_->getToolManager()->setCurrentTool( set_goal_tool_ );
}

void Base3DView::newSelection( Ogre::Vector3 position )
{
    selection_position_ = position;
}

void Base3DView::transform(const std::string& target_frame, geometry_msgs::PoseStamped& pose)
{

    tf::Quaternion bt_orientation(pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w);
    tf::Vector3 bt_position(pose.pose.position.x,pose.pose.position.y,pose.pose.position.z);

    tf::Stamped<tf::Pose> pose_in(tf::Transform(bt_orientation,bt_position), ros::Time(), pose.header.frame_id);
    tf::Stamped<tf::Pose> pose_out;

    try
    {
        manager_->getFrameManager()->getTFClient()->transformPose( target_frame.c_str(), pose_in, pose_out );
    }
    catch(tf::TransformException& e)
    {
        ROS_DEBUG("Error transforming from frame '%s' to frame '%s': %s", pose.header.frame_id.c_str(), target_frame.c_str(), e.what());
        return;
    }

    bt_position = pose_out.getOrigin();
    bt_orientation = pose_out.getRotation();

    pose.pose.position.x = bt_position.x();
    pose.pose.position.y = bt_position.y();
    pose.pose.position.z = bt_position.z();
    pose.pose.orientation.x = bt_orientation.x();
    pose.pose.orientation.y = bt_orientation.y();
    pose.pose.orientation.z = bt_orientation.z();
    pose.pose.orientation.w = bt_orientation.w();

    pose.header.frame_id = target_frame;
}

void Base3DView::insertTemplate( QString path )
{
    std::cout << "adding template" << std::endl;

    flor_ocs_msgs::OCSTemplateAdd cmd;
    geometry_msgs::PoseStamped pose;

    cmd.template_path = path.toStdString();

    pose.pose.position.x = selection_position_.x;
    pose.pose.position.y = selection_position_.y;
    pose.pose.position.z = selection_position_.z;
    pose.pose.orientation.x = 0;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = 0;
    pose.pose.orientation.w = 1;

    pose.header.frame_id = base_frame_;

    transform("/world",pose);

    cmd.pose = pose;

    // publish complete list of templates and poses
    template_add_pub_.publish( cmd );
}

void Base3DView::insertWaypoint()
{
    std::cout << "adding waypoint" << std::endl;

    flor_ocs_msgs::OCSWaypointAdd cmd;
    geometry_msgs::PoseStamped pose;

    pose.pose.position.x = selection_position_.x;
    pose.pose.position.y = selection_position_.y;
    pose.pose.position.z = selection_position_.z;
    pose.pose.orientation.x = 0;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = 0;
    pose.pose.orientation.w = 1;

    pose.header.frame_id = base_frame_;

    transform("/world",pose);

    cmd.pose = pose;

    // publish complete list of templates and poses
    waypoint_add_pub_.publish( cmd );
}

}
