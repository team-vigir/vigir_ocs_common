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
#include <QPoint>
#include <QMenu>

#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

#include "rviz/visualization_manager.h"
//#include "rviz/render_panel.h"
#include <render_panel_custom.h>
#include "rviz/display.h"
#include "rviz/frame_manager.h"
#include "rviz/tool_manager.h"
#include <template_display_custom.h>
#include "map_display_custom.h"
#include "base_3d_view.h"

#include "flor_ocs_msgs/OCSTemplateAdd.h"
#include "flor_ocs_msgs/OCSWaypointAdd.h"
#include "flor_perception_msgs/EnvironmentRegionRequest.h"
#include "flor_planning_msgs/TargetConfigIkRequest.h"

namespace vigir_ocs
{
// Constructor for Base3DView.  This does most of the work of the class.
Base3DView::Base3DView( std::string base_frame, QWidget* parent )
    : QWidget( parent )
    , base_frame_(base_frame)
    , selected_(false)
    , update_markers_(false)
    , snap_ghost_to_robot_(true)
{
    // Construct and lay out render panel.
    render_panel_ = new rviz::RenderPanelCustom();
    ((rviz::RenderPanelCustom*)render_panel_)->setEventFilters(rviz::RenderPanelCustom::MOUSE_PRESS_EVENT,false,Qt::NoModifier,Qt::RightButton);
    ((rviz::RenderPanelCustom*)render_panel_)->setEventFilters(rviz::RenderPanelCustom::MOUSE_RELEASE_EVENT,false,Qt::NoModifier,Qt::RightButton);
    ((rviz::RenderPanelCustom*)render_panel_)->setEventFilters(rviz::RenderPanelCustom::MOUSE_MOVE_EVENT,false,Qt::NoModifier,Qt::RightButton);

    QVBoxLayout* main_layout = new QVBoxLayout;
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

    // Set topic that will be used as 0,0,0 -> reference for all the other transforms
    // IMPORTANT: WITHOUT THIS, ALL THE DIFFERENT PARTS OF THE ROBOT MODEL WILL BE DISPLAYED AT 0,0,0
    manager_->setFixedFrame(base_frame_.c_str());

    manager_->initialize();
    manager_->startUpdate();

    // First remove all existin tools
    manager_->getToolManager()->removeAll();
    // Add support for interactive markers
    interactive_markers_tool_ = manager_->getToolManager()->addTool( "rviz/Interact" );
    // Add support for selection
    //selection_tool_ = manager_->getToolManager()->addTool( "rviz/Select" );
    // Add support for camera movement
    move_camera_tool_ = manager_->getToolManager()->addTool( "rviz/MoveCamera" );
    // Add support for goal specification/vector navigation
    set_goal_tool_ = manager_->getToolManager()->addTool( "rviz/SetGoal" );

    // Create a LaserScan display.
    laser_scan_ = manager_->createDisplay( "rviz/LaserScan", "Laser Scan", false );
    ROS_ASSERT( laser_scan_ != NULL );
    laser_scan_->subProp( "Topic" )->setValue( "/multisense_sl/laser/scan" );
    laser_scan_->subProp( "Size (m)" )->setValue( 0.1 );
    laser_scan_->subProp( "Decay Time" )->setValue( 1 );
    laser_scan_->subProp( "Selectable" )->setValue( false );

    // Create a MarkerArray display.
    octomap_ = manager_->createDisplay( "rviz/OctomapDisplayCustom", "Octomap", false );
    ROS_ASSERT( octomap_ != NULL );

    octomap_->subProp( "Marker Topic" )->setValue( "/worldmodel_main/occupied_cells_vis_array" );

    // Create a point cloud display.
    stereo_point_cloud_viewer_ = manager_->createDisplay( "rviz/PointCloud2", "Point Cloud", false );
    ROS_ASSERT( stereo_point_cloud_viewer_ != NULL );
    stereo_point_cloud_viewer_->subProp( "Style" )->setValue( "Points" );
    stereo_point_cloud_viewer_->subProp( "Topic" )->setValue( "/multisense_sl/camera/points2" );
    stereo_point_cloud_viewer_->subProp( "Size (Pixels)" )->setValue( 3 );
    stereo_point_cloud_viewer_->subProp( "Selectable" )->setValue( false );

    lidar_point_cloud_viewer_ = manager_->createDisplay( "rviz/PointCloud2", "Point Cloud", false );
    ROS_ASSERT( lidar_point_cloud_viewer_ != NULL );
    lidar_point_cloud_viewer_->subProp( "Style" )->setValue( "Points" );
    lidar_point_cloud_viewer_->subProp( "Topic" )->setValue( "/worldmodel_main/pointcloud_vis" );
    lidar_point_cloud_viewer_->subProp( "Size (Pixels)" )->setValue( 3 );
    lidar_point_cloud_viewer_->subProp( "Selectable" )->setValue( false );

    // point cloud request
    point_cloud_request_viewer_ = manager_->createDisplay( "rviz/PointCloud2", "Point Cloud", true );
    point_cloud_request_viewer_->subProp( "Style" )->setValue( "Points" );
    point_cloud_request_viewer_->subProp( "Topic" )->setValue( "/flor/worldmodel/ocs/dist_query_pointcloud_result" );
    point_cloud_request_viewer_->subProp( "Size (Pixels)" )->setValue( 3 );
    point_cloud_request_viewer_->subProp( "Color Transformer" )->setValue( "AxisColor" );
    point_cloud_request_viewer_->subProp( "Selectable" )->setValue( false );

    // Create a template display to display all templates listed by the template nodelet
    template_display_ = manager_->createDisplay( "rviz/TemplateDisplayCustom", "Template Display", true );
    ((rviz::TemplateDisplayCustom*)template_display_)->setVisualizationManager(manager_);

    // Create a display for 3D selection
    selection_3d_display_ = manager_->createDisplay( "rviz/Selection3DDisplayCustom", "3D Selection Display", true );

    // Connect the 3D selection tool to
    QObject::connect(this, SIGNAL(queryContext(int,int)), selection_3d_display_, SLOT(queryContext(int,int)));
    QObject::connect(selection_3d_display_, SIGNAL(setContext(int)), this, SLOT(setContext(int)));

    // Create a display for waypoints
    waypoints_display_ = manager_->createDisplay( "rviz/PathDisplayCustom", "Path Display", true );
    waypoints_display_->subProp( "Topic" )->setValue( "/waypoint/list" );

    // Create another display for waypoints, this time the ones that have already been achieved
    achieved_waypoints_display_ = manager_->createDisplay( "rviz/PathDisplayCustom", "Path Display", true );
    achieved_waypoints_display_->subProp( "Topic" )->setValue( "/waypoint/achieved_list" );
    achieved_waypoints_display_->subProp( "Color" )->setValue( QColor( 150, 150, 255 ) );

    // connect the 3d selection tool to its display
    QObject::connect(this, SIGNAL(setRenderPanel(rviz::RenderPanel*)), selection_3d_display_, SLOT(setRenderPanel(rviz::RenderPanel*)));
    QObject::connect(selection_3d_display_, SIGNAL(newSelection(Ogre::Vector3)), this, SLOT(newSelection(Ogre::Vector3)));
    QObject::connect(selection_3d_display_, SIGNAL(setSelectionRay(Ogre::Ray)), this, SLOT(setSelectionRay(Ogre::Ray)));
    QObject::connect(this, SIGNAL(resetSelection()), selection_3d_display_, SLOT(resetSelection()));
    QObject::connect(this, SIGNAL(setMarkerScale(float)), selection_3d_display_, SLOT(setMarkerScale(float)));

    Q_EMIT setRenderPanel(this->render_panel_);

    // handles mouse events without rviz::tool
    mouse_event_handler_ = new vigir_ocs::MouseEventHandler();
    QObject::connect(render_panel_, SIGNAL(signalMousePressEvent(QMouseEvent*)), mouse_event_handler_, SLOT(mousePressEvent(QMouseEvent*)));
    QObject::connect(render_panel_, SIGNAL(signalMouseReleaseEvent(QMouseEvent*)), mouse_event_handler_, SLOT(mouseReleaseEvent(QMouseEvent*)));
    QObject::connect(mouse_event_handler_, SIGNAL(mouseLeftButtonCtrl(bool,int,int)), selection_3d_display_, SLOT(raycastRequest(bool,int,int)));//SLOT(createMarker(bool,int,int))); // RAYCAST -> need createMarkerOnboard that sends raycast query
    QObject::connect(mouse_event_handler_, SIGNAL(mouseLeftButtonShift(bool,int,int)), selection_3d_display_, SLOT(raycastRequestROI(bool,int,int)));//SLOT(createROISelection(bool,int,int)));
    QObject::connect(mouse_event_handler_, SIGNAL(mouseRightButton(bool,int,int)), this, SLOT(createContextMenu(bool,int,int)));

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

    // Make the move camera tool the currently selected one
    manager_->getToolManager()->setCurrentTool( move_camera_tool_ );

    // Footstep array
    footsteps_array_ = manager_->createDisplay( "rviz/MarkerArray", "Footsteps array", true );
    footsteps_array_->subProp( "Marker Topic" )->setValue( "/flor_footstep_planner/footsteps_array" );

    goal_pose_ = manager_->createDisplay( "rviz/Pose", "Goal pose", true );
    goal_pose_->subProp( "Topic" )->setValue( "/goalpose" );
    goal_pose_->subProp( "Shape" )->setValue( "Axes" );

    planner_start_ = manager_->createDisplay( "rviz/Pose", "Start pose", true );
    planner_start_->subProp( "Topic" )->setValue( "/ros_footstep_planner/start" );
    planner_start_->subProp( "Shape" )->setValue( "Axes" );

    planned_path_ = manager_->createDisplay( "rviz/Path", "Planned path", true );
    planned_path_->subProp( "Topic" )->setValue( "/flor/path" );

    set_goal_tool_->getPropertyContainer()->subProp( "Topic" )->setValue( "/goalpose" );

    // create the hands displays
    left_hand_model_ = manager_->createDisplay( "rviz/RobotDisplayCustom", "Robot left hand model", true );
    left_hand_model_->subProp( "TF Prefix" )->setValue( "/left_hand_model" );
    left_hand_model_->subProp( "Robot Description" )->setValue( "left_hand_robot_description" );
    left_hand_model_->subProp( "Alpha" )->setValue( 0.5f );
    left_hand_model_->subProp( "Color" )->setValue( QColor( 255, 255, 0 ) );

    right_hand_model_ = manager_->createDisplay( "rviz/RobotDisplayCustom", "Robot right hand model", true );
    right_hand_model_->subProp( "TF Prefix" )->setValue( "/right_hand_model" );
    right_hand_model_->subProp( "Robot Description" )->setValue( "right_hand_robot_description" );
    right_hand_model_->subProp( "Alpha" )->setValue( 0.5f );
    right_hand_model_->subProp( "Color" )->setValue( QColor( 0, 255, 255 ) );

    // create the simulation robot display
    //ghost_robot_model_ = manager_->createDisplay( "rviz/RobotDisplayCustom", "Robot simulation model", false );
    //ghost_robot_model_->subProp( "TF Prefix" )->setValue( "/simulation" );
    //ghost_robot_model_->subProp( "Visual Enabled" )->setValue( true );
    //ghost_robot_model_->subProp( "Collision Enabled" )->setValue( false );
    //ghost_robot_model_->subProp( "Color" )->setValue( QColor( 30, 200, 30 ) );
    //ghost_robot_model_->subProp( "Alpha" )->setValue( 0.5f );

    ghost_robot_model_ = manager_->createDisplay( "moveit_rviz_plugin/RobotState", "Robot simulation model", false );
    ghost_robot_model_->subProp( "Robot State Topic" )->setValue( "/flor/ghost/robot_state_vis" );
    ghost_robot_model_->subProp( "Robot Alpha" )->setValue( 0.0f );
    ghost_robot_model_->subProp( "Selectable" )->setValue( false );
    ghost_robot_model_->setEnabled(false);

    // Add custom interactive markers to control ghost robot
    rviz::Display* im_left_arm = manager_->createDisplay( "rviz/InteractiveMarkers", "Interactive marker - left arm", false );
    im_left_arm->subProp( "Update Topic" )->setValue( "/l_arm_pose_marker/pose_marker/update" );
    im_left_arm->subProp( "Show Axes" )->setValue( true );
    im_left_arm->subProp( "Show Visual Aids" )->setValue( true );
    im_ghost_robot_.push_back(im_left_arm);
    rviz::Display* im_right_arm = manager_->createDisplay( "rviz/InteractiveMarkers", "Interactive marker - right arm", false );
    im_right_arm->subProp( "Update Topic" )->setValue( "/r_arm_pose_marker/pose_marker/update" );
    im_right_arm->subProp( "Show Axes" )->setValue( true );
    im_right_arm->subProp( "Show Visual Aids" )->setValue( true );
    im_ghost_robot_.push_back(im_right_arm);
    rviz::Display* im_pelvis = manager_->createDisplay( "rviz/InteractiveMarkers", "Interactive marker - right arm", false );
    im_pelvis->subProp( "Update Topic" )->setValue( "/pelvis_pose_marker/pose_marker/update" );
    im_pelvis->subProp( "Show Axes" )->setValue( true );
    im_pelvis->subProp( "Show Visual Aids" )->setValue( true );
    im_ghost_robot_.push_back(im_pelvis);

    geometry_msgs::Point point;
    static InteractiveMarkerServerCustom* marker_server_left_arm = new InteractiveMarkerServerCustom("Ghost Left Arm", "/l_arm_pose_marker", manager_->getFixedFrame().toStdString(), 0.4, point);
    im_ghost_robot_server_.push_back(marker_server_left_arm);
    im_ghost_robot_server_[im_ghost_robot_server_.size()-1]->onFeedback = boost::bind(&Base3DView::onMarkerFeedback, this, _1, _2);
    static InteractiveMarkerServerCustom* marker_server_right_arm = new InteractiveMarkerServerCustom("Ghost Right Arm", "/r_arm_pose_marker", manager_->getFixedFrame().toStdString(), 0.4, point);
    im_ghost_robot_server_.push_back(marker_server_right_arm);
    im_ghost_robot_server_[im_ghost_robot_server_.size()-1]->onFeedback = boost::bind(&Base3DView::onMarkerFeedback, this, _1, _2);
    static InteractiveMarkerServerCustom* marker_server_pelvis = new InteractiveMarkerServerCustom("Ghost Pelvis", "/pelvis_pose_marker", manager_->getFixedFrame().toStdString(), 0.4, point);
    im_ghost_robot_server_.push_back(marker_server_pelvis);
    im_ghost_robot_server_[im_ghost_robot_server_.size()-1]->onFeedback = boost::bind(&Base3DView::onMarkerFeedback, this, _1, _2);

    // subscribe to the pose topics
    end_effector_sub_.push_back(n_.subscribe<geometry_msgs::PoseStamped>( "/flor/ghost/pose/left_hand", 5, &Base3DView::processLeftArmEndEffector, this ));
    end_effector_sub_.push_back(n_.subscribe<geometry_msgs::PoseStamped>( "/flor/ghost/pose/right_hand", 5, &Base3DView::processRightArmEndEffector, this ));

    end_effector_pub_ = n_.advertise<flor_planning_msgs::TargetConfigIkRequest>( "/flor/ghost/set_appendage_poses", 1, false );

    ghost_root_pose_pub_ = n_.advertise<geometry_msgs::PoseStamped>( "/flor/ghost/set_root_pose", 1, false );
    ghost_joint_state_pub_ = n_.advertise<sensor_msgs::JointState>( "/flor/ghost/set_joint_states", 1, false );

    // initialize ghost control config
    saved_state_planning_group_.push_back(0);
    saved_state_planning_group_.push_back(0);
    saved_state_planning_group_.push_back(0);
    saved_state_pose_source_.push_back(0);
    saved_state_pose_source_.push_back(0);
    saved_state_pose_source_.push_back(0);
    saved_state_world_lock_.push_back(0);
    saved_state_world_lock_.push_back(0);
    saved_state_world_lock_.push_back(0);
    saved_state_collision_avoidance_ = 0;
    saved_state_lock_pelvis_ = 1;

    // ghost state
    ghost_control_state_sub_ = n_.subscribe<flor_ocs_msgs::OCSGhostControl>( "/flor/ocs/ghost_ui_state", 5, &Base3DView::processGhostControlState, this );
    reset_pelvis_sub_ = n_.subscribe<std_msgs::Bool>( "/flor/ocs/reset_pelvis", 5, &Base3DView::processPelvisResetRequest, this );

    // Create a RobotModel display.
    robot_model_ = manager_->createDisplay( "rviz/RobotDisplayCustom", "Robot model", true );
    robot_model_->subProp( "Color" )->setValue( QColor( 200, 200, 200 ) );
    robot_model_->subProp( "Alpha" )->setValue( 1.0f );

    // ground map middle man
    //ground_map_sub_ = n_.subscribe<nav_msgs::OccupancyGrid>( "/flor/worldmodel/grid_map_near_robot", 5, &Base3DView::processNewMap, this );
    ground_map_sub_ = n_.subscribe<nav_msgs::OccupancyGrid>( "/flor/worldmodel/ocs/gridmap_result", 5, &Base3DView::processNewMap, this );

    // point cloud request/selection publisher
    point_cloud_result_sub_ =  n_.subscribe<sensor_msgs::PointCloud2>( "/flor/worldmodel/ocs/dist_query_pointcloud_result", 5, &Base3DView::processPointCloud, this );
    global_selection_pos_pub_ = n_.advertise<geometry_msgs::Point>( "/new_point_cloud_request", 1, false );
    global_selection_pos_sub_ = n_.subscribe<geometry_msgs::Point>( "/new_point_cloud_request", 5, &Base3DView::processNewSelection, this );

    QObject::connect(this, SIGNAL(setMarkerPosition(float,float,float)), selection_3d_display_, SLOT(setMarkerPosition(float,float,float)));

    joint_states_sub_ = n_.subscribe<sensor_msgs::JointState>( "atlas/joint_states", 2, &Base3DView::processJointStates, this );

    // advertise pointcloud request
    pointcloud_request_world_pub_ = n_.advertise<flor_perception_msgs::RaycastRequest>( "/flor/worldmodel/ocs/dist_query_pointcloud_request_world", 1, false );

    // frustum
    frustum_viewer_list_["head_left"] = manager_->createDisplay( "rviz/FrustumDisplayCustom", "Frustum - Left Eye", true );
    QObject::connect(this, SIGNAL(setFrustum(const float&,const float&,const float&,const float&)), frustum_viewer_list_["head_left"], SLOT(setFrustum(const float&,const float&,const float&,const float&)));


}

// Destructor.
Base3DView::~Base3DView()
{
    delete manager_;
}

void Base3DView::robotModelToggled( bool selected )
{
    robot_model_->setEnabled( selected );
    ghost_robot_model_->subProp( "Robot Alpha" )->setValue( 0.5f );
    robot_model_->subProp( "Alpha" )->setValue( 1.0f );
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

void Base3DView::gridMapToggled( bool selected )
{
    for(int i = 0; i < ground_map_.size(); i++)
        ground_map_[i]->setEnabled( selected );
}

void Base3DView::footstepPlanningToggled( bool selected )
{
    goal_pose_->setEnabled( selected );
    planner_start_->setEnabled( selected );
    planned_path_->setEnabled( selected );
    footsteps_array_->setEnabled( selected );
}

void Base3DView::simulationRobotToggled( bool selected )
{
    ghost_robot_model_->setEnabled( selected );
    if(selected)
    {
        update_markers_ = true;

        publishGhostPoses();
    }
    ghost_robot_model_->subProp( "Robot Alpha" )->setValue( 0.5f );
    robot_model_->subProp( "Alpha" )->setValue( 1.0f );
}

void Base3DView::cameraToggled( bool selected )
{
    if(selected)
    {
        manager_->getToolManager()->setCurrentTool( move_camera_tool_ );

        // enable robot IK markers
        for( int i = 0; i < im_ghost_robot_.size(); i++ )
        {
            im_ghost_robot_[i]->setEnabled( false );
        }

        // disable template marker
        Q_EMIT enableTemplateMarkers( false );
    }
}

void Base3DView::selectToggled( bool selected )
{
    //if(selected)
    //    manager_->getToolManager()->setCurrentTool( selection_tool_ );
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
        for( int i = 0; i < im_ghost_robot_.size(); i++ )
        {
            im_ghost_robot_[i]->setEnabled( true );
        }

        publishGhostPoses();

        // disable template marker
        Q_EMIT enableTemplateMarkers( false );
    }
}

void Base3DView::markerTemplateToggled( bool selected )
{
    if(selected)
    {
        manager_->getToolManager()->setCurrentTool( interactive_markers_tool_ );

        // disable robot IK markers
        for( int i = 0; i < im_ghost_robot_.size(); i++ )
        {
            im_ghost_robot_[i]->setEnabled( false );
        }
        // enable template markers
        Q_EMIT enableTemplateMarkers( true );
    }
}

void Base3DView::vectorPressed()
{
    manager_->getToolManager()->setCurrentTool( set_goal_tool_ );
}

void Base3DView::processPointCloud( const sensor_msgs::PointCloud2::ConstPtr& pc )
{
    std::cout << "point cloud received" << std::endl;
    pcl::PointCloud<pcl::PointXYZ> pclCloud;
    pcl::fromROSMsg(*pc, pclCloud);
    Eigen::Vector4f centroid;
    if(pcl::compute3DCentroid(pclCloud,centroid) != 0)
    {
        geometry_msgs::Point point;
        point.x = centroid[0];
        point.y = centroid[1];
        point.z = centroid[2];
        std::cout << "centroid: " << point.x << ", " << point.y << ", " << point.z << std::endl;
        global_selection_pos_pub_.publish(point);
    }
}

void Base3DView::processNewSelection( const geometry_msgs::Point::ConstPtr& pose )
{
    Q_EMIT setMarkerPosition(pose->x,pose->y,pose->z);
}

void Base3DView::newSelection( Ogre::Vector3 position )
{
    selection_position_ = position;
    selected_ = true;
}

void Base3DView::templatePathChanged( QString path )
{
    selected_template_path_ = path;
}

void Base3DView::transform(Ogre::Vector3& position, Ogre::Quaternion& orientation, const char* from_frame, const char* to_frame)
{
    //std::cout << "POS bt: " << position.x << ", " << position.y << ", " << position.z << std::endl;
    // put all pose data into a tf stamped pose
    tf::Quaternion bt_orientation(orientation.x, orientation.y, orientation.z, orientation.w);
    tf::Vector3 bt_position(position.x, position.y, position.z);

    std::string frame(from_frame);
    tf::Stamped<tf::Pose> pose_in(tf::Transform(bt_orientation,bt_position), ros::Time(), frame);
    tf::Stamped<tf::Pose> pose_out;

    // convert pose into new frame
    try
    {
      manager_->getFrameManager()->getTFClient()->transformPose( to_frame, pose_in, pose_out );
    }
    catch(tf::TransformException& e)
    {
      ROS_DEBUG("Error transforming from frame '%s' to frame '%s': %s", from_frame, to_frame, e.what());
      return;
    }

    bt_position = pose_out.getOrigin();
    position = Ogre::Vector3(bt_position.x(), bt_position.y(), bt_position.z());
    //std::cout << "POS transform: " << position.x << ", " << position.y << ", " << position.z << std::endl;

    bt_orientation = pose_out.getRotation();
    orientation = Ogre::Quaternion( bt_orientation.w(), bt_orientation.x(), bt_orientation.y(), bt_orientation.z() );
    //std::cout << "QUAT transform: " << orientation.x << ", " << orientation.y << ", " << orientation.z << ", " << orientation.w << std::endl;
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

    if(!selected_)
    {
        flor_ocs_msgs::OCSTemplateAdd cmd;
        geometry_msgs::PoseStamped pose;

        cmd.template_path = path.toStdString();

        pose.pose.position.x = 1;
        pose.pose.position.y = 0;
        pose.pose.position.z = .2;
        pose.pose.orientation.x = 0;
        pose.pose.orientation.y = 0;
        pose.pose.orientation.z = 0;
        pose.pose.orientation.w = 1;
        pose.header.frame_id = "/pelvis";
        transform("/world",pose);

        cmd.pose = pose;

        // publish complete list of templates and poses
        template_add_pub_.publish( cmd );
    }
    else
    {
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

        pose.header.frame_id = "/world";

        cmd.pose = pose;

        // publish complete list of templates and poses
        template_add_pub_.publish( cmd );

        selected_ = false;

        Q_EMIT resetSelection();
    }
}

void Base3DView::insertWaypoint()
{
    if(selected_)
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

        pose.header.frame_id = "/world";

        cmd.pose = pose;

        // publish complete list of templates and poses
        waypoint_add_pub_.publish( cmd );

        selected_ = false;
        Q_EMIT resetSelection();
    }
}

void Base3DView::createContextMenu(bool, int x, int y)
{
    // first we need to query the 3D scene to retrieve the context
    Q_EMIT queryContext(x,y);
    // context is stored in the active_context_ variable

    QPoint globalPos = this->mapToGlobal(QPoint(x+10,y+10));

    QMenu myMenu;

    myMenu.addAction("Insert Template");
    if(selected_) myMenu.addAction("Insert Waypoint");

    QAction* selectedItem = myMenu.exec(globalPos);
    std::cout << selectedItem << std::endl;
    if(selectedItem != NULL)
    {
        if (selectedItem->text() == QString("Insert Template"))
        {
            if(!selected_template_path_.isEmpty())
                insertTemplate(selected_template_path_);
        }
        else if (selectedItem->text() == QString("Insert Waypoint"))
        {
            insertWaypoint();
        }
        else
        {
            // nothing was chosen, probably don't need this anymore since checking for NULL
        }
    }
}

void Base3DView::setContext(int context)
{
    active_context_ = context;
    std::cout << "Active context: " << active_context_ << std::endl;
}

void Base3DView::setSelectionRay( Ogre::Ray ray )
{
    last_selection_ray_ = ray;
}

void Base3DView::publishPointCloudWorldRequest()
{
    flor_perception_msgs::RaycastRequest request;

    request.origin.x = last_selection_ray_.getOrigin().x;
    request.origin.y = last_selection_ray_.getOrigin().y;
    request.origin.z = last_selection_ray_.getOrigin().z;

    request.direction.x = last_selection_ray_.getDirection().x;
    request.direction.y = last_selection_ray_.getDirection().y;
    request.direction.z = last_selection_ray_.getDirection().z;

    pointcloud_request_world_pub_.publish(request);
}

void Base3DView::processNewMap(const nav_msgs::OccupancyGrid::ConstPtr &map)
{
    static int counter = 0;
    std::stringstream map_topic;
    map_topic << "map_topic_" << counter++;
    rviz::Display* ground_map = manager_->createDisplay( "rviz/MapDisplayCustom", "Ground map", true );
    ((rviz::MapDisplayCustom*)ground_map)->incomingMap(map);
    ground_map_.push_back(ground_map);
    int stored_maps = 30;// THIS VALUE DETERMINES HOW MANY WE STORE
    if(ground_map_.size() > stored_maps)
    {
        rviz::Display* tmp = ground_map_[0];
        tmp->setEnabled(false);
        delete tmp;
        manager_->notifyConfigChanged();
        ground_map_.erase(ground_map_.begin());
    }
    float alpha = 1.0f, step = (0.95f/stored_maps);
    unsigned short priority = stored_maps-1;
    for(int i = ground_map_.size()-1; i >= 0; i--,alpha-=step)
    {
        //std::cout << i << " " << alpha << std::endl;
        ground_map_[i]->subProp("Alpha")->setValue(alpha);
        ((rviz::MapDisplayCustom*)ground_map_[i])->setPriority(priority--);
    }
}

void Base3DView::processLeftArmEndEffector(const geometry_msgs::PoseStamped::ConstPtr &pose)
{
    //if(update_markers_)
    {
        for(int i = 0; i < im_ghost_robot_server_.size(); i++)
        {
            if(im_ghost_robot_server_[i]->getMarkerName() == "Ghost Left Arm")
            {
                im_ghost_robot_server_[i]->setPose(*pose);
                break;
            }
        }
        update_markers_ = false;
    }
}

void Base3DView::processRightArmEndEffector(const geometry_msgs::PoseStamped::ConstPtr &pose)
{
    //if(update_markers_)
    {
        for(int i = 0; i < im_ghost_robot_server_.size(); i++)
        {
            if(im_ghost_robot_server_[i]->getMarkerName() == "Ghost Right Arm")
            {
                im_ghost_robot_server_[i]->setPose(*pose);
                break;
            }
        }
        update_markers_ = false;
    }
}

void Base3DView::onMarkerFeedback(std::string topic_name, geometry_msgs::PoseStamped pose)
{
    //ROS_ERROR("Marker feedback on topic %s, have %d markers instantiated",topic_name.c_str(),(unsigned int)im_ghost_robot_.size());
    end_effector_pose_list_[topic_name] = pose;

    publishGhostPoses();

}

void Base3DView::publishGhostPoses()
{
    bool left = saved_state_planning_group_[0];
    bool right = saved_state_planning_group_[1];
    bool torso = saved_state_planning_group_[2];

    flor_planning_msgs::TargetConfigIkRequest cmd;


    if(left && end_effector_pose_list_.find( "/l_arm_pose_marker") != end_effector_pose_list_.end())
        cmd.target_poses.push_back(end_effector_pose_list_["/l_arm_pose_marker"]);

    if(right && end_effector_pose_list_.find( "/r_arm_pose_marker") != end_effector_pose_list_.end())
        cmd.target_poses.push_back(end_effector_pose_list_["/r_arm_pose_marker"]);

    cmd.lock_to_world.resize(cmd.target_poses.size());
    for(int i = 0; i < cmd.target_poses.size(); i++)
        cmd.lock_to_world[i].data = saved_state_world_lock_[i];


    if(left && !right && !torso)
        cmd.planning_group.data = "l_arm_group";
    else if(left && !right && torso)
        cmd.planning_group.data = "l_arm_with_torso_group";
    else if(!left && right && !torso)
        cmd.planning_group.data = "r_arm_group";
    else if(!left && right && torso)
        cmd.planning_group.data = "r_arm_with_torso_group";
    else if(left && right && !torso)
        cmd.planning_group.data = "both_arms_group";
    else if(left && right && torso)
        cmd.planning_group.data = "both_arms_with_torso_group";

    end_effector_pub_.publish(cmd);

    if(saved_state_lock_pelvis_)
    {
        Ogre::Vector3 position(0,0,0);
        Ogre::Quaternion orientation(1,0,0,0);
        transform(position, orientation, "/pelvis", "/world");

        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = position.x;
        pose.pose.position.y = position.y;
        pose.pose.position.z = position.z;
        pose.pose.orientation.x = orientation.x;
        pose.pose.orientation.y = orientation.y;
        pose.pose.orientation.z = orientation.z;
        pose.pose.orientation.w = orientation.w;
        pose.header.frame_id = "/world";
        pose.header.stamp = ros::Time::now();

        ghost_root_pose_pub_.publish(pose);

        for(int i = 0; i < im_ghost_robot_server_.size(); i++)
        {
            if(im_ghost_robot_server_[i]->getMarkerName() == "Ghost Pelvis")
            {
                im_ghost_robot_server_[i]->setPose(pose);
                break;
            }
        }
    }
    else
    {
        // how do I set world lock for torso?
        ghost_root_pose_pub_.publish(end_effector_pose_list_["/pelvis_pose_marker"]);
    }
}

void Base3DView::processGhostControlState(const flor_ocs_msgs::OCSGhostControl::ConstPtr &msg)
{
    saved_state_planning_group_.clear();
    saved_state_pose_source_.clear();
    saved_state_world_lock_.clear();

    saved_state_planning_group_ = msg->planning_group;
    saved_state_pose_source_ = msg->pose_source;
    saved_state_world_lock_ = msg->world_lock;
    saved_state_collision_avoidance_ = msg->collision_avoidance;
    saved_state_lock_pelvis_ = msg->lock_pelvis;

    if(msg->snap)
        snap_ghost_to_robot_ = true;
}

void Base3DView::processJointStates(const sensor_msgs::JointState::ConstPtr &states)
{
    if(snap_ghost_to_robot_)
    {
        ghost_joint_state_pub_.publish(states);
        snap_ghost_to_robot_ = false;
    }
}

void Base3DView::processPelvisResetRequest( const std_msgs::Bool::ConstPtr &msg )
{
    if(end_effector_pose_list_.find( "/pelvis_pose_marker") != end_effector_pose_list_.end())
    {
        // trasnfrom current marker pose from world to pelvis
        geometry_msgs::PoseStamped marker_pose = end_effector_pose_list_["/pelvis_pose_marker"];
        Ogre::Vector3 position(0,0,0);
        Ogre::Quaternion orientation(1,0,0,0);
        transform(position, orientation, "/pelvis", "/world");

        // create pose based on /pelvis frame
        geometry_msgs::PoseStamped pose;// = end_effector_pose_list_["/pelvis_pose_marker"];
        pose.pose.position.x = marker_pose.pose.position.x;
        pose.pose.position.y = marker_pose.pose.position.y;
        pose.pose.position.z = marker_pose.pose.position.z;
        pose.pose.orientation.x = orientation.x;
        pose.pose.orientation.y = orientation.y;
        pose.pose.orientation.z = orientation.z;
        pose.pose.orientation.w = orientation.w;
        pose.header.frame_id = "/world";
        pose.header.stamp = ros::Time::now();

        ghost_root_pose_pub_.publish(pose);

        for(int i = 0; i < im_ghost_robot_server_.size(); i++)
        {
            if(im_ghost_robot_server_[i]->getMarkerName() == "Ghost Pelvis")
            {
                im_ghost_robot_server_[i]->setPose(pose);
                break;
            }
        }
    }
}
}
