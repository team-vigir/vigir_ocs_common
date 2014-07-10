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
#include <QWidgetAction>
#include <QSlider>
#include <QDoubleSpinBox>
#include <QCheckBox>
#include <QVector3D>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

#include <pcl_conversions/pcl_conversions.h>

#include "rviz/visualization_manager.h"
#include "rviz/visualization_frame.h"
//#include "rviz/render_panel.h"
#include <render_panel_custom.h>
#include "rviz/display.h"
#include "rviz/frame_manager.h"
#include "rviz/tool_manager.h"
#include "rviz/view_manager.h"
#include "rviz/default_plugin/view_controllers/fixed_orientation_ortho_view_controller.h"
#include "rviz/default_plugin/view_controllers/orbit_view_controller.h"
#include "rviz/properties/parse_color.h"
#include "rviz/properties/vector_property.h"
#include <template_display_custom.h>
#include "selection_3d_display_custom.h"
#include "map_display_custom.h"
#include "base_3d_view.h"

#include "flor_ocs_msgs/OCSTemplateAdd.h"
#include "flor_ocs_msgs/OCSTemplateRemove.h"
#include "flor_ocs_msgs/OCSWaypointAdd.h"
#include "flor_perception_msgs/EnvironmentRegionRequest.h"
#include "flor_planning_msgs/TargetConfigIkRequest.h"
#include "flor_planning_msgs/CartesianMotionRequest.h"
#include "flor_planning_msgs/CircularMotionRequest.h"

namespace vigir_ocs
{

// Constructor for Base3DView.  This does most of the work of the class.
Base3DView::Base3DView( Base3DView* copy_from, std::string base_frame, std::string widget_name, QWidget *parent )
    : QWidget( parent )
    , base_frame_(base_frame)
    , widget_name_(widget_name)
    , selected_(false)
    , update_markers_(false)
    , snap_ghost_to_robot_(true)
    , snap_left_hand_to_ghost_(false)
    , snap_right_hand_to_ghost_(false)
    , marker_published_(0)
    , stored_maps_(30) // determines how many maps will be stored
    , moving_pelvis_(false)
    , moving_l_arm_(false)
    , moving_r_arm_(false)
    , update_l_arm_color_(false)
    , update_r_arm_color_(false)
    , left_marker_moveit_loopback_(true)
    , right_marker_moveit_loopback_(true)
    , position_only_ik_(false)
    , visualize_grid_map_(true)
    , initializing_context_menu_(0)
    , circular_marker_(0)
{
    // Construct and lay out render panel.
    render_panel_ = new rviz::RenderPanelCustom();
    ((rviz::RenderPanelCustom*)render_panel_)->setEventFilters(rviz::RenderPanelCustom::MOUSE_PRESS_EVENT,false,Qt::NoModifier,Qt::RightButton);
    ((rviz::RenderPanelCustom*)render_panel_)->setEventFilters(rviz::RenderPanelCustom::MOUSE_RELEASE_EVENT,false,Qt::NoModifier,Qt::RightButton);
    ((rviz::RenderPanelCustom*)render_panel_)->setEventFilters(rviz::RenderPanelCustom::MOUSE_MOVE_EVENT,false,Qt::NoModifier,Qt::RightButton);

    QVBoxLayout* main_layout = new QVBoxLayout;
    main_layout->setMargin(0);
    main_layout->addWidget( render_panel_ );

    // Set the top-level layout for this MyViz widget.
    setLayout( main_layout );

    // if there's
    if(copy_from != NULL)
    {
        is_primary_view_ = false;

        manager_ = copy_from->getVisualizationManager();
        render_panel_->initialize( manager_->getSceneManager(), manager_ );
        view_id_ = manager_->addRenderPanel( render_panel_ );

        selection_3d_display_ = copy_from->getSelection3DDisplay();
        mouse_event_handler_ = copy_from->getMouseEventHander();
    }
    else
    {
        is_primary_view_ = true;

        // Next we initialize the main RViz classes.
        //
        // The VisualizationManager is the container for Display objects,
        // holds the main Ogre scene, holds the ViewController, etc.  It is
        // very central and we will probably need one in every usage of
        // librviz.
        manager_ = new rviz::VisualizationManager( render_panel_ );
        render_panel_->initialize( manager_->getSceneManager(), manager_ );
        view_id_ = manager_->addRenderPanel( render_panel_ );

        Ogre::SceneNode* lightSceneNode = NULL;
        Ogre::Light* light = manager_->getSceneManager()->createLight();

        // I can set some attributes of the light.
        // The basic light type can be :
        //		pointlight (like a candle?)
        //		spotlight (kind of 'conic' light)
        //		directional light (like the sun in an outdoor scene).
        // Directional light is like parallel rays coming from 1 direction.
        light->setType(Ogre::Light::LT_DIRECTIONAL);

        // Here I choose the color of the light.
        // The diffuse color is the main color of the light.
        // The specular color is its color when reflected on an imperfect surface.
        // For example, when my bald head skin reflect the sun, it makes a bright round of specular color.
        //
        // The final color of an object also depends on its material.
        // Color values vary between 0.0(minimum) to 1.0 (maximum).
        light->setDiffuseColour(0.25f, 0.25f, 0.25f); // this will be a red light
        light->setSpecularColour(1.0f, 1.0f, 1.0f);// color of 'reflected' light

        lightSceneNode = manager_->getSceneManager()->getRootSceneNode()->createChildSceneNode();
        lightSceneNode->attachObject(light);

        // Set topic that will be used as 0,0,0 -> reference for all the other transforms
        // IMPORTANT: WITHOUT THIS, ALL THE DIFFERENT PARTS OF THE ROBOT MODEL WILL BE DISPLAYED AT 0,0,0
        manager_->setFixedFrame(base_frame_.c_str());

        manager_->initialize();
        manager_->startUpdate();

        // First remove all existin tools
        manager_->getToolManager()->removeAll();
        // Add support for interactive markers
        interactive_markers_tool_ = manager_->getToolManager()->addTool( "rviz/InteractionToolCustom" );
        // Add support for selection
        //selection_tool_ = manager_->getToolManager()->addTool( "rviz/Select" );
        // Add support for camera movement
        move_camera_tool_ = manager_->getToolManager()->addTool( "rviz/MoveCamera" );
        // Add support for goal specification/vector navigation
        set_walk_goal_tool_ = manager_->getToolManager()->addTool( "rviz/SetGoal" );
        set_walk_goal_tool_->getPropertyContainer()->subProp( "Topic" )->setValue( "/goal_pose_walk" );
        set_step_goal_tool_ = manager_->getToolManager()->addTool( "rviz/SetGoal" );
        set_step_goal_tool_->getPropertyContainer()->subProp( "Topic" )->setValue( "/goal_pose_step" );

        grid_ = manager_->createDisplay( "rviz/Grid", "Grid", true );
        ROS_ASSERT( grid_ != NULL );
        grid_->subProp( "Plane Cell Count" )->setValue( 50 );
        grid_->subProp( "Cell Size" )->setValue( .5 );
        grid_->subProp( "Alpha" )->setValue( 0.1 );

        // Create a LaserScan display.
        laser_scan_ = manager_->createDisplay( "rviz/LaserScan", "Laser Scan", false );
        ROS_ASSERT( laser_scan_ != NULL );
        laser_scan_->subProp( "Topic" )->setValue( "/laser/scan" );
        laser_scan_->subProp( "Style" )->setValue( "Points" );
        laser_scan_->subProp( "Size (Pixels)" )->setValue( 3 );
        //laser_scan_->subProp( "Size (m)" )->setValue( 0.1 );
        laser_scan_->subProp( "Decay Time" )->setValue( 5 );
        laser_scan_->subProp( "Color Transformer" )->setValue( "AxisColor" );
        laser_scan_->subProp( "Axis" )->setValue( "Z" );
        laser_scan_->subProp( "Selectable" )->setValue( false );

        // Create a MarkerArray display.
        octomap_ = manager_->createDisplay( "octomap_rviz_plugin/OccupancyGrid", "Octomap", true );
        ROS_ASSERT( octomap_ != NULL );
        octomap_->subProp( "Octomap Topic" )->setValue( "/flor/worldmodel/ocs/octomap_result" );

        // Create a point cloud display.
        stereo_point_cloud_viewer_ = manager_->createDisplay( "rviz/PointCloud2", "Stereo Point Cloud", true );
        ROS_ASSERT( stereo_point_cloud_viewer_ != NULL );
        stereo_point_cloud_viewer_->subProp( "Style" )->setValue( "Flat Squares" );
        stereo_point_cloud_viewer_->subProp( "Topic" )->setValue( "/flor/worldmodel/ocs/stereo_cloud_result" );
        stereo_point_cloud_viewer_->subProp( "Size (m)" )->setValue( 0.01 );
        stereo_point_cloud_viewer_->subProp( "Color Transformer" )->setValue("Intensity");
        stereo_point_cloud_viewer_->subProp( "Channel Name" )->setValue("intensity");
        stereo_point_cloud_viewer_->subProp( "Use rainbow" )->setValue(false);
        stereo_point_cloud_viewer_->subProp( "Decay Time" )->setValue( 0 );
        stereo_point_cloud_viewer_->subProp( "Selectable" )->setValue( false );

        region_point_cloud_viewer_ = manager_->createDisplay( "rviz/PointCloud2", "LIDAR Point Cloud", true );
        ROS_ASSERT( region_point_cloud_viewer_ != NULL );
        region_point_cloud_viewer_->subProp( "Style" )->setValue( "Flat Squares" );
        region_point_cloud_viewer_->subProp( "Topic" )->setValue( "/flor/worldmodel/ocs/cloud_result" );
        region_point_cloud_viewer_->subProp( "Size (m)" )->setValue( 0.01 );
        region_point_cloud_viewer_->subProp( "Color Transformer" )->setValue( "AxisColor" );
        region_point_cloud_viewer_->subProp( "Axis" )->setValue( "Z" );
        region_point_cloud_viewer_->subProp( "Decay Time" )->setValue( 0 );
        region_point_cloud_viewer_->subProp( "Selectable" )->setValue( false );

        // point cloud request
        raycast_point_cloud_viewer_ = manager_->createDisplay( "rviz/PointCloud2", "Raycast Point Cloud", true );
        ROS_ASSERT( raycast_point_cloud_viewer_ != NULL );
        raycast_point_cloud_viewer_->subProp( "Style" )->setValue( "Flat Squares" );
        raycast_point_cloud_viewer_->subProp( "Topic" )->setValue( "/flor/worldmodel/ocs/dist_query_pointcloud_result" );
        raycast_point_cloud_viewer_->subProp( "Size (Pixels)" )->setValue( 3 );
        raycast_point_cloud_viewer_->subProp( "Color Transformer" )->setValue( "AxisColor" );
        raycast_point_cloud_viewer_->subProp( "Decay Time" )->setValue( 0 );
        raycast_point_cloud_viewer_->subProp( "Selectable" )->setValue( false );

        // Create a template display to display all templates listed by the template nodelet
        template_display_ = manager_->createDisplay( "rviz/TemplateDisplayCustom", "Template Display", true );
        ((rviz::TemplateDisplayCustom*)template_display_)->setVisualizationManager(manager_);

        // Create a display for waypoints
        waypoints_display_ = manager_->createDisplay( "rviz/PathDisplayCustom", "Path Display", true );
        waypoints_display_->subProp( "Topic" )->setValue( "/waypoint/list" );

        // Create another display for waypoints, this time the ones that have already been achieved
        achieved_waypoints_display_ = manager_->createDisplay( "rviz/PathDisplayCustom", "Path Display", true );
        achieved_waypoints_display_->subProp( "Topic" )->setValue( "/waypoint/achieved_list" );
        achieved_waypoints_display_->subProp( "Color" )->setValue( QColor( 150, 150, 255 ) );


        // create a publisher to add templates
        template_add_pub_   = nh_.advertise<flor_ocs_msgs::OCSTemplateAdd>( "/template/add", 1, false );

        // create a publisher to add waypoints
        waypoint_add_pub_   = nh_.advertise<flor_ocs_msgs::OCSWaypointAdd>( "/waypoint/add", 1, false );

        selection_position_ = Ogre::Vector3(0,0,0);

        // Make the move camera tool the currently selected one
        manager_->getToolManager()->setCurrentTool( interactive_markers_tool_ );

        // Footstep array
        footsteps_array_ = manager_->createDisplay( "rviz/MarkerArray", "Footsteps array", true );
        footsteps_array_->subProp( "Marker Topic" )->setValue( "/flor/walk_monitor/footsteps_array" );

        footsteps_path_body_array_ = manager_->createDisplay( "rviz/MarkerArray", "Footsteps Path Body", true );
        footsteps_path_body_array_->subProp( "Marker Topic" )->setValue( "/flor/walk_monitor/footsteps_path_body_array" );

        goal_pose_walk_ = manager_->createDisplay( "rviz/Pose", "Goal pose", true );
        goal_pose_walk_->subProp( "Topic" )->setValue( "/goal_pose_walk" );
        goal_pose_walk_->subProp( "Shape" )->setValue( "Axes" );

        goal_pose_step_ = manager_->createDisplay( "rviz/Pose", "Goal pose", true );
        goal_pose_step_->subProp( "Topic" )->setValue( "/goal_pose_step" );
        goal_pose_step_->subProp( "Shape" )->setValue( "Axes" );

        planner_start_ = manager_->createDisplay( "rviz/Pose", "Start pose", true );
        planner_start_->subProp( "Topic" )->setValue( "/ros_footstep_planner/start" );
        planner_start_->subProp( "Shape" )->setValue( "Axes" );

        planned_path_ = manager_->createDisplay( "rviz/Path", "Planned path", true );
        planned_path_->subProp( "Topic" )->setValue( "/flor/walk_monitor/path" );

        left_ft_sensor_ = manager_->createDisplay("rviz/WrenchStamped", "Left F/T sensor", false);
        left_ft_sensor_->subProp("Topic")->setValue("/flor/l_hand/force_torque_sensor");
        left_ft_sensor_->subProp("Alpha")->setValue(0.5);
        left_ft_sensor_->subProp("Arrow Scale")->setValue(0.01);
        left_ft_sensor_->subProp("Arrow Width")->setValue(0.3);

        right_ft_sensor_ = manager_->createDisplay("rviz/WrenchStamped", "Right F/T sensor", false);
        right_ft_sensor_->subProp("Topic")->setValue("/flor/r_hand/force_torque_sensor");
        right_ft_sensor_->subProp("Alpha")->setValue(0.5);
        right_ft_sensor_->subProp("Arrow Scale")->setValue(0.01);
        right_ft_sensor_->subProp("Arrow Width")->setValue(0.3);


        // create the grasp hands displays (blue/yellow hands)
        left_grasp_hand_model_ = manager_->createDisplay( "moveit_rviz_plugin/RobotState", "Robot left grasp hand model", true );
        left_grasp_hand_model_->subProp( "Robot Description" )->setValue( "left_hand_robot_description" );
        left_grasp_hand_model_->subProp( "Robot State Topic" )->setValue( "/flor/ghost/template_left_hand" );
        left_grasp_hand_model_->subProp( "Robot Root Link" )->setValue( "base" );
        left_grasp_hand_model_->subProp( "Robot Alpha" )->setValue( 0.5f );

        right_grasp_hand_model_ = manager_->createDisplay( "moveit_rviz_plugin/RobotState", "Robot right grasp hand model", true );
        right_grasp_hand_model_->subProp( "Robot Description" )->setValue( "right_hand_robot_description" );
        right_grasp_hand_model_->subProp( "Robot State Topic" )->setValue( "/flor/ghost/template_right_hand" );
        right_grasp_hand_model_->subProp( "Robot Root Link" )->setValue( "base" );
        right_grasp_hand_model_->subProp( "Robot Alpha" )->setValue( 0.5f );

        // create the hands displays (iron man hands)
        left_hand_model_ = manager_->createDisplay( "moveit_rviz_plugin/RobotState", "Robot left hand model", true );
        left_hand_model_->subProp( "Robot Description" )->setValue( "left_hand_robot_description" );
        left_hand_model_->subProp( "Robot State Topic" )->setValue( "/flor/ghost/marker_left_hand" );
        left_hand_model_->subProp( "Robot Root Link" )->setValue( "base" );
        left_hand_model_->subProp( "Robot Alpha" )->setValue( 0.5f );

        try
        {
            left_hand_model_loader_.reset(new robot_model_loader::RobotModelLoader("left_hand_robot_description"));
            left_hand_robot_model_ = left_hand_model_loader_->getModel();
            left_hand_robot_state_.reset(new robot_state::RobotState(left_hand_robot_model_));
            left_hand_robot_state_vis_pub_ = nh_.advertise<moveit_msgs::DisplayRobotState>("/flor/ghost/marker_left_hand",1, true);
        }
        catch(...)
        {
            ROS_ERROR("Base3DView: MoveIt! failed to load left hand robot description.");
        }

        {
            // change color of the ghost template hands
            const std::vector<std::string>& link_names = left_hand_robot_model_->getLinkModelNames();

            for (size_t i = 0; i < link_names.size(); ++i)
            {
                moveit_msgs::ObjectColor tmp;
                tmp.id = link_names[i];
                tmp.color.a = 0.5f;
                tmp.color.r = 0.2f;
                tmp.color.g = 0.8f;
                tmp.color.b = 0.2f;
                left_display_state_msg_.highlight_links.push_back(tmp);
            }
        }

        // this is for publishing the hand position in world coordinates for moveit
        left_hand_virtual_link_joint_states_.name.push_back("world_virtual_joint/trans_x");
        left_hand_virtual_link_joint_states_.name.push_back("world_virtual_joint/trans_y");
        left_hand_virtual_link_joint_states_.name.push_back("world_virtual_joint/trans_z");
        left_hand_virtual_link_joint_states_.name.push_back("world_virtual_joint/rot_x");
        left_hand_virtual_link_joint_states_.name.push_back("world_virtual_joint/rot_y");
        left_hand_virtual_link_joint_states_.name.push_back("world_virtual_joint/rot_z");
        left_hand_virtual_link_joint_states_.name.push_back("world_virtual_joint/rot_w");
        left_hand_virtual_link_joint_states_.position.resize(7);

        right_hand_model_ = manager_->createDisplay( "moveit_rviz_plugin/RobotState", "Robot right hand model", true );
        right_hand_model_->subProp( "Robot Description" )->setValue( "right_hand_robot_description" );
        right_hand_model_->subProp( "Robot State Topic" )->setValue( "/flor/ghost/marker_right_hand" );
        right_hand_model_->subProp( "Robot Root Link" )->setValue( "base" );
        right_hand_model_->subProp( "Robot Alpha" )->setValue( 0.5f );

        try
        {
            right_hand_model_loader_.reset(new robot_model_loader::RobotModelLoader("right_hand_robot_description"));
            right_hand_robot_model_ = right_hand_model_loader_->getModel();
            right_hand_robot_state_.reset(new robot_state::RobotState(right_hand_robot_model_));
            right_hand_robot_state_vis_pub_ = nh_.advertise<moveit_msgs::DisplayRobotState>("/flor/ghost/marker_right_hand",1, true);
        }
        catch(...)
        {
            ROS_ERROR("Base3DView: MoveIt! failed to load right hand robot description.");
        }

        {
            // change color of the ghost template hands
            const std::vector<std::string>& link_names = right_hand_robot_model_->getLinkModelNames();

            for (size_t i = 0; i < link_names.size(); ++i)
            {
                moveit_msgs::ObjectColor tmp;
                tmp.id = link_names[i];
                tmp.color.a = 0.5f;
                tmp.color.r = 0.2f;
                tmp.color.g = 0.8f;
                tmp.color.b = 0.2f;
                right_display_state_msg_.highlight_links.push_back(tmp);
            }
        }

        // this is for publishing the hand position in world coordinates for moveit
        right_hand_virtual_link_joint_states_.name.push_back("world_virtual_joint/trans_x");
        right_hand_virtual_link_joint_states_.name.push_back("world_virtual_joint/trans_y");
        right_hand_virtual_link_joint_states_.name.push_back("world_virtual_joint/trans_z");
        right_hand_virtual_link_joint_states_.name.push_back("world_virtual_joint/rot_x");
        right_hand_virtual_link_joint_states_.name.push_back("world_virtual_joint/rot_y");
        right_hand_virtual_link_joint_states_.name.push_back("world_virtual_joint/rot_z");
        right_hand_virtual_link_joint_states_.name.push_back("world_virtual_joint/rot_w");
        right_hand_virtual_link_joint_states_.position.resize(7);

        // create the simulation robot display
        ghost_robot_model_ = manager_->createDisplay( "moveit_rviz_plugin/RobotState", "Robot simulation model", false );
        ghost_robot_model_->subProp( "Robot State Topic" )->setValue( "/flor/ghost/robot_state_vis" );
        ghost_robot_model_->subProp( "Robot Alpha" )->setValue( 0.0f );
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
        rviz::Display* im_pelvis = manager_->createDisplay( "rviz/InteractiveMarkers", "Interactive marker - pelvis", false );
        im_pelvis->subProp( "Update Topic" )->setValue( "/pelvis_pose_marker/pose_marker/update" );
        im_pelvis->subProp( "Show Axes" )->setValue( true );
        im_pelvis->subProp( "Show Visual Aids" )->setValue( true );
        im_ghost_robot_.push_back(im_pelvis);

        //geometry_msgs::Point point;
        //static InteractiveMarkerServerCustom* marker_server_left_arm = new InteractiveMarkerServerCustom("Ghost Left Arm", "/l_arm_pose_marker", manager_->getFixedFrame().toStdString(), 0.4, point);
        //im_ghost_robot_server_.push_back(marker_server_left_arm);
        //im_ghost_robot_server_[im_ghost_robot_server_.size()-1]->onFeedback = boost::bind(&Base3DView::onMarkerFeedback, this, _1, _2);
        //static InteractiveMarkerServerCustom* marker_server_right_arm = new InteractiveMarkerServerCustom("Ghost Right Arm", "/r_arm_pose_marker", manager_->getFixedFrame().toStdString(), 0.4, point);
        //im_ghost_robot_server_.push_back(marker_server_right_arm);
        //im_ghost_robot_server_[im_ghost_robot_server_.size()-1]->onFeedback = boost::bind(&Base3DView::onMarkerFeedback, this, _1, _2);
        //static InteractiveMarkerServerCustom* marker_server_pelvis = new InteractiveMarkerServerCustom("Ghost Pelvis", "/pelvis_pose_marker", manager_->getFixedFrame().toStdString(), 0.4, point);
        //.push_back(marker_server_pelvis);
        //im_ghost_robot_server_[im_ghost_robot_server_.size()-1]->onFeedback = boost::bind(&Base3DView::onMarkerFeedback, this, _1, _2);

        interactive_marker_add_pub_ = nh_.advertise<flor_ocs_msgs::OCSInteractiveMarkerAdd>( "/flor/ocs/interactive_marker_server/add", 1, true );
        interactive_marker_update_pub_ = nh_.advertise<flor_ocs_msgs::OCSInteractiveMarkerUpdate>( "/flor/ocs/interactive_marker_server/update", 1, false );
        interactive_marker_feedback_sub_ = nh_.subscribe( "/flor/ocs/interactive_marker_server/feedback", 5, &Base3DView::onMarkerFeedback, this );;
        interactive_marker_remove_pub_ = nh_.advertise<std_msgs::String>( "/flor/ocs/interactive_marker_server/remove", 1, false );

        // subscribe to the moveit pose topics
        end_effector_sub_.push_back(nh_.subscribe<geometry_msgs::PoseStamped>( "/flor/ghost/pose/left_hand", 5, &Base3DView::processLeftArmEndEffector, this ));
        end_effector_sub_.push_back(nh_.subscribe<geometry_msgs::PoseStamped>( "/flor/ghost/pose/right_hand", 5, &Base3DView::processRightArmEndEffector, this ));

        end_effector_pub_ = nh_.advertise<flor_planning_msgs::TargetConfigIkRequest>( "/flor/ghost/set_appendage_poses", 1, false );

        ghost_root_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>( "/flor/ghost/set_root_pose", 1, false );
        ghost_joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>( "/flor/ghost/set_joint_states", 1, false );

        // subscribe to the grasp ghost hands pose
        ghost_hand_left_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>( "/ghost_left_hand_pose", 5, &Base3DView::processLeftGhostHandPose, this );
        ghost_hand_right_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>( "/ghost_right_hand_pose", 5, &Base3DView::processRightGhostHandPose, this );

        // initialize ghost control config
        ghost_planning_group_.push_back(0);
        ghost_planning_group_.push_back(1);
        ghost_planning_group_.push_back(0);
        ghost_pose_source_.push_back(0);
        ghost_pose_source_.push_back(0);
        ghost_pose_source_.push_back(0);
        ghost_world_lock_.push_back(0);
        ghost_world_lock_.push_back(0);
        ghost_world_lock_.push_back(0);
        moveit_collision_avoidance_ = 0;
        ghost_lock_pelvis_ = 1;

        // ghost state
        ghost_control_state_sub_ = nh_.subscribe<flor_ocs_msgs::OCSGhostControl>( "/flor/ocs/ghost_ui_state", 5, &Base3DView::processGhostControlState, this );
        reset_pelvis_sub_ = nh_.subscribe<std_msgs::Bool>( "/flor/ocs/reset_pelvis", 5, &Base3DView::processPelvisResetRequest, this );
        send_pelvis_sub_ = nh_.subscribe<std_msgs::Bool>( "/flor/ocs/send_pelvis_to_footstep", 5, &Base3DView::processSendPelvisToFootstepRequest, this );
        send_footstep_goal_walk_pub_ = nh_.advertise<geometry_msgs::PoseStamped>( "/goal_pose_walk", 1, false );
        send_footstep_goal_step_pub_ = nh_.advertise<geometry_msgs::PoseStamped>( "/goal_pose_step", 1, false );

        // subscribe to goal pose
        set_walk_goal_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>( "/goal_pose_walk", 5, boost::bind(&Base3DView::processGoalPose, this, _1, 1) );
        set_step_goal_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>( "/goal_pose_step", 5, boost::bind(&Base3DView::processGoalPose, this, _1, 2) );

        // Create a RobotModel display.
        robot_model_ = manager_->createDisplay( "rviz/RobotDisplayCustom", "Robot model", true );
        robot_model_->subProp( "Color" )->setValue( QColor( 200, 200, 200 ) );
        robot_model_->subProp( "Alpha" )->setValue( 1.0f );

        // ground map middle man
        //ground_map_sub_ = n_.subscribe<nav_msgs::OccupancyGrid>( "/flor/worldmodel/grid_map_near_robot", 5, &Base3DView::processNewMap, this );
        ground_map_sub_ = nh_.subscribe<nav_msgs::OccupancyGrid>( "/flor/worldmodel/ocs/gridmap_result", 5, &Base3DView::processNewMap, this );

        // point cloud request/selection publisher
        point_cloud_result_sub_ =  nh_.subscribe<sensor_msgs::PointCloud2>( "/flor/worldmodel/ocs/dist_query_pointcloud_result", 5, &Base3DView::processPointCloud, this );
        global_selection_pos_pub_ = nh_.advertise<geometry_msgs::Point>( "/new_point_cloud_request", 1, false );
        global_selection_pos_sub_ = nh_.subscribe<geometry_msgs::Point>( "/new_point_cloud_request", 5, &Base3DView::processNewSelection, this );

        joint_states_sub_ = nh_.subscribe<sensor_msgs::JointState>( "atlas/joint_states", 2, &Base3DView::processJointStates, this );

        // advertise pointcloud request
        pointcloud_request_world_pub_ = nh_.advertise<flor_perception_msgs::RaycastRequest>( "/flor/worldmodel/ocs/dist_query_pointcloud_request_world", 1, false );

        // frustum
        frustum_viewer_list_["head_left"] = manager_->createDisplay( "rviz/FrustumDisplayCustom", "Frustum - Left Eye", true );

        // Create a display for 3D selection
        selection_3d_display_ = manager_->createDisplay( "rviz/Selection3DDisplayCustom", "3D Selection Display", true );
        //}

        // and advertise the template remove option
        template_remove_pub_ = nh_.advertise<flor_ocs_msgs::OCSTemplateRemove>( "/template/remove", 1, false );

        // flor mode publisher and subscriber
        flor_mode_command_pub_ = nh_.advertise<flor_control_msgs::FlorControlModeCommand>( "/flor/controller/mode_command", 1, false );
        flor_mode_sub_ = nh_.subscribe<flor_control_msgs::FlorControlMode>( "/flor/controller/mode", 5, &Base3DView::processControlMode, this );

        // Connect to the template markers
        QObject::connect(this, SIGNAL(enableTemplateMarker(int, bool)), template_display_, SLOT(enableTemplateMarker(int, bool)));
        QObject::connect(this, SIGNAL(enableTemplateMarkers(bool)), template_display_, SLOT(enableTemplateMarkers(bool)));

        // publishers for the current marker poses
        l_arm_marker_pose_pub_  = nh_.advertise<geometry_msgs::PoseStamped>( "/flor/ghost/l_arm_marker_pose", 1, false );
        r_arm_marker_pose_pub_  = nh_.advertise<geometry_msgs::PoseStamped>( "/flor/ghost/r_arm_marker_pose", 1, false );
        pelvis_marker_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>( "/flor/ghost/pelvis_marker_pose", 1, false );

        // add bounding boxes for the left/right/pelvis markers
        rviz::Display* left_hand_bounding_box_ = manager_->createDisplay( "rviz/BoundingObjectDisplayCustom", "BoundingObject for left hand", true );
        left_hand_bounding_box_->subProp( "Name" )->setValue( "LeftArm" );
        left_hand_bounding_box_->subProp( "Pose Topic" )->setValue( "/flor/ghost/l_arm_marker_pose" );
        left_hand_bounding_box_->subProp( "Alpha" )->setValue( 0.0f );
        rviz::Display* right_hand_bounding_box_ = manager_->createDisplay( "rviz/BoundingObjectDisplayCustom", "BoundingObject for right hand", true );
        right_hand_bounding_box_->subProp( "Name" )->setValue( "RightArm" );
        right_hand_bounding_box_->subProp( "Pose Topic" )->setValue( "/flor/ghost/r_arm_marker_pose" );
        right_hand_bounding_box_->subProp( "Alpha" )->setValue( 0.0f );
        rviz::Display* pelvis_bounding_box_ = manager_->createDisplay( "rviz/BoundingObjectDisplayCustom", "BoundingObject for pelvis", true );
        pelvis_bounding_box_->subProp( "Name" )->setValue( "Pelvis" );
        pelvis_bounding_box_->subProp( "Pose Topic" )->setValue( "/flor/ghost/pelvis_marker_pose" );
        pelvis_bounding_box_->subProp( "Alpha" )->setValue( 0.0f );
        ((rviz::VectorProperty*)pelvis_bounding_box_->subProp( "Scale" ))->setVector( Ogre::Vector3(0.005f,0.005f,0.005f) );

        // set frustum
        QObject::connect(this, SIGNAL(setFrustum(const float&,const float&,const float&,const float&)), frustum_viewer_list_["head_left"], SLOT(setFrustum(const float&,const float&,const float&,const float&)));

        // create the window for cartesian motion
        cartesian_config_widget_ = new QWidget();
        cartesian_config_widget_->setStyleSheet("font: 8pt \"MS Shell Dlg 2\";");

        cartesian_use_collision_ = new QCheckBox("Use Collision Avoidance");

        cartesian_keep_orientation_ = new QCheckBox("Keep Endeffector Orientation");

        QPushButton* cartesian_send_left_ = new QPushButton("Send to left arm");
        QObject::connect(cartesian_send_left_, SIGNAL(clicked()), this, SLOT(sendCartesianLeft()));
        cartesian_send_left_->setStyleSheet("font: 8pt \"MS Shell Dlg 2\";");
        QPushButton* cartesian_send_right_ = new QPushButton("Send to right arm");
        QObject::connect(cartesian_send_right_, SIGNAL(clicked()), this, SLOT(sendCartesianRight()));
        cartesian_send_right_->setStyleSheet("font: 8pt \"MS Shell Dlg 2\";");

        QHBoxLayout* cartesian_button_layout_ = new QHBoxLayout();
        cartesian_button_layout_->setMargin(0);
        cartesian_button_layout_->addWidget(cartesian_send_left_);
        cartesian_button_layout_->addWidget(cartesian_send_right_);

        QVBoxLayout* cartesian_layout_ = new QVBoxLayout();
        cartesian_layout_->setMargin(3);
        cartesian_layout_->setSpacing(3);
        cartesian_layout_->addWidget(cartesian_use_collision_);
        cartesian_layout_->addWidget(cartesian_keep_orientation_);
        cartesian_layout_->addLayout(cartesian_button_layout_);

        cartesian_config_widget_->setLayout(cartesian_layout_);
        cartesian_config_widget_->setWindowFlags(Qt::WindowStaysOnTopHint | Qt::CustomizeWindowHint | Qt::WindowTitleHint);
        cartesian_config_widget_->hide();

        // and necessary publisher
        cartesian_plan_request_pub_ = nh_.advertise<flor_planning_msgs::CartesianMotionRequest>( "/flor/planning/upper_body/plan_cartesian_request", 1, false );

        // create the window for circular motion
        circular_config_widget_ = new QWidget();
        circular_config_widget_->setStyleSheet("font: 8pt \"MS Shell Dlg 2\";");

        circular_use_collision_ = new QCheckBox("Use Collision Avoidance");

        circular_keep_orientation_ = new QCheckBox("Keep Endeffector Orientation");

        QLabel* circular_angle_label_ = new QLabel("Rotation");
        circular_angle_ = new QDoubleSpinBox();
        circular_angle_->setDecimals(2);
        circular_angle_->setMaximum(1080);
        circular_angle_->setMinimum(-1080);

        QHBoxLayout* circular_angle_layout_ = new QHBoxLayout();
        circular_angle_layout_->setMargin(0);
        circular_angle_layout_->addWidget(circular_angle_label_);
        circular_angle_layout_->addWidget(circular_angle_);

        QPushButton* circular_send_left_ = new QPushButton("Send to left arm");
        QObject::connect(circular_send_left_, SIGNAL(clicked()), this, SLOT(sendCircularLeft()));
        circular_send_left_->setStyleSheet("font: 8pt \"MS Shell Dlg 2\";");
        QPushButton* circular_send_right_ = new QPushButton("Send to right arm");
        QObject::connect(circular_send_right_, SIGNAL(clicked()), this, SLOT(sendCircularRight()));
        circular_send_right_->setStyleSheet("font: 8pt \"MS Shell Dlg 2\";");

        QHBoxLayout* circular_button_layout_ = new QHBoxLayout();
        circular_button_layout_->setMargin(0);
        circular_button_layout_->addWidget(circular_send_left_);
        circular_button_layout_->addWidget(circular_send_right_);

        QVBoxLayout* circular_layout_ = new QVBoxLayout();
        circular_layout_->setMargin(3);
        circular_layout_->setSpacing(3);
        circular_layout_->addWidget(circular_use_collision_);
        circular_layout_->addWidget(circular_keep_orientation_);
        circular_layout_->addLayout(circular_angle_layout_);
        circular_layout_->addLayout(circular_button_layout_);

        circular_config_widget_->setLayout(circular_layout_);
        circular_config_widget_->setWindowFlags(Qt::WindowStaysOnTopHint | Qt::CustomizeWindowHint | Qt::WindowTitleHint);
        circular_config_widget_->hide();

        // and necessary publisher
        circular_plan_request_pub_ = nh_.advertise<flor_planning_msgs::CircularMotionRequest>( "/flor/planning/upper_body/plan_circular_request", 1, false );

        // subscribe to the topic sent by the ghost widget
        send_cartesian_sub_ = nh_.subscribe<std_msgs::Bool>( "/flor/ocs/send_cartesian", 5, &Base3DView::processSendCartesian, this );
        send_ghost_pelvis_pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>( "/flor/ocs/ghost/set_pose", 5, &Base3DView::processGhostPelvisPose, this );

        // create publisher and subscriber for object selection
        // PUBLISHER WILL BE USED BY THE RIGHT/DOUBLE CLICK TO INFORM WHICH TEMPLATE/HAND/OBJECT HAS BEEN selected
        // SUBSCRIBER WILL BE USED TO CHANGE VISIBILITY OF THE OBJECT THAT IS BEING USED (E.G., TALK TO TEMPLATE DISPLAY AND SET VISIBILITY OF MARKERS)
        select_object_pub_ = nh_.advertise<flor_ocs_msgs::OCSObjectSelection>( "/flor/ocs/object_selection", 1, false );
        select_object_sub_ = nh_.subscribe<flor_ocs_msgs::OCSObjectSelection>( "/flor/ocs/object_selection", 5, &Base3DView::processObjectSelection, this );

        // finally the key events
        key_event_sub_ = nh_.subscribe<flor_ocs_msgs::OCSKeyEvent>( "/flor/ocs/key_event", 5, &Base3DView::processNewKeyEvent, this );
        hotkey_relay_sub_ = nh_.subscribe<flor_ocs_msgs::OCSHotkeyRelay>( "/flor/ocs/hotkey_relay", 5, &Base3DView::processHotkeyRelayMessage, this );

        // create camera subscriber so we can control the camera from outside
        camera_transform_sub_ = nh_.subscribe<flor_ocs_msgs::OCSCameraTransform>( "/flor/ocs/set_camera_transform", 5, &Base3DView::processCameraTransform, this );


    }

    // Connect the 3D selection tool to
    QObject::connect(this, SIGNAL(queryContext(int,int)), selection_3d_display_, SLOT(queryContext(int,int)));
    QObject::connect(selection_3d_display_, SIGNAL(setContext(int,std::string)), this, SLOT(setContext(int,std::string)));

    // connect the 3d selection tool to its display
    QObject::connect(this, SIGNAL(setRenderPanel(rviz::RenderPanel*)), selection_3d_display_, SLOT(setRenderPanel(rviz::RenderPanel*)));
    Q_EMIT setRenderPanel(this->render_panel_);
    QObject::connect(selection_3d_display_, SIGNAL(newSelection(Ogre::Vector3)), this, SLOT(newSelection(Ogre::Vector3)));
    QObject::connect(selection_3d_display_, SIGNAL(setSelectionRay(Ogre::Ray)), this, SLOT(setSelectionRay(Ogre::Ray)));
    QObject::connect(this, SIGNAL(resetSelection()), selection_3d_display_, SLOT(resetSelection()));
    QObject::connect(this, SIGNAL(setMarkerScale(float)), selection_3d_display_, SLOT(setMarkerScale(float)));
    QObject::connect(this, SIGNAL(setMarkerPosition(float,float,float)), selection_3d_display_, SLOT(setMarkerPosition(float,float,float)));

    // handles mouse events without rviz::tool
    mouse_event_handler_ = new vigir_ocs::MouseEventHandler();
    QObject::connect(render_panel_, SIGNAL(signalMousePressEvent(QMouseEvent*)), mouse_event_handler_, SLOT(mousePressEvent(QMouseEvent*)));
    QObject::connect(render_panel_, SIGNAL(signalMouseReleaseEvent(QMouseEvent*)), mouse_event_handler_, SLOT(mouseReleaseEvent(QMouseEvent*)));
    QObject::connect(render_panel_, SIGNAL(signalMouseDoubleClickEvent(QMouseEvent*)), mouse_event_handler_, SLOT(mouseDoubleClick(QMouseEvent*)));
    QObject::connect(mouse_event_handler_, SIGNAL(mouseLeftButtonCtrl(bool,int,int)), selection_3d_display_, SLOT(raycastRequest(bool,int,int)));//SLOT(createMarker(bool,int,int))); // RAYCAST -> need createMarkerOnboard that sends raycast query
    QObject::connect(mouse_event_handler_, SIGNAL(mouseLeftButtonShift(bool,int,int)), selection_3d_display_, SLOT(raycastRequestROI(bool,int,int)));//SLOT(createROISelection(bool,int,int)));
    QObject::connect(mouse_event_handler_, SIGNAL(mouseRightButton(bool,int,int)), this, SLOT(createContextMenu(bool,int,int)));
    QObject::connect(mouse_event_handler_, SIGNAL(signalMouseLeftDoubleClick(int,int)), this, SLOT(selectOnDoubleClick(int,int)));

    Q_FOREACH( QWidget* sp, findChildren<QWidget*>() ) {
        sp->installEventFilter( this );
        sp->setMouseTracking( true );
    }

//    position_widget_ = new QWidget(this);
//    position_widget_->setStyleSheet("background-color: rgb(0, 0, 0);color: rgb(108, 108, 108);border-color: rgb(0, 0, 0);");
//    position_widget_->setMaximumHeight(18);
//    position_label_ = new QLineEditSmall("",position_widget_);
//    position_label_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
//    position_label_->setReadOnly(true);
//    position_label_->setStyleSheet("background-color: rgb(0, 0, 0);font: 8pt \"MS Shell Dlg 2\";color: rgb(108, 108, 108);border-color: rgb(0, 0, 0);");
//    position_label_->setFrame(false);

    reset_view_button_ = new QPushButton("Center On Robot", this);
    reset_view_button_->setStyleSheet("font: 8pt \"MS Shell Dlg 2\";background-color: rgb(0, 0, 0);color: rgb(108, 108, 108);border-color: rgb(0, 0, 0);");
    reset_view_button_->setMaximumSize(100,20);
    reset_view_button_->adjustSize();
   // reset_view_button_->move(0,300);
    QObject::connect(reset_view_button_, SIGNAL(clicked()), this, SLOT(resetView()));

    stop_button_ = new QPushButton("STOP",this);
    stop_button_->setStyleSheet("font: 100pt \"MS Shell Dlg 2\";background-color: red;color: white;border-color: red;");
    stop_button_->setMaximumSize(400,300);
    stop_button_->adjustSize();
    QObject::connect(stop_button_, SIGNAL(clicked()), this, SIGNAL(emergencyStop()));
    stop_button_->setVisible(false);

//    QHBoxLayout* position_layout = new QHBoxLayout();
//    position_layout->setSpacing(0);
//    position_layout->setMargin(0);
//    position_layout->addWidget(reset_view_button_);
   // position_layout->addWidget(position_label_);
   // position_widget_->setLayout(position_layout);
    main_layout->setMargin(0);
    main_layout->setSpacing(0);
  //  main_layout->addWidget(position_widget_);

    XmlRpc::XmlRpcValue   hand_T_palm;

    nh_.getParam("/l_hand_tf/hand_T_palm", hand_T_palm);
    l_hand_T_palm_.setOrigin(tf::Vector3(static_cast<double>(hand_T_palm[0]),static_cast<double>(hand_T_palm[1]),static_cast<double>(hand_T_palm[2])));
    l_hand_T_palm_.setRotation(tf::Quaternion(static_cast<double>(hand_T_palm[3]),static_cast<double>(hand_T_palm[4]),static_cast<double>(hand_T_palm[5]),static_cast<double>(hand_T_palm[6])));

    nh_.getParam("/r_hand_tf/hand_T_palm", hand_T_palm);
    r_hand_T_palm_.setOrigin(tf::Vector3(static_cast<double>(hand_T_palm[0]),static_cast<double>(hand_T_palm[1]),static_cast<double>(hand_T_palm[2])));
    r_hand_T_palm_.setRotation(tf::Quaternion(static_cast<double>(hand_T_palm[3]),static_cast<double>(hand_T_palm[4]),static_cast<double>(hand_T_palm[5]),static_cast<double>(hand_T_palm[6])));

    nh_.getParam("/l_hand_tf/hand_T_marker", hand_T_palm);
    l_hand_T_marker_.setOrigin(tf::Vector3(static_cast<double>(hand_T_palm[0]),static_cast<double>(hand_T_palm[1]),static_cast<double>(hand_T_palm[2])));
    l_hand_T_marker_.setRotation(tf::Quaternion(static_cast<double>(hand_T_palm[3]),static_cast<double>(hand_T_palm[4]),static_cast<double>(hand_T_palm[5]),static_cast<double>(hand_T_palm[6])));

    nh_.getParam("/r_hand_tf/hand_T_marker", hand_T_palm);
    r_hand_T_marker_.setOrigin(tf::Vector3(static_cast<double>(hand_T_palm[0]),static_cast<double>(hand_T_palm[1]),static_cast<double>(hand_T_palm[2])));
    r_hand_T_marker_.setRotation(tf::Quaternion(static_cast<double>(hand_T_palm[3]),static_cast<double>(hand_T_palm[4]),static_cast<double>(hand_T_palm[5]),static_cast<double>(hand_T_palm[6])));

    nh_.getParam("/l_hand_type", l_hand_type);
    nh_.getParam("/r_hand_type", r_hand_type);

    // set background color to rviz default
    render_panel_->getViewport()->setBackgroundColour(rviz::qtToOgre(QColor(48,48,48)));

    // advertise publisher for camera transform
    camera_transform_pub_ = nh_.advertise<flor_ocs_msgs::OCSCameraTransform>( "/flor/ocs/camera_transform", 1, false );

    //build context menu
    addBase3DContextElements();

    // this is only used to make sure we close window if ros::shutdown has already been called
    timer.start(33, this);
}

// Destructor.
Base3DView::~Base3DView()
{
    delete manager_;
}

void Base3DView::timerEvent(QTimerEvent *event)
{
    // check if ros is still running; if not, just kill the application
    if(!ros::ok())
        qApp->quit();

    //set button on corner of views on any size
    reset_view_button_->setGeometry(0,this->geometry().bottomLeft().y()-18,100,20);

    // make sure the selection point is visible
    //position_widget_->setGeometry(0,
    //                              this->geometry().bottomLeft().y()-18,
    //                              this->geometry().bottomRight().x()-this->geometry().bottomLeft().x()+2,
    //                              20);

    publishCameraTransform();

    //std::bitset<32> x(render_panel_->getViewport()->getVisibilityMask());
    //std::cout << x << std::endl;

    //render_panel_->getRenderWindow()->update(true);

    //float lastFPS, avgFPS, bestFPS, worstFPS;
    //Ogre::RenderTarget::FrameStats stats = render_panel_->getRenderWindow()->getStatistics();
    //std::cout << "View (" << view_id_ << "): " << stats.lastFPS << ", " << stats.avgFPS << ", " << stats.bestFrameTime << ", " << stats.worstFrameTime << ", " << stats.triangleCount << std::endl;

    // no need to spin as rviz is already doing that for us.
    //ros::spinOnce();
}

void Base3DView::processCameraTransform(const flor_ocs_msgs::OCSCameraTransform::ConstPtr& msg)
{
    Ogre::Camera* camera = this->render_panel_->getCamera();
    Ogre::Vector3 pos;
    pos.x = msg->pose.position.x;
    pos.y = msg->pose.position.y;
    pos.z = msg->pose.position.z;
    camera->setPosition(pos);
    Ogre::Quaternion orientation;
    orientation.x = msg->pose.orientation.x;
    orientation.y = msg->pose.orientation.y;
    orientation.z = msg->pose.orientation.z;
    orientation.w = msg->pose.orientation.w;
    camera->setOrientation(orientation);
}

void Base3DView::publishCameraTransform()
{
    // send the camera transform associated with this render panel to whoever connects to this
    Ogre::Camera* camera = this->render_panel_->getCamera();
    Ogre::Vector3 position = camera->getPosition();
    Ogre::Quaternion orientation = camera->getOrientation();

    // if it's ortho, we need to calculate distance from the viewing plane 0 again
    if(camera->getProjectionType() == Ogre::PT_ORTHOGRAPHIC)
    {
        // calculate projection matrix for orthographic window
        Ogre::Matrix4 m = camera->getProjectionMatrix();
        float near   =  (1+m[2][3])/m[2][2];
        float far    = -(1-m[2][3])/m[2][2];
        float bottom =  (1-m[1][3])/m[1][1];
        float top    = -(1+m[1][3])/m[1][1];
        float left   = -(1+m[0][3])/m[0][0];
        float right  =  (1-m[0][3])/m[0][0];
        //std::cout << "ortho:\n\t" << left << "\n\t" << right << "\n\t" << bottom << "\n\t" << top << "\n\t" << near << "\n\t" << far << std::endl;
        // get the new distance from 0 relative to the viewing plane
        if(position.z == 500)
            position.z = fabs(bottom)+fabs(top);
        else if(position.y == -500)
            position.y = fabs(bottom)+fabs(top);
        else if(position.x == 500)
            position.x = fabs(bottom)+fabs(top);
    }

    flor_ocs_msgs::OCSCameraTransform cmd;
    cmd.widget_name = widget_name_;
    cmd.view_id = view_id_;
    cmd.pose.position.x = position.x;
    cmd.pose.position.y = position.y;
    cmd.pose.position.z = position.z;
    cmd.pose.orientation.x = orientation.x;
    cmd.pose.orientation.y = orientation.y;
    cmd.pose.orientation.z = orientation.z;
    cmd.pose.orientation.w = orientation.w;
    camera_transform_pub_.publish(cmd);
}

void Base3DView::updateRenderMask( bool mask )
{
    manager_->updateRenderMask( view_id_, mask );
}

void Base3DView::robotModelToggled( bool selected )
{
    robot_model_->setEnabled( selected );
}

void Base3DView::graspModelToggled( bool selected )
{
    left_grasp_hand_model_->setEnabled( selected );
    right_grasp_hand_model_->setEnabled( selected );
}

void Base3DView::templatesToggled( bool selected )
{
    template_display_->setEnabled( selected );
}

void Base3DView::requestedPointCloudToggled( bool selected )
{
    // we can't enable/disable point cloud requests, since the existing ones are lost if we disable them
    raycast_point_cloud_viewer_->subProp("Alpha")->setValue(selected ? 1.0f : 0.0f);
}

void Base3DView::lidarPointCloudToggled( bool selected )
{
    //lidar_point_cloud_viewer_->setEnabled( selected );
    region_point_cloud_viewer_->subProp("Alpha")->setValue(selected ? 1.0f : 0.0f);
}

void Base3DView::stereoPointCloudToggled( bool selected )
{
    //stereo_point_cloud_viewer_->setEnabled( selected );
    stereo_point_cloud_viewer_->subProp("Alpha")->setValue(selected ? 1.0f : 0.0f);
}

void Base3DView::laserScanToggled( bool selected )
{
    laser_scan_->setEnabled( selected );
}

void Base3DView::ft_sensorToggled(bool selected )
{
    left_ft_sensor_->setEnabled( selected );
    right_ft_sensor_->setEnabled( selected );
}

void Base3DView::markerArrayToggled( bool selected )
{
    // we can't enable/disable octomaps, since the existing ones are lost if we disable them
    //octomap_->subProp("Alpha")->setValue(selected ? 1.0f : 0.0f);
    octomap_->setEnabled(selected);
}

void Base3DView::gridMapToggled( bool selected )
{
    // we can't enable/disable gridmaps, since the existing ones are lost if we disable them
    visualize_grid_map_ = selected;
    if(!selected)
    {
        for(int i = 0; i < ground_map_.size(); i++)
        {
            ground_map_[i]->subProp("Alpha")->setValue(0.0f);
        }
    }
    else
    {
        float alpha = 1.0f, step = (0.95f/stored_maps_);
        unsigned short priority = stored_maps_-1;
        for(int i = ground_map_.size()-1; i >= 0; i--,alpha-=step)
        {
            //std::cout << i << " " << alpha << std::endl;
            ground_map_[i]->subProp("Alpha")->setValue(alpha);
            ((rviz::MapDisplayCustom*)ground_map_[i])->setPriority(priority--);
        }
    }
}

void Base3DView::footstepPlanningToggled( bool selected )
{
    goal_pose_walk_->setEnabled( selected );
    goal_pose_step_->setEnabled( selected );
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

void Base3DView::defineWalkPosePressed()
{
    //set_goal_tool_->getPropertyContainer()->subProp( "Topic" )->setValue( "/goal_pose_walk" );
    manager_->getToolManager()->setCurrentTool( set_walk_goal_tool_ );
}

void Base3DView::defineStepPosePressed()
{
    //set_goal_tool_->getPropertyContainer()->subProp( "Topic" )->setValue( "/goal_pose_step" );
    manager_->getToolManager()->setCurrentTool( set_step_goal_tool_ );
}

void Base3DView::processGoalPose(const geometry_msgs::PoseStamped::ConstPtr &pose, int type)
{
    last_footstep_plan_type_ = type;
}

void Base3DView::processPointCloud( const sensor_msgs::PointCloud2::ConstPtr& pc )
{
    //std::cout << "point cloud received" << std::endl;
    pcl::PointCloud<pcl::PointXYZ> pclCloud;
    pcl::fromROSMsg(*pc, pclCloud);
    Eigen::Vector4f centroid;
    if(pcl::compute3DCentroid(pclCloud,centroid) != 0)
    {
        geometry_msgs::Point point;
        point.x = centroid[0];
        point.y = centroid[1];
        point.z = centroid[2];
        //std::cout << "centroid: " << point.x << ", " << point.y << ", " << point.z << std::endl;
        global_selection_pos_pub_.publish(point);
    }
}

void Base3DView::processNewSelection( const geometry_msgs::Point::ConstPtr& pose )
{
    Q_EMIT setMarkerPosition(pose->x,pose->y,pose->z);
}

void Base3DView::processControlMode( const flor_control_msgs::FlorControlMode::ConstPtr& msg )
{
    flor_atlas_current_mode_ = msg->behavior;
}

void Base3DView::newSelection( Ogre::Vector3 position )
{
    QString new_label = QString::number(position.x,'f',2)+", "+QString::number(position.y,'f',2)+", "+QString::number(position.z,'f',2);
    //position_label_->setText(new_label);
    Q_EMIT sendPositionText(new_label);

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
    //std::cout << "adding template" << std::endl;

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
        //std::cout << "adding waypoint" << std::endl;

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

void Base3DView::setTemplateTree(QTreeWidget * root)
{
    if(root != NULL)
    {
        templateRoot = root;
        addTemplatesToContext();
    }
}

void Base3DView::addBase3DContextElements()
{
    contextMenuItem * separator = new contextMenuItem();
    separator->name = "Separator";

    selectMenu = makeContextChild("Select Template",boost::bind(&Base3DView::selectContextMenu,this),NULL,contextMenuItems);

    leftArmMenu = makeContextChild("Select Left Arm",boost::bind(&Base3DView::selectLeftArm,this),NULL,contextMenuItems);
    rightArmMenu = makeContextChild("Select Right Arm",boost::bind(&Base3DView::selectRightArm,this),NULL,contextMenuItems);

    addToContextVector(separator);

    snapHandMenu = makeContextChild("Snap Hand to Ghost",boost::bind(&Base3DView::snapHandGhost,this),NULL,contextMenuItems);

    addToContextVector(separator);

    insertTemplateMenu = makeContextParent("Insert Template",contextMenuItems);
    removeTemplateMenu = makeContextChild("Remove Template",boost::bind(&Base3DView::removeTemplateContextMenu,this),NULL,contextMenuItems);

    addToContextVector(separator);

    lockLeftMenu = makeContextChild("Lock Left Arm to Template",boost::bind(&Base3DView::setTemplateGraspLock,this,flor_ocs_msgs::OCSObjectSelection::LEFT_ARM),NULL,contextMenuItems);
    lockRightMenu = makeContextChild("Lock Right Arm to Template",boost::bind(&Base3DView::setTemplateGraspLock,this,flor_ocs_msgs::OCSObjectSelection::RIGHT_ARM),NULL,contextMenuItems);
    unlockArmsMenu = makeContextChild("Unlock Arms",boost::bind(&Base3DView::setTemplateGraspLock,this,-1),NULL,contextMenuItems);

    addToContextVector(separator);

    cartesianMotionMenu = makeContextParent("Cartesian Motion", contextMenuItems);

    createCartesianMarkerMenu = makeContextChild("Create Cartesian Motion Marker",boost::bind(&Base3DView::createCartesianContextMenu,this),cartesianMotionMenu,contextMenuItems);
    removeCartesianMarkerMenu = makeContextChild("Remove All Markers",boost::bind(&Base3DView::removeCartesianContextMenu,this),cartesianMotionMenu,contextMenuItems);

    circularMotionMenu = makeContextParent("Circular Motion", contextMenuItems);

    createCircularMarkerMenu = makeContextChild("Create Circular Motion Marker",boost::bind(&Base3DView::createCircularContextMenu,this),circularMotionMenu,contextMenuItems);
    removeCircularMarkerMenu = makeContextChild("Remove marker",boost::bind(&Base3DView::removeCircularContextMenu,this),circularMotionMenu,contextMenuItems);

    addToContextVector(separator);


    footstepPlanMenuWalk = makeContextChild(QString("Execute Footstep Plan - ")+(last_footstep_plan_type_ == 1 ? "Step" : "Walk"),boost::bind(&Base3DView::executeFootstepPlanContextMenu,this),NULL,contextMenuItems);
    footstepPlanMenuWalkManipulation = makeContextChild(QString("Execute Footstep Plan - ")+(last_footstep_plan_type_ == 1 ? "Step" : "Walk")+" Manipulate",boost::bind(&Base3DView::executeFootstepPlanContextMenu,this),NULL,contextMenuItems);

    addToContextVector(separator);
}

void Base3DView::addTemplatesToContext()
{
    if(templateRoot != NULL)
    {
        QTreeWidgetItemIterator it(templateRoot);
        while (*it)
        {
            contextMenuItem * localParent;
            if(!(*it)->text(0).contains(".mesh")) //only templates have .mesh
            {
                //will be traversed before children, can be used locally for children as well
                //next parent won't be called until after all children, preorder traversal
                //need to make manually as we have a parent with a parent(not handled by built-in method)
                localParent = new contextMenuItem();
                localParent->hasChildren = true;
                localParent->name = (*it)->text(0);
                localParent->parent = insertTemplateMenu;
                addToContextVector(localParent);
            }
            else
            {
                //build path to template for insertion
                QString path = localParent->name + "/" + (*it)->text(0);
                // ROS_ERROR("path %s",qPrintable(path));
                makeContextChild((*it)->text(0),boost::bind(&Base3DView::contextInsertTemplate,this,path),localParent,contextMenuItems);
            }
            ++it;
        }
    }
}


void Base3DView::contextInsertTemplate(QString path)
{
    insertTemplate(path);
}

contextMenuItem * Base3DView::makeContextParent(QString name,std::vector<contextMenuItem * > &contextMenuElements)
{
    contextMenuItem * parent = new contextMenuItem();
    parent->name = name;
    parent->hasChildren = true;
    contextMenuElements.push_back(parent);
    return parent;
}

contextMenuItem * Base3DView::makeContextChild(QString name,boost::function<void()> function,contextMenuItem * parent,std::vector<contextMenuItem * > &contextMenuElements)
{
    contextMenuItem * child = new contextMenuItem();
    child->name = name;
    child->function = function;
    child->parent = parent;
    child->hasChildren = false;
    contextMenuElements.push_back(child);
    return child;
}

void Base3DView::selectOnDoubleClick(int x, int y)
{
    Q_EMIT queryContext(x,y);    
    if(active_context_name_.find("LeftArm") != std::string::npos)
        selectLeftArm();
    else if(active_context_name_.find("RightArm") != std::string::npos)
        selectRightArm();
    else if(active_context_name_.find("template") != std::string::npos)
        selectContextMenu();
}


void Base3DView::createContextMenu(bool, int x, int y)
{
    initializing_context_menu_++;

    context_menu_selected_item_ = NULL;

    context_menu_.clear();
    context_menu_.setTitle("Base Menu");

    context_menu_.setStyleSheet("font-size:11px;");

    // first we need to query the 3D scene to retrieve the context
    Q_EMIT queryContext(x,y);
    // context is stored in the active_context_ variable
    std::cout << "Active context: " << active_context_ << std::endl;

    addToContextMenuFromVector();

    //insert stuff in constructor
    //have special case for empty vector item. insert separator when found
    //context_menu_.addAction("Insert Template");

    ROS_ERROR("CONTEXT: %s",active_context_name_.c_str());

    //arms selection
    if(active_context_name_.find("LeftArm") != std::string::npos)
    {
        context_menu_.removeAction(rightArmMenu->action);
    }
    else if(active_context_name_.find("RightArm") != std::string::npos)
    {
        context_menu_.removeAction(leftArmMenu->action);
    }
    else //neither arm selected
    {
        context_menu_.removeAction(rightArmMenu->action);
        context_menu_.removeAction(leftArmMenu->action);
    }

    //lock/unlock arms context items
    if(active_context_name_.find("template") == std::string::npos)
    {
        //remove context items as not needed
        context_menu_.removeAction(removeTemplateMenu->action);
        context_menu_.removeAction(selectMenu->action);
        context_menu_.removeAction(lockLeftMenu->action);
        context_menu_.removeAction(lockRightMenu->action);
    }

    if((ghost_pose_source_[flor_ocs_msgs::OCSObjectSelection::RIGHT_ARM] && ghost_world_lock_[flor_ocs_msgs::OCSObjectSelection::RIGHT_ARM]) || (ghost_pose_source_[flor_ocs_msgs::OCSObjectSelection::LEFT_ARM] && ghost_world_lock_[flor_ocs_msgs::OCSObjectSelection::LEFT_ARM]))
    {
        //show only unlock
        context_menu_.removeAction(lockLeftMenu->action);
        context_menu_.removeAction(lockRightMenu->action);
    }
    else
    {
        //dont show unlock.. both arms are free and ready to be locked
        context_menu_.removeAction(unlockArmsMenu->action);
    }

    if(flor_atlas_current_mode_ == 0 || flor_atlas_current_mode_ == 100)
    {
        footstepPlanMenuWalk->action->setEnabled(true);
        footstepPlanMenuWalkManipulation->action->setEnabled(true);
    }
    else
    {
        context_menu_.removeAction(footstepPlanMenuWalk->action);
        context_menu_.removeAction(footstepPlanMenuWalkManipulation->action);
    }

    if(cartesian_marker_list_.size() == 0)
    {
        cartesianMotionMenu->menu->removeAction(removeCartesianMarkerMenu->action);
        //removeCartesianMarkerMenu->action->setEnabled(false);
    }
    else
        removeCartesianMarkerMenu->action->setEnabled(true);

    if(circular_marker_ != NULL)
    {        
        createCircularMarkerMenu->action->setEnabled(false);
        removeCircularMarkerMenu->action->setEnabled(true);
    }
    else if(circular_marker_ == NULL)
    {
        createCircularMarkerMenu->action->setEnabled(true);
        circularMotionMenu->menu->removeAction(removeCircularMarkerMenu->action);
        //removeCircularMarkerMenu->action->setEnabled(false);
    }


    if(initializing_context_menu_ == 1)
        processContextMenu(x, y);

    initializing_context_menu_--;
}

void Base3DView::addToContextVector(contextMenuItem* item)
{
    contextMenuItems.push_back(item); //insert new element
}

void Base3DView::addToContextMenuFromVector()
{
    for(int i=0;i<contextMenuItems.size();i++)
    {
        if(contextMenuItems[i]->name == "Separator")
        {
            context_menu_.addSeparator();
            continue;
        }
        //top level menu item
        if(contextMenuItems[i]->parent == NULL)
        {
            if(contextMenuItems[i]->hasChildren)
            {
                QMenu * menu = context_menu_.addMenu(contextMenuItems[i]->name);
                contextMenuItems[i]->menu = menu;
            }
            else //no children, must be action
            {
                QAction * action = context_menu_.addAction(contextMenuItems[i]->name);
                contextMenuItems[i]->action = action;
            }
        }
        else // can guarantee parent has already been added provided elements were added in correct order to vector
        {
            if(contextMenuItems[i]->hasChildren)
            {
                QMenu * menu = contextMenuItems[i]->parent->menu->addMenu(contextMenuItems[i]->name);
                contextMenuItems[i]->menu = menu;
            }
            else
            {
                QAction * action = contextMenuItems[i]->parent->menu->addAction(contextMenuItems[i]->name);
                contextMenuItems[i]->action = action;
            }
        }
    }
    Q_EMIT updateMainViewItems();
}

void Base3DView::processContextMenuVector()
{
    for(int i =0; i<contextMenuItems.size();i++)
    {
        //check parent if it exists?
        if(contextMenuItems[i]->parent != NULL && ((QMenu*)context_menu_selected_item_->parent())->title() == contextMenuItems[i]->parent->name )
        {
            //check actual item
            if( context_menu_selected_item_->text() == contextMenuItems[i]->name)
            {
                if(contextMenuItems[i]->function != NULL)
                {
                    contextMenuItems[i]->function(); //call binded function
                }
            }
        }
        else // no parent, must still check item
        {
            if( context_menu_selected_item_->text() == contextMenuItems[i]->name)
            {
                if(contextMenuItems[i]->function != NULL)
                {
                    contextMenuItems[i]->function(); //call binded function
                }
            }
        }
    }
}

//callback functions for context Menu
void Base3DView::insertTemplateContextMenu()
{
    if(!selected_template_path_.isEmpty())
    {
        insertTemplate(selected_template_path_);
    }

}

void Base3DView::removeTemplateContextMenu()
{
    int start = active_context_name_.find(" ")+1;
    int end = active_context_name_.find(".");
    QString template_number(active_context_name_.substr(start, end-start).c_str());
    ROS_INFO("%d %d %s",start,end,template_number.toStdString().c_str());
    bool ok;
    int t = template_number.toInt(&ok);
    if(ok) removeTemplate(t);
}

void Base3DView::selectContextMenu()
{
    int id;
    if((id = findObjectContext("template")) != -1)
        selectTemplate(id);
}

int Base3DView::findObjectContext(std::string obj_type)
{
    if(active_context_name_.find(obj_type) != std::string::npos)
    {
        int start = active_context_name_.find(" ")+1;
        int end = active_context_name_.find(".");
        QString template_number(active_context_name_.substr(start, end-start).c_str());
        ROS_INFO("%d %d %s",start,end,template_number.toStdString().c_str());
        bool ok;
        int t = template_number.toInt(&ok);
        if(ok) return t;
        return -1;

    }
}

void Base3DView::selectTemplate(int id)
{
    flor_ocs_msgs::OCSObjectSelection cmd;
    cmd.type = flor_ocs_msgs::OCSObjectSelection::TEMPLATE;
    cmd.id = id;
    select_object_pub_.publish(cmd);
}

void Base3DView::selectLeftArm()
{
    flor_ocs_msgs::OCSObjectSelection cmd;
    cmd.type = flor_ocs_msgs::OCSObjectSelection::END_EFFECTOR;
    cmd.id = flor_ocs_msgs::OCSObjectSelection::LEFT_ARM;
    select_object_pub_.publish(cmd);
}

void Base3DView::selectRightArm()
{
    flor_ocs_msgs::OCSObjectSelection cmd;
    cmd.type = flor_ocs_msgs::OCSObjectSelection::END_EFFECTOR;
    cmd.id = flor_ocs_msgs::OCSObjectSelection::RIGHT_ARM;
    select_object_pub_.publish(cmd);
}

void Base3DView::snapHandGhost()
{
    snap_left_hand_to_ghost_ = true;
    snap_right_hand_to_ghost_ = true;
}

// this function will toggle the template grasp lock
void Base3DView::setTemplateGraspLock(int arm)
{
    int id = findObjectContext("template");
    if (arm == -1) // unlocks both arms
    {
        selectTemplate(id);

        ghost_pose_source_[flor_ocs_msgs::OCSObjectSelection::LEFT_ARM] = false;
        ghost_world_lock_[flor_ocs_msgs::OCSObjectSelection::LEFT_ARM] = false;


        ghost_pose_source_[flor_ocs_msgs::OCSObjectSelection::RIGHT_ARM] = false;
        ghost_world_lock_[flor_ocs_msgs::OCSObjectSelection::RIGHT_ARM] = false;
        return;
    }
    else if(id != -1) //locks arm
    {
        selectTemplate(id);

        ghost_pose_source_[arm] = true;
        ghost_world_lock_[arm] = true;
    }

}

void Base3DView::processObjectSelection(const flor_ocs_msgs::OCSObjectSelection::ConstPtr& msg)
{
    // disable all template markers
    Q_EMIT enableTemplateMarkers( false );

    // disable all robot IK markers
    for( int i = 0; i < im_ghost_robot_.size(); i++ )
        im_ghost_robot_[i]->setEnabled( false );

    // enable loopback for both arms
    left_marker_moveit_loopback_ = true;
    right_marker_moveit_loopback_ = true;

    switch(msg->type)
    {
        case flor_ocs_msgs::OCSObjectSelection::END_EFFECTOR:
            // enable template marker
            if(msg->id == flor_ocs_msgs::OCSObjectSelection::LEFT_ARM)
                left_marker_moveit_loopback_ = false;
            else if(msg->id == flor_ocs_msgs::OCSObjectSelection::RIGHT_ARM)
                right_marker_moveit_loopback_ = false;
            im_ghost_robot_[msg->id]->setEnabled( true );
            break;
        case flor_ocs_msgs::OCSObjectSelection::TEMPLATE:
            // enable template marker
            Q_EMIT enableTemplateMarker( msg->id, true );
            break;
        case flor_ocs_msgs::OCSObjectSelection::FOOTSTEP:
        default:
            break;
    }
}

void Base3DView::executeFootstepPlanContextMenu()
{
    flor_control_msgs::FlorControlModeCommand cmd;
    if(context_menu_selected_item_->text().contains("Step Manipulate"))
        cmd.behavior = cmd.FLOR_STEP_MANI;
    else if(context_menu_selected_item_->text().contains("Walk Manipulate"))
        cmd.behavior = cmd.FLOR_WALK_MANI;
    else if(context_menu_selected_item_->text().contains("Step"))
        cmd.behavior = cmd.FLOR_STEP;
    else if(context_menu_selected_item_->text().contains("Walk"))
        cmd.behavior = cmd.FLOR_WALK;
    flor_mode_command_pub_.publish(cmd);
}

void Base3DView::createCartesianContextMenu()
{
    // if this is the first cartesian marker, create config window
    if(!cartesian_config_widget_->isVisible())
    {
        cartesian_config_widget_->move(QPoint(QCursor::pos().x()+5, QCursor::pos().y()+5));
        cartesian_config_widget_->show();
    }

    unsigned int id = cartesian_marker_list_.size();
    std::string pose_string = std::string("/cartesian_pose_")+boost::to_string((unsigned int)id);

    // Add cartesian marker
    rviz::Display* cartesian_marker = manager_->createDisplay( "rviz/InteractiveMarkers", (std::string("Cartesian Marker ")+boost::to_string((unsigned int)id)).c_str(), true );
    cartesian_marker->subProp( "Update Topic" )->setValue( (pose_string+std::string("/pose_marker/update")).c_str() );
    cartesian_marker->setEnabled( true );
    cartesian_marker->subProp( "Show Axes" )->setValue( true );
    cartesian_marker->subProp( "Show Visual Aids" )->setValue( true );
    cartesian_marker_list_.push_back(cartesian_marker);

    // Add it in front of the robot
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 1;
    pose.pose.position.y = 0;
    pose.pose.position.z = .2;
    pose.pose.orientation.x = 0;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = 0;
    pose.pose.orientation.w = 1;
    pose.header.frame_id = "/pelvis";
    transform(base_frame_,pose);

    geometry_msgs::Point pos;
    pos.x = pose.pose.position.x;
    pos.y = pose.pose.position.y;
    pos.z = pose.pose.position.z;

    flor_ocs_msgs::OCSInteractiveMarkerAdd marker;
    marker.name  = std::string("Cartesian Waypoint ")+boost::to_string((unsigned int)id);
    marker.topic = pose_string;
    marker.frame = base_frame_;
    marker.scale = 0.2;
    marker.point = pos;
    interactive_marker_add_pub_.publish(marker);

    cartesian_waypoint_list_.push_back(pose.pose);
}

void Base3DView::removeCartesianContextMenu()
{
    cartesian_config_widget_->hide();

    for(int i = 0; i < cartesian_marker_list_.size(); i++)
    {
        std_msgs::String topic;
        topic.data = std::string("/cartesian_pose_")+boost::to_string((unsigned int)i);
        interactive_marker_remove_pub_.publish(topic);
        // Displays can emit signals from other threads with self pointers.  We're
        // freeing the display now, so ensure no one is listening to those signals.
        cartesian_marker_list_[i]->disconnect();
        // Delete display later in case there are pending signals to it.
        cartesian_marker_list_[i]->deleteLater();
    }
    manager_->notifyConfigChanged();
    cartesian_marker_list_.clear();
    cartesian_waypoint_list_.clear();
}

void Base3DView::createCircularContextMenu()
{
    if(!circular_config_widget_->isVisible())
    {
        circular_config_widget_->move(QPoint(QCursor::pos().x()+5, QCursor::pos().y()+5));
        circular_config_widget_->show();
    }

    std::string pose_string = std::string("/circular_pose"); // one for each template

    // Add cartesian marker
    circular_marker_ = manager_->createDisplay( "rviz/InteractiveMarkers", "Circular Marker", true );
    circular_marker_->subProp( "Update Topic" )->setValue( (pose_string+std::string("/pose_marker/update")).c_str() );
    circular_marker_->setEnabled( true );
    circular_marker_->subProp( "Show Axes" )->setValue( true );
    circular_marker_->subProp( "Show Visual Aids" )->setValue( true );

    // Add it in front of the robot
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 1;
    pose.pose.position.y = 0;
    pose.pose.position.z = .2;
    pose.pose.orientation.x = 0;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = 0;
    pose.pose.orientation.w = 1;
    pose.header.frame_id = "/pelvis";
    transform(base_frame_,pose);

    geometry_msgs::Point pos;
    pos.x = pose.pose.position.x;
    pos.y = pose.pose.position.y;
    pos.z = pose.pose.position.z;

    flor_ocs_msgs::OCSInteractiveMarkerAdd marker;
    marker.name  = std::string("Center of Rotation");
    marker.topic = pose_string;
    marker.frame = base_frame_;
    marker.scale = 0.2;
    marker.point = pos;
    interactive_marker_add_pub_.publish(marker);

    circular_center_ = pose.pose;
}

void Base3DView::removeCircularContextMenu()
{
    circular_config_widget_->hide();

    std_msgs::String topic;
    topic.data = std::string("/circular_pose");
    interactive_marker_remove_pub_.publish(topic);
    // Displays can emit signals from other threads with self pointers.  We're
    // freeing the display now, so ensure no one is listening to those signals.
    circular_marker_->disconnect();
    // Delete display later in case there are pending signals to it.
    circular_marker_->deleteLater();
    manager_->notifyConfigChanged();
    circular_marker_ = NULL;
}
void Base3DView::processContextMenu(int x, int y)
{
    QPoint globalPos = this->mapToGlobal(QPoint(x,y));
    context_menu_selected_item_ = context_menu_.exec(globalPos);

    //std::cout << selectedItem << std::endl;
    if(context_menu_selected_item_ != NULL)
    {
        processContextMenuVector();
    }
}



void Base3DView::removeTemplate(int id)
{
    flor_ocs_msgs::OCSTemplateRemove cmd;

    cmd.template_id = id;

    // publish template to be removed
    template_remove_pub_.publish( cmd );
}

void Base3DView::setContext(int context, std::string name)
{
    active_context_ = context;
    active_context_name_ = name;
    //std::cout << "Active context: " << active_context_ << std::endl;
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
    if(ground_map_.size() > stored_maps_)
    {
        rviz::Display* tmp = ground_map_[0];
        tmp->setEnabled(false);
        delete tmp;
        manager_->notifyConfigChanged();
        ground_map_.erase(ground_map_.begin());
    }
    gridMapToggled(visualize_grid_map_);
}

// Moveit callback for left arm end effector
void Base3DView::processLeftArmEndEffector(const geometry_msgs::PoseStamped::ConstPtr &pose)
{
    // calculate pose of the marker based on interactive marker transform
    geometry_msgs::PoseStamped wrist_pose;
    calcWristTarget(*pose,l_hand_T_marker_,wrist_pose);

    // if the moveit loopback is enabled or we want to snap back to ghost, we update the interactive marker based a possible configuration returned by moveit
    if(left_marker_moveit_loopback_ || snap_left_hand_to_ghost_)
    {
        // reset snap flag
        snap_left_hand_to_ghost_ = false;

        // setup interactive markers if they haven't been already
        if(marker_published_ < 3)
            publishMarkers();

        // publishes the grasp hands pose
        publishHandPose("left",*pose);

        // update interactive marker pose
        flor_ocs_msgs::OCSInteractiveMarkerUpdate cmd;
        cmd.topic = "/l_arm_pose_marker";
        cmd.pose = wrist_pose;
        interactive_marker_update_pub_.publish(cmd);

        l_arm_marker_pose_pub_.publish(wrist_pose);

        //ROS_ERROR("LEFT ARM POSE:");
        //ROS_ERROR("  position: %.2f %.2f %.2f",cmd.pose.pose.position.x,cmd.pose.pose.position.y,cmd.pose.pose.position.z);
        //ROS_ERROR("  orientation: %.2f %.2f %.2f %.2f",cmd.pose.pose.orientation.w,cmd.pose.pose.orientation.x,cmd.pose.pose.orientation.y,cmd.pose.pose.orientation.z);

        // doesn't happen if in template lock mode
        if(!moving_pelvis_ && ghost_pose_source_[0] == 0)
            end_effector_pose_list_[cmd.topic] = cmd.pose;
    }

    // save last moveit return pose transformed to wrist CS
    // used for cartesian as well as checking if interactive marker == moveit
    last_l_arm_moveit_pose_ = wrist_pose.pose;

    updateHandColors();
}

// Moveit callback for right arm end effector
void Base3DView::processRightArmEndEffector(const geometry_msgs::PoseStamped::ConstPtr &pose)
{
    // calculate pose of the marker based on interactive marker transform
    geometry_msgs::PoseStamped wrist_pose;
    calcWristTarget(*pose,r_hand_T_marker_,wrist_pose);

    // if the moveit loopback is enabled or we want to snap back to ghost, we update the interactive marker based a possible configuration returned by moveit
    if(right_marker_moveit_loopback_ || snap_right_hand_to_ghost_)
    {
        // reset snap flag
        snap_right_hand_to_ghost_ = false;

        // setup interactive markers if they haven't been already
        if(marker_published_ < 3)
            publishMarkers();

        publishHandPose("right",*pose);

        // update interactive marker pose
        flor_ocs_msgs::OCSInteractiveMarkerUpdate cmd;
        cmd.topic = "/r_arm_pose_marker";
        cmd.pose = wrist_pose;
        interactive_marker_update_pub_.publish(cmd);

        r_arm_marker_pose_pub_.publish(wrist_pose);

        //ROS_ERROR("RIGHT ARM POSE:");
        //ROS_ERROR("  position: %.2f %.2f %.2f",cmd.pose.pose.position.x,cmd.pose.pose.position.y,cmd.pose.pose.position.z);
        //ROS_ERROR("  orientation: %.2f %.2f %.2f %.2f",cmd.pose.pose.orientation.w,cmd.pose.pose.orientation.x,cmd.pose.pose.orientation.y,cmd.pose.pose.orientation.z);

        // doesn't happen if in template lock mode
        if(!moving_pelvis_ && ghost_pose_source_[1] == 0)
            end_effector_pose_list_[cmd.topic] = cmd.pose;
    }

    // save last moveit return pose transformed to wrist
    // used for cartesian as well as checking if interactive marker == moveit
    last_r_arm_moveit_pose_ = wrist_pose.pose;

    updateHandColors();
}

int staticTransform(geometry_msgs::Pose& palm_pose, tf::Transform hand_T_palm)
{
    tf::Transform o_T_hand;    //describes hand in object's frame
    tf::Transform o_T_palm;       //describes palm_from_graspit in object's frame

    o_T_palm.setRotation(tf::Quaternion(palm_pose.orientation.x,palm_pose.orientation.y,palm_pose.orientation.z,palm_pose.orientation.w));
    o_T_palm.setOrigin(tf::Vector3(palm_pose.position.x,palm_pose.position.y,palm_pose.position.z) );

    o_T_hand = o_T_palm * hand_T_palm.inverse();

    //ROS_INFO("hand_T_palm: p=(%f, %f, %f) q=(%f, %f, %f, %f)",
    //         hand_T_palm.getOrigin().getX(),hand_T_palm.getOrigin().getY(),hand_T_palm.getOrigin().getZ(),
    //         hand_T_palm.getRotation().getW(),hand_T_palm.getRotation().getX(),hand_T_palm.getRotation().getY(),hand_T_palm.getRotation().getZ());

    tf::Quaternion hand_quat;
    tf::Vector3    hand_vector;
    hand_quat   = o_T_hand.getRotation();
    hand_vector = o_T_hand.getOrigin();

    palm_pose.position.x = hand_vector.getX();
    palm_pose.position.y = hand_vector.getY();
    palm_pose.position.z = hand_vector.getZ();
    palm_pose.orientation.x = hand_quat.getX();
    palm_pose.orientation.y = hand_quat.getY();
    palm_pose.orientation.z = hand_quat.getZ();
    palm_pose.orientation.w = hand_quat.getW();

    return 0;
}

// publishes the grasp ghost robot hands
void Base3DView::publishHandPose(std::string hand, const geometry_msgs::PoseStamped& end_effector_transform)
{
    geometry_msgs::PoseStamped hand_transform; // the first hand transform is really where I want the palm to be, or identity in this case
    if(hand == "left")
    {
        calcWristTarget(end_effector_transform, l_hand_T_palm_, hand_transform);

        left_hand_virtual_link_joint_states_.position[0] = hand_transform.pose.position.x;
        left_hand_virtual_link_joint_states_.position[1] = hand_transform.pose.position.y;
        left_hand_virtual_link_joint_states_.position[2] = hand_transform.pose.position.z;
        left_hand_virtual_link_joint_states_.position[3] = hand_transform.pose.orientation.x;
        left_hand_virtual_link_joint_states_.position[4] = hand_transform.pose.orientation.y;
        left_hand_virtual_link_joint_states_.position[5] = hand_transform.pose.orientation.z;
        left_hand_virtual_link_joint_states_.position[6] = hand_transform.pose.orientation.w;

        moveit::core::jointStateToRobotState(left_hand_virtual_link_joint_states_, *left_hand_robot_state_);
    }
    else
    {
        calcWristTarget(end_effector_transform, r_hand_T_palm_, hand_transform);

        right_hand_virtual_link_joint_states_.position[0] = hand_transform.pose.position.x;
        right_hand_virtual_link_joint_states_.position[1] = hand_transform.pose.position.y;
        right_hand_virtual_link_joint_states_.position[2] = hand_transform.pose.position.z;
        right_hand_virtual_link_joint_states_.position[3] = hand_transform.pose.orientation.x;
        right_hand_virtual_link_joint_states_.position[4] = hand_transform.pose.orientation.y;
        right_hand_virtual_link_joint_states_.position[5] = hand_transform.pose.orientation.z;
        right_hand_virtual_link_joint_states_.position[6] = hand_transform.pose.orientation.w;

        moveit::core::jointStateToRobotState(right_hand_virtual_link_joint_states_, *right_hand_robot_state_);
    }

    publishHandJointStates(hand);
}

void Base3DView::publishHandJointStates(std::string hand)
{
    std::string hand_type;
    if(hand == "left")
        hand_type = l_hand_type;
    else
        hand_type = r_hand_type;

    sensor_msgs::JointState joint_states;

    joint_states.header.stamp = ros::Time::now();
    joint_states.header.frame_id = std::string("/")+hand+std::string("_hand_model/")+hand+"_palm";

    if(hand_type.find("irobot") != std::string::npos)
    {
        // must match the order used in the .grasp file
        joint_states.name.push_back(hand+"_f0_j1");
        joint_states.name.push_back(hand+"_f1_j1");
        joint_states.name.push_back(hand+"_f2_j1");
        joint_states.name.push_back(hand+"_f0_j0"); // .grasp finger position [4] -> IGNORE [3], use [4] for both
        joint_states.name.push_back(hand+"_f1_j0"); // .grasp finger position [4]
        joint_states.name.push_back(hand+"_f0_j2"); // 0 for now
        joint_states.name.push_back(hand+"_f1_j2"); // 0 for now
        joint_states.name.push_back(hand+"_f2_j2"); // 0 for now

    }
    else if(hand_type.find("robotiq") != std::string::npos)
    {
        // must match the order used in the .grasp file
        joint_states.name.push_back(hand+"_f0_j1");
        joint_states.name.push_back(hand+"_f1_j1");
        joint_states.name.push_back(hand+"_f2_j1");
        joint_states.name.push_back(hand+"_f1_j0"); // .grasp finger position [4] -> IGNORE [3], use [4] for both
        joint_states.name.push_back(hand+"_f2_j0"); // .grasp finger position [4]
        joint_states.name.push_back(hand+"_f0_j2"); // 0 for now
        joint_states.name.push_back(hand+"_f1_j2"); // 0 for now
        joint_states.name.push_back(hand+"_f2_j2"); // 0 for now
        joint_states.name.push_back(hand+"_f0_j3");
        joint_states.name.push_back(hand+"_f1_j3");
        joint_states.name.push_back(hand+"_f2_j3");
    }
    else if(hand_type.find("sandia") != std::string::npos)
    {
        // must match those inside of the /sandia_hands/?_hand/joint_states/[right_/left_]+
        joint_states.name.push_back(hand+"_f0_j0");
        joint_states.name.push_back(hand+"_f0_j1");
        joint_states.name.push_back(hand+"_f0_j2");
        joint_states.name.push_back(hand+"_f1_j0");
        joint_states.name.push_back(hand+"_f1_j1");
        joint_states.name.push_back(hand+"_f1_j2");
        joint_states.name.push_back(hand+"_f2_j0");
        joint_states.name.push_back(hand+"_f2_j1");
        joint_states.name.push_back(hand+"_f2_j2");
        joint_states.name.push_back(hand+"_f3_j0");
        joint_states.name.push_back(hand+"_f3_j1");
        joint_states.name.push_back(hand+"_f3_j2");
    }

    joint_states.position.resize(joint_states.name.size());
    joint_states.effort.resize(joint_states.name.size());
    joint_states.velocity.resize(joint_states.name.size());

    for(unsigned int i = 0; i < joint_states.position.size(); ++i)
    {
        joint_states.effort[i] = 0;
        joint_states.velocity[i] = 0;
        joint_states.position[i] = 0;
    }

    if(hand == "left")
    {
        moveit::core::jointStateToRobotState(joint_states, *left_hand_robot_state_);
        robot_state::robotStateToRobotStateMsg(*left_hand_robot_state_, left_display_state_msg_.state);
        left_hand_robot_state_vis_pub_.publish(left_display_state_msg_);
    }
    else
    {
        moveit::core::jointStateToRobotState(joint_states, *right_hand_robot_state_);
        robot_state::robotStateToRobotStateMsg(*right_hand_robot_state_, right_display_state_msg_.state);
        right_hand_robot_state_vis_pub_.publish(right_display_state_msg_);
    }
}

int Base3DView::calcWristTarget(const geometry_msgs::PoseStamped& end_effector_pose, tf::Transform hand_T_palm, geometry_msgs::PoseStamped& final_pose)
{
    // Transform wrist_pose into the template pose frame
    tf::Transform ef_pose;
    tf::Transform target_pose;

    ef_pose.setRotation(tf::Quaternion(end_effector_pose.pose.orientation.x,end_effector_pose.pose.orientation.y,end_effector_pose.pose.orientation.z,end_effector_pose.pose.orientation.w));
    ef_pose.setOrigin(tf::Vector3(end_effector_pose.pose.position.x,end_effector_pose.pose.position.y,end_effector_pose.pose.position.z) );
    target_pose = ef_pose * hand_T_palm;

//    ROS_INFO("ef_pose: p=(%f, %f, %f) q=(%f, %f, %f, %f)",
//             ef_pose.getOrigin().getX(),ef_pose.getOrigin().getY(),ef_pose.getOrigin().getZ(),
//             ef_pose.getRotation().getW(),ef_pose.getRotation().getX(),ef_pose.getRotation().getY(),ef_pose.getRotation().getZ());

//    ROS_INFO("hand_T_palm: p=(%f, %f, %f) q=(%f, %f, %f, %f)",
//             hand_T_palm.getOrigin().getX(),hand_T_palm.getOrigin().getY(),hand_T_palm.getOrigin().getZ(),
//             hand_T_palm.getRotation().getW(),hand_T_palm.getRotation().getX(),hand_T_palm.getRotation().getY(),hand_T_palm.getRotation().getZ());

//    ROS_INFO("target_pose: p=(%f, %f, %f) q=(%f, %f, %f, %f)",
//             target_pose.getOrigin().getX(),target_pose.getOrigin().getY(),target_pose.getOrigin().getZ(),
//             target_pose.getRotation().getW(),target_pose.getRotation().getX(),target_pose.getRotation().getY(),target_pose.getRotation().getZ());

    tf::Quaternion tg_quat;
    tf::Vector3    tg_vector;
    tg_quat   = target_pose.getRotation();
    tg_vector = target_pose.getOrigin();

    final_pose.pose.orientation.w = tg_quat.getW();
    final_pose.pose.orientation.x = tg_quat.getX();
    final_pose.pose.orientation.y = tg_quat.getY();
    final_pose.pose.orientation.z = tg_quat.getZ();

    final_pose.pose.position.x = tg_vector.getX();
    final_pose.pose.position.y = tg_vector.getY();
    final_pose.pose.position.z = tg_vector.getZ();
    return 0;
}

// callback for the grasp widget left hand pose
void Base3DView::processLeftGhostHandPose(const geometry_msgs::PoseStamped::ConstPtr &pose)
{
    //ROS_INFO("LEFT GHOST HAND POSE:");
    //ROS_INFO("  position: %.2f %.2f %.2f",pose->pose.position.x,pose->pose.position.y,pose->pose.position.z);
    //ROS_INFO("  orientation: %.2f %.2f %.2f %.2f",pose->pose.orientation.w,pose->pose.orientation.x,pose->pose.orientation.y,pose->pose.orientation.z);

    // will only process this if in template lock
    if(!moving_pelvis_ && ghost_world_lock_[0] == 1)
    {
        geometry_msgs::Pose transformed_pose = pose->pose;
        staticTransform(transformed_pose, l_hand_T_palm_);
        end_effector_pose_list_["/l_arm_pose_marker"].pose = transformed_pose;
        publishGhostPoses();
    }
}

// callback for the grasp widget right hand pose
void Base3DView::processRightGhostHandPose(const geometry_msgs::PoseStamped::ConstPtr &pose)
{
    //ROS_INFO("RIGHT GHOST HAND POSE:");
    //ROS_INFO("  position: %.2f %.2f %.2f",pose->pose.position.x,pose->pose.position.y,pose->pose.position.z);
    //ROS_INFO("  orientation: %.2f %.2f %.2f %.2f",pose->pose.orientation.w,pose->pose.orientation.x,pose->pose.orientation.y,pose->pose.orientation.z);

    // will only process this if in template lock
    if(!moving_pelvis_ && ghost_world_lock_[1] == 1)
    {
        geometry_msgs::Pose transformed_pose = pose->pose;
        staticTransform(transformed_pose, r_hand_T_palm_);
        end_effector_pose_list_["/r_arm_pose_marker"].pose = transformed_pose;
        publishGhostPoses();
    }
}

void Base3DView::processGhostPelvisPose(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    flor_ocs_msgs::OCSInteractiveMarkerUpdate cmd;
    cmd.pose = *msg;
    cmd.topic = "/pelvis_pose_marker";
    interactive_marker_update_pub_.publish(cmd);
    onMarkerFeedback(cmd);
}

void Base3DView::updateHandColors()
{
    // need to verify if marker pose == moveit, with 5cm and 2deg tolerance
    if(update_l_arm_color_ && !checkPoseMatch(last_l_arm_moveit_pose_,last_l_arm_marker_pose_,0.005f,2.0f))
    {
        // change color of the ghost template hands
        for (size_t i = 0; i < left_display_state_msg_.highlight_links.size(); ++i)
        {
            left_display_state_msg_.highlight_links[i].color.a = 0.7f;
            left_display_state_msg_.highlight_links[i].color.r = 0.8f;
            left_display_state_msg_.highlight_links[i].color.g = 0.2f;
            left_display_state_msg_.highlight_links[i].color.b = 0.2f;
        }
    }
    // if not, restore green
    else
    {
        // change color of the ghost template hands
        for (size_t i = 0; i < left_display_state_msg_.highlight_links.size(); ++i)
        {
            left_display_state_msg_.highlight_links[i].color.a = 0.5f;
            left_display_state_msg_.highlight_links[i].color.r = 0.2f;
            left_display_state_msg_.highlight_links[i].color.g = 0.8f;
            left_display_state_msg_.highlight_links[i].color.b = 0.2f;
        }
    }

    // need to verify if marker pose == moveit, with 5cm and 2deg tolerance
    if(update_r_arm_color_ && !checkPoseMatch(last_r_arm_moveit_pose_,last_r_arm_marker_pose_,0.005f,2.0f))
    {
        // change color of the ghost template hands
        for (size_t i = 0; i < right_display_state_msg_.highlight_links.size(); ++i)
        {
            right_display_state_msg_.highlight_links[i].color.a = 0.7f;
            right_display_state_msg_.highlight_links[i].color.r = 0.8f;
            right_display_state_msg_.highlight_links[i].color.g = 0.2f;
            right_display_state_msg_.highlight_links[i].color.b = 0.2f;
        }
    }
    // if not, restore green
    else
    {
        // change color of the ghost template hands
        for (size_t i = 0; i < right_display_state_msg_.highlight_links.size(); ++i)
        {
            right_display_state_msg_.highlight_links[i].color.a = 0.5f;
            right_display_state_msg_.highlight_links[i].color.r = 0.2f;
            right_display_state_msg_.highlight_links[i].color.g = 0.8f;
            right_display_state_msg_.highlight_links[i].color.b = 0.2f;
        }
    }
}

// this will return true if position and orientation are within the acceptable thresholds (default values = 0.0f)
bool Base3DView::checkPoseMatch(const geometry_msgs::Pose& p1, const geometry_msgs::Pose& p2, float scalar_error_threshold, float angle_error_threshold)
{
    Ogre::Vector3 p1_p(p1.position.x,p1.position.y,p1.position.z);
    Ogre::Vector3 p2_p(p2.position.x,p2.position.y,p2.position.z);

    // first check distance between two positions
    float diff_position = p1_p.distance(p2_p);
    if(diff_position > scalar_error_threshold)
        return false;

    // then we check the angle difference
    Ogre::Quaternion p1_o(p1.orientation.w,p1.orientation.x,p1.orientation.y,p1.orientation.z);
    Ogre::Quaternion p2_o(p2.orientation.w,p2.orientation.x,p2.orientation.y,p2.orientation.z);

    Ogre::Quaternion diff_orientation = p2_o * p1_o.Inverse();
    Ogre::Degree diff_angle;
    Ogre::Vector3 diff_axis;
    diff_orientation.ToAngleAxis(diff_angle,diff_axis);
    if(diff_angle.valueDegrees() > angle_error_threshold)
        return false;

    // returns true if p1 == p2 considering the thresholds
    return true;
}

void Base3DView::onMarkerFeedback(const flor_ocs_msgs::OCSInteractiveMarkerUpdate& msg)//std::string topic_name, geometry_msgs::PoseStamped pose)
{
    geometry_msgs::PoseStamped joint_pose;
    joint_pose = msg.pose;

    //ROS_ERROR("Marker feedback on topic %s, have markers instantiated",msg.topic.c_str());
    if(msg.topic == "/l_arm_pose_marker")
    {
        ghost_planning_group_[0] = 1;
        ghost_planning_group_[1] = 0;
        moving_pelvis_ = false;
        moving_l_arm_ = true;
        moving_r_arm_ = false;

        //ROS_INFO("LEFT GHOST HAND POSE:");
        //ROS_INFO("  position: %.2f %.2f %.2f",msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z);
        //ROS_INFO("  orientation: %.2f %.2f %.2f %.2f",msg.pose.pose.orientation.w,msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z);

        // this will publish the iron man hand position based on the left arm marker pose
        calcWristTarget(msg.pose,l_hand_T_marker_.inverse(),joint_pose);
        publishHandPose(std::string("left"),joint_pose);

        last_l_arm_marker_pose_ = msg.pose.pose;

        update_l_arm_color_ = true;
        updateHandColors();

        l_arm_marker_pose_pub_.publish(msg.pose);
    }
    else if(msg.topic == "/r_arm_pose_marker")
    {
        ghost_planning_group_[0] = 0;
        ghost_planning_group_[1] = 1;
        moving_pelvis_ = false;
        moving_l_arm_ = false;
        moving_r_arm_ = true;

        //ROS_INFO("RIGHT GHOST HAND POSE:");
        //ROS_INFO("  position: %.2f %.2f %.2f",msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z);
        //ROS_INFO("  orientation: %.2f %.2f %.2f %.2f",msg.pose.pose.orientation.w,msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z);

        // this will publish the iron man hand position based on the right arm marker pose
        calcWristTarget(msg.pose,r_hand_T_marker_.inverse(),joint_pose);
        publishHandPose(std::string("right"),joint_pose);

        last_r_arm_marker_pose_ = msg.pose.pose;

        update_r_arm_color_ = true;
        updateHandColors();

        r_arm_marker_pose_pub_.publish(msg.pose);
    }
    else if(msg.topic == "/pelvis_pose_marker")
    {
        moving_pelvis_ = true;
        moving_l_arm_ = false;
        moving_r_arm_ = false;

        pelvis_marker_pose_pub_.publish(msg.pose);
    }
    else if(msg.topic.find("/cartesian_pose_") != std::string::npos)
    {
        std::string id_str = msg.topic.substr(16);
        try
        {
            int id = boost::lexical_cast<int>(id_str);
            ROS_INFO("cartesian number %d",id);
            if(id < cartesian_waypoint_list_.size())
                cartesian_waypoint_list_[id] = joint_pose.pose;
        }
        catch(...)
        {

        }

        return;
    }
    else if(msg.topic == "/circular_pose")
    {
        circular_center_ = joint_pose.pose;
        return;
    }

    end_effector_pose_list_[msg.topic] = joint_pose; //msg.pose;

    if(marker_published_ < 3)
    {
        publishMarkers();

        if(msg.topic == "/l_arm_pose_marker" && marker_published_ == 0)
            marker_published_++;
        else if(msg.topic == "/r_arm_pose_marker" && marker_published_ == 1)
            marker_published_++;
        else if(msg.topic == "/pelvis_pose_marker" && marker_published_ == 2)
            marker_published_++;
    }

    //else
    //    ROS_ERROR("Marker feedback on topic %s, have no markers instantiated",msg.topic.c_str());
    publishGhostPoses();
}

// sends marker poses and so on to moveit
void Base3DView::publishGhostPoses()
{
    bool left = ghost_planning_group_[0];
    bool right = ghost_planning_group_[1];
    bool torso = ghost_planning_group_[2];

    bool left_lock = ghost_world_lock_[0];
    bool right_lock = ghost_world_lock_[1];
    //ROS_ERROR("Moving marker? %s",moving_pelvis_?"Yes, Pelvis":(moving_l_arm_?"Yes, Left Arm":(moving_r_arm_?"Yes, Right Arm":"No")));
    //ROS_ERROR("Left lock: %s Right lock: %s",left_lock?"yes":"no",right_lock?"yes":"no");

    if(moving_pelvis_ || right_lock || left_lock)
    {
        if(!left_lock && left)
            left = false;
        else if(left_lock)
            left = true;
        if(!right_lock && right)
            right = false;
        else if(right_lock)
            right = true;
        moving_pelvis_ = false;

        if(moving_l_arm_ && right_lock)
        {
            left = true;
            right = false;
            torso = false;
            moving_l_arm_ = false;
        }
        if(moving_r_arm_ && left_lock)
        {
            left = false;
            right = true;
            torso = false;
            moving_l_arm_ = false;
        }
    }

    //ROS_ERROR("Left: %s Right:%s Pelvis: %s",left?"yes":"no",right?"yes":"no",torso?"yes":"no");

    flor_planning_msgs::TargetConfigIkRequest cmd;

    if(left && end_effector_pose_list_.find( "/l_arm_pose_marker") != end_effector_pose_list_.end())
    {
        cmd.target_poses.push_back(end_effector_pose_list_["/l_arm_pose_marker"]);
//        ROS_ERROR("PUBLISHING LEFT ARM POSE:");
//        ROS_ERROR("  position: %.2f %.2f %.2f",end_effector_pose_list_["/l_arm_pose_marker"].pose.position.x,end_effector_pose_list_["/l_arm_pose_marker"].pose.position.y,end_effector_pose_list_["/l_arm_pose_marker"].pose.position.z);
//        ROS_ERROR("  orientation: %.2f %.2f %.2f %.2f",end_effector_pose_list_["/l_arm_pose_marker"].pose.orientation.w,end_effector_pose_list_["/l_arm_pose_marker"].pose.orientation.x,end_effector_pose_list_["/l_arm_pose_marker"].pose.orientation.y,end_effector_pose_list_["/l_arm_pose_marker"].pose.orientation.z);
    }

    if(right && end_effector_pose_list_.find( "/r_arm_pose_marker") != end_effector_pose_list_.end())
    {
        cmd.target_poses.push_back(end_effector_pose_list_["/r_arm_pose_marker"]);
//        ROS_ERROR("PUBLISHING RIGHT ARM POSE:");
//        ROS_ERROR("  position: %.2f %.2f %.2f",end_effector_pose_list_["/r_arm_pose_marker"].pose.position.x,end_effector_pose_list_["/r_arm_pose_marker"].pose.position.y,end_effector_pose_list_["/r_arm_pose_marker"].pose.position.z);
//        ROS_ERROR("  orientation: %.2f %.2f %.2f %.2f",end_effector_pose_list_["/r_arm_pose_marker"].pose.orientation.w,end_effector_pose_list_["/r_arm_pose_marker"].pose.orientation.x,end_effector_pose_list_["/r_arm_pose_marker"].pose.orientation.y,end_effector_pose_list_["/r_arm_pose_marker"].pose.orientation.z);
    }

    cmd.lock_to_world.resize(cmd.target_poses.size());
    for(int i = 0; i < cmd.target_poses.size(); i++)
        cmd.lock_to_world[i].data = 0;//saved_state_world_lock_[i];

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

    // IK not possible for multi appendage (non-chain) groups
    if(position_only_ik_ && !(left && right && torso) && !(left && right && !torso))
        cmd.planning_group.data += "_position_only_ik";

    if(left || right)
        end_effector_pub_.publish(cmd);

    if(ghost_lock_pelvis_)
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

//        for(int i = 0; i < im_ghost_robot_server_.size(); i++)
//        {
//            if(im_ghost_robot_server_[i]->getMarkerName() == "Ghost Pelvis")
//            {
//                im_ghost_robot_server_[i]->setPose(pose);
//                break;
//            }
//        }

        flor_ocs_msgs::OCSInteractiveMarkerUpdate cmd;
        cmd.topic = "/pelvis_pose_marker";
        cmd.pose = pose;
        interactive_marker_update_pub_.publish(cmd);

        pelvis_marker_pose_pub_.publish(pose);
    }
    else
    {
        // how do I set world lock for torso?
        ghost_root_pose_pub_.publish(end_effector_pose_list_["/pelvis_pose_marker"]);

        pelvis_marker_pose_pub_.publish(end_effector_pose_list_["/pelvis_pose_marker"]);
    }
}

void Base3DView::processGhostControlState(const flor_ocs_msgs::OCSGhostControl::ConstPtr &msg)
{
    if(msg->snap)
    {
        snap_ghost_to_robot_ = true;
        return;
    }
    ghost_planning_group_.clear();
    ghost_pose_source_.clear();
    //ghost_world_lock_.clear();

    ghost_planning_group_ = msg->planning_group;
    ghost_pose_source_ = msg->pose_source;
    //ghost_world_lock_ = msg->world_lock;
    moveit_collision_avoidance_ = msg->collision_avoidance;
    ghost_lock_pelvis_ = msg->lock_pelvis;

    snap_ghost_to_robot_ = msg->snap;

    left_marker_moveit_loopback_ = msg->left_moveit_marker_loopback;
    right_marker_moveit_loopback_ = msg->right_moveit_marker_loopback;

    position_only_ik_ = msg->position_only_ik;
}

void Base3DView::processJointStates(const sensor_msgs::JointState::ConstPtr &states)
{
    if(snap_ghost_to_robot_)
    {
        ghost_joint_state_pub_.publish(states);

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

        flor_ocs_msgs::OCSInteractiveMarkerUpdate cmd;
        cmd.topic = "/pelvis_pose_marker";
        cmd.pose = pose;
        interactive_marker_update_pub_.publish(cmd);

        pelvis_marker_pose_pub_.publish(pose);

        snap_ghost_to_robot_ = false;
    }

}

void Base3DView::processPelvisResetRequest( const std_msgs::Bool::ConstPtr &msg )
{
    if(end_effector_pose_list_.find( "/pelvis_pose_marker") != end_effector_pose_list_.end())
    {
        // trasnfrom current marker pose from world to pelvis
        geometry_msgs::PoseStamped marker_pose = end_effector_pose_list_["/pelvis_pose_marker"];
        Ogre::Quaternion original_orientation(marker_pose.pose.orientation.w,marker_pose.pose.orientation.x,marker_pose.pose.orientation.y,marker_pose.pose.orientation.z);
        Ogre::Vector3 position(0,0,0);
        Ogre::Quaternion orientation(1,0,0,0);
        transform(position, orientation, "/pelvis", "/world");

        //ROS_ERROR("ROTATION PELVIS: %f %f %f",orientation.getPitch().valueDegrees(),orientation.getYaw().valueDegrees(),orientation.getRoll().valueDegrees());
        //ROS_ERROR("ROTATION GHOST : %f %f %f",original_orientation.getPitch().valueDegrees(),original_orientation.getYaw().valueDegrees(),original_orientation.getRoll().valueDegrees());

        Ogre::Quaternion final_orientation = Ogre::Quaternion(original_orientation.getRoll(), Ogre::Vector3::UNIT_Z) * Ogre::Quaternion(orientation.getPitch(), Ogre::Vector3::UNIT_X) * Ogre::Quaternion(orientation.getYaw(), Ogre::Vector3::UNIT_Y);

        // create pose based on /pelvis frame
        geometry_msgs::PoseStamped pose;// = end_effector_pose_list_["/pelvis_pose_marker"];
        pose.pose.position.x = marker_pose.pose.position.x;
        pose.pose.position.y = marker_pose.pose.position.y;
        pose.pose.position.z = marker_pose.pose.position.z;
        pose.pose.orientation.x = final_orientation.x;
        pose.pose.orientation.y = final_orientation.y;
        pose.pose.orientation.z = final_orientation.z;
        pose.pose.orientation.w = final_orientation.w;
        pose.header.frame_id = "/world";
        pose.header.stamp = ros::Time::now();

        ghost_root_pose_pub_.publish(pose);

//        for(int i = 0; i < im_ghost_robot_server_.size(); i++)
//        {
//            if(im_ghost_robot_server_[i]->getMarkerName() == "Ghost Pelvis")
//            {
//                im_ghost_robot_server_[i]->setPose(pose);
//                break;
//            }
//        }

        flor_ocs_msgs::OCSInteractiveMarkerUpdate cmd;
        cmd.topic = "/pelvis_pose_marker";
        cmd.pose = pose;
        interactive_marker_update_pub_.publish(cmd);

        pelvis_marker_pose_pub_.publish(pose);
    }
}

void Base3DView::processSendPelvisToFootstepRequest( const std_msgs::Bool::ConstPtr& msg )
{
    if(!msg->data)
        send_footstep_goal_step_pub_.publish(end_effector_pose_list_["/pelvis_pose_marker"]);
    else
        send_footstep_goal_walk_pub_.publish(end_effector_pose_list_["/pelvis_pose_marker"]);
}

void Base3DView::publishMarkers()
{
    geometry_msgs::Point point;

    flor_ocs_msgs::OCSInteractiveMarkerAdd marker_server_left_arm;
    marker_server_left_arm.name = "Ghost Left Arm";
    marker_server_left_arm.topic = "/l_arm_pose_marker";
    marker_server_left_arm.frame = manager_->getFixedFrame().toStdString();
    marker_server_left_arm.scale = 0.2;
    marker_server_left_arm.point = point;
    interactive_marker_add_pub_.publish(marker_server_left_arm);

    flor_ocs_msgs::OCSInteractiveMarkerAdd marker_server_right_arm;
    marker_server_right_arm.name = "Ghost Right Arm";
    marker_server_right_arm.topic = "/r_arm_pose_marker";
    marker_server_right_arm.frame = manager_->getFixedFrame().toStdString();
    marker_server_right_arm.scale = 0.2;
    marker_server_right_arm.point = point;
    interactive_marker_add_pub_.publish(marker_server_right_arm);

    flor_ocs_msgs::OCSInteractiveMarkerAdd marker_server_pelvis;
    marker_server_pelvis.name = "Ghost Pelvis";
    marker_server_pelvis.topic = "/pelvis_pose_marker";
    marker_server_pelvis.frame = manager_->getFixedFrame().toStdString();
    marker_server_pelvis.scale = 0.2;
    marker_server_pelvis.point = point;
    interactive_marker_add_pub_.publish(marker_server_pelvis);
}

void Base3DView::resetView()
{
    //ROS_ERROR("RESET VIEW");
    getCurrentViewController()->reset();
    Ogre::Vector3 position(0,0,0);
    Ogre::Quaternion orientation(1,0,0,0);
    transform(position, orientation, "/pelvis", "/world");
    getCurrentViewController()->lookAt(position);
    //if(dynamic_cast<rviz::OrbitViewController*>(manager_->getViewManager()->getCurrent()) == NULL)
    //    ((rviz::OrbitViewController*)manager_->getViewManager()->getCurrent())->lookAt(position);
    //else if(dynamic_cast<rviz::FixedOrientationOrthoViewController*>(manager_->getViewManager()->getCurrent()) == NULL)
    //    ((rviz::FixedOrientationOrthoViewController*)manager_->getViewManager()->getCurrent())->lookAt(position);
}

void Base3DView::clearPointCloudRaycastRequests()
{
    raycast_point_cloud_viewer_->setEnabled(false);
    raycast_point_cloud_viewer_->setEnabled(true);
}

void Base3DView::clearPointCloudStereoRequests()
{
    stereo_point_cloud_viewer_->setEnabled(false);
    stereo_point_cloud_viewer_->setEnabled(true);
}

void Base3DView::clearPointCloudRegionRequests()
{
    region_point_cloud_viewer_->setEnabled(false);
    region_point_cloud_viewer_->setEnabled(true);
}

void Base3DView::clearMapRequests()
{
    while(ground_map_.size() > 0)
    {
        rviz::Display* tmp = ground_map_[0];
        tmp->setEnabled(false);
        delete tmp;
        manager_->notifyConfigChanged();
        ground_map_.erase(ground_map_.begin());
    }
}

rviz::ViewController* Base3DView::getCurrentViewController()
{
     return manager_->getViewManager()->getCurrent();
}

void Base3DView::processSendCartesian(const std_msgs::Bool::ConstPtr &msg)
{
    std::vector<geometry_msgs::Pose> waypoints;
    // send the marker position
    if(msg->data)
        waypoints.push_back(last_r_arm_moveit_pose_);
    else
        waypoints.push_back(last_l_arm_moveit_pose_);
    bool old_state = cartesian_keep_orientation_->isChecked();
    cartesian_keep_orientation_->setChecked(false);
    sendCartesianTarget(msg->data, waypoints);
    cartesian_keep_orientation_->setChecked(old_state);
}

void Base3DView::sendCartesianTarget(bool right_hand, std::vector<geometry_msgs::Pose> waypoints) // 0 left 1 right
{
    std::string prefix = (right_hand ? "r" : "l");

    flor_planning_msgs::CartesianMotionRequest cmd;

    cmd.header.frame_id = "/world";
    cmd.header.stamp = ros::Time::now();

    cmd.waypoints = waypoints;

    // get position of the wrist in world coordinates
    Ogre::Vector3 wrist_position(0,0,0);
    Ogre::Quaternion wrist_orientation(1,0,0,0);
    transform(wrist_position, wrist_orientation, (std::string("/")+prefix+"_hand").c_str(), "/world");

    // get position of the marker in world coordinates
    geometry_msgs::PoseStamped hand, marker;
    hand.pose.position.x = wrist_position.x;
    hand.pose.position.y = wrist_position.y;
    hand.pose.position.z = wrist_position.z;
    hand.pose.orientation.x = wrist_orientation.x;
    hand.pose.orientation.y = wrist_orientation.y;
    hand.pose.orientation.z = wrist_orientation.z;
    hand.pose.orientation.w = wrist_orientation.w;
    calcWristTarget(hand,(right_hand ? r_hand_T_marker_ : l_hand_T_marker_),marker);

    // calculate the difference between them
    Ogre::Vector3 diff_vector;
    diff_vector.x = wrist_position.x - marker.pose.position.x;
    diff_vector.y = wrist_position.y - marker.pose.position.y;
    diff_vector.z = wrist_position.z - marker.pose.position.z;

    for(int i = 0; i < cmd.waypoints.size(); i++)
    {
        // apply the difference to each one of the waypoints
        if(cartesian_keep_orientation_->isChecked())
        {
            cmd.waypoints[i].position.x = cmd.waypoints[i].position.x + diff_vector.x;
            cmd.waypoints[i].position.y = cmd.waypoints[i].position.y + diff_vector.y;
            cmd.waypoints[i].position.z = cmd.waypoints[i].position.z + diff_vector.z;
            cmd.waypoints[i].orientation.x = wrist_orientation.x;
            cmd.waypoints[i].orientation.y = wrist_orientation.y;
            cmd.waypoints[i].orientation.z = wrist_orientation.z;
            cmd.waypoints[i].orientation.w = wrist_orientation.w;
        }
        else
        {
            geometry_msgs::PoseStamped waypoint, new_waypoint;
            waypoint.pose.position.x = cmd.waypoints[i].position.x;
            waypoint.pose.position.y = cmd.waypoints[i].position.y;
            waypoint.pose.position.z = cmd.waypoints[i].position.z;
            waypoint.pose.orientation.x = cmd.waypoints[i].orientation.x;
            waypoint.pose.orientation.y = cmd.waypoints[i].orientation.y;
            waypoint.pose.orientation.z = cmd.waypoints[i].orientation.z;
            waypoint.pose.orientation.w = cmd.waypoints[i].orientation.w;
            calcWristTarget(waypoint,(right_hand ? r_hand_T_marker_.inverse() : l_hand_T_marker_.inverse()),new_waypoint);

            cmd.waypoints[i] = new_waypoint.pose;
        }

    }

    cmd.use_environment_obstacle_avoidance = cartesian_use_collision_->isChecked();

    if(!ghost_planning_group_[2]) // torso selected in the ghost widget
        cmd.planning_group = prefix+"_arm_group";
    else
        cmd.planning_group = prefix+"_arm_with_torso_group";

    if(position_only_ik_)
        cmd.planning_group += prefix+"_position_only_ik";

    cartesian_plan_request_pub_.publish(cmd);
}

void Base3DView::sendCartesianLeft()
{
    sendCartesianTarget(0,cartesian_waypoint_list_);
}

void Base3DView::sendCartesianRight()
{
    sendCartesianTarget(1,cartesian_waypoint_list_);
}

void Base3DView::sendCircularTarget(bool right_hand)
{
    std::string prefix = (right_hand ? "r" : "l");
    flor_planning_msgs::CircularMotionRequest cmd;

    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "/world";
    pose.header.stamp = ros::Time::now();
    pose.pose = circular_center_;

    // calculating the rotation based on position of the markers
    if(circular_keep_orientation_->isChecked())
    {
        // get position of the wrist in world coordinates
        Ogre::Vector3 wrist_position(0,0,0);
        Ogre::Quaternion wrist_orientation(1,0,0,0);
        transform(wrist_position, wrist_orientation, (std::string("/")+prefix+"_hand").c_str(), "/world");

        // get position of the marker in world coordinates
        geometry_msgs::PoseStamped hand, marker;
        hand.pose.position.x = wrist_position.x;
        hand.pose.position.y = wrist_position.y;
        hand.pose.position.z = wrist_position.z;
        hand.pose.orientation.x = wrist_orientation.x;
        hand.pose.orientation.y = wrist_orientation.y;
        hand.pose.orientation.z = wrist_orientation.z;
        hand.pose.orientation.w = wrist_orientation.w;
        calcWristTarget(hand,(right_hand ? r_hand_T_marker_ : l_hand_T_marker_),marker);

        // calculate the difference between them
        Ogre::Vector3 diff_vector;
        diff_vector.x = wrist_position.x - marker.pose.position.x;
        diff_vector.y = wrist_position.y - marker.pose.position.y;
        diff_vector.z = wrist_position.z - marker.pose.position.z;

        // apply the difference to the circular center
        pose.pose.position.x = circular_center_.position.x + diff_vector.x;
        pose.pose.position.y = circular_center_.position.y + diff_vector.y;
        pose.pose.position.z = circular_center_.position.z + diff_vector.z;
    }

    cmd.rotation_center_pose = pose;

    cmd.rotation_angle = circular_angle_->value()*0.0174532925; // UI in deg, msg in rad

    cmd.use_environment_obstacle_avoidance = circular_use_collision_->isChecked();

    cmd.keep_endeffector_orientation = circular_keep_orientation_->isChecked();

    if(!ghost_planning_group_[2]) // torso selected in the ghost widget
        cmd.planning_group = prefix+"_arm_group";
    else
        cmd.planning_group = prefix+"_arm_with_torso_group";

    if(position_only_ik_)
        cmd.planning_group += "_position_only_ik";

    circular_plan_request_pub_.publish(cmd);
}

void Base3DView::sendCircularLeft()
{
    sendCircularTarget(false);
}

void Base3DView::sendCircularRight()
{
    sendCircularTarget(true);
}

bool Base3DView::eventFilter( QObject * o, QEvent * e )
{
    if ( e->type() == QEvent::Enter )
    {
        Q_EMIT setRenderPanel(this->render_panel_);
    }
    else if ( e->type() == QEvent::MouseMove )
    {
        Q_EMIT setRenderPanel(this->render_panel_);
    }
    return QWidget::eventFilter( o, e );
}

void Base3DView::processNewKeyEvent(const flor_ocs_msgs::OCSKeyEvent::ConstPtr &key_event)
{
    // store key state
    if(key_event->state)
        keys_pressed_list_.push_back(key_event->key);
    else
        keys_pressed_list_.erase(std::remove(keys_pressed_list_.begin(), keys_pressed_list_.end(), key_event->key), keys_pressed_list_.end());

    // process hotkeys
    bool ctrl_is_pressed = (std::find(keys_pressed_list_.begin(), keys_pressed_list_.end(), 37) != keys_pressed_list_.end());
    bool shift_is_pressed = (std::find(keys_pressed_list_.begin(), keys_pressed_list_.end(), 50) != keys_pressed_list_.end());
    bool alt_is_pressed = (std::find(keys_pressed_list_.begin(), keys_pressed_list_.end(), 64) != keys_pressed_list_.end());

    if(key_event->key == 24 && key_event->state && ctrl_is_pressed) // 'q'
        robotModelToggled(!robot_model_->isEnabled());
    else if(key_event->key == 25 && key_event->state && ctrl_is_pressed) // 'w'
        simulationRobotToggled(!ghost_robot_model_->isEnabled());
    else if(key_event->key == 10 && key_event->state && ctrl_is_pressed) // ctrl+1
    {
        clearPointCloudRaycastRequests();
        clearPointCloudRegionRequests();
        clearPointCloudStereoRequests();
    }
    else if(key_event->key == 18 && key_event->state && ctrl_is_pressed) // ctrl+9
    {
        // rainbow color
        region_point_cloud_viewer_->subProp( "Color Transformer" )->setValue( "AxisColor" );
    }
    else if(key_event->key == 19 && key_event->state && ctrl_is_pressed) // ctrl+9
    {
        // intensity
        region_point_cloud_viewer_->subProp( "Color Transformer" )->setValue( "Intensity" );
    }
    else if(ctrl_is_pressed && alt_is_pressed) //emergency stop
    {
        stop_button_->setVisible(true);
        stop_button_->setGeometry(this->geometry().bottomRight().x()/2 - 200,this->geometry().bottomRight().y()/2 - 150,400,300);
    }
    else
    {
        stop_button_->setVisible(false);
    }

}

void Base3DView::processHotkeyRelayMessage(const flor_ocs_msgs::OCSHotkeyRelay::ConstPtr &msg)
{
    if(msg->relay_code == flor_ocs_msgs::OCSHotkeyRelay::CLEAR_CLOUD_DATA)
    {
        clearPointCloudRaycastRequests();
        clearPointCloudRegionRequests();
        clearPointCloudStereoRequests();
    }
    if(msg->relay_code == flor_ocs_msgs::OCSHotkeyRelay::SET_LIDAR_RAINBOW)
    {
        region_point_cloud_viewer_->subProp( "Color Transformer" )->setValue( "AxisColor" );
    }
    if(msg->relay_code == flor_ocs_msgs::OCSHotkeyRelay::SET_LIDAR_INTENSITY)
    {
        region_point_cloud_viewer_->subProp( "Color Transformer" )->setValue( "Intensity" );
    }
}

}
