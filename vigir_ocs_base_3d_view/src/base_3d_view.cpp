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

#include <render_panel_custom.h>
#include "rviz/display.h"
#include "rviz/frame_manager.h"
#include "rviz/tool_manager.h"
#include "rviz/view_manager.h"
#include "rviz/default_plugin/view_controllers/fixed_orientation_ortho_view_controller.h"
#include "rviz/default_plugin/view_controllers/orbit_view_controller.h"
#include "rviz/default_plugin/view_controllers/fps_view_controller.h"
#include "rviz/properties/parse_color.h"
#include "rviz/properties/vector_property.h"
#include "rviz/properties/quaternion_property.h"
#include "rviz/display_group.h"
#include <template_display_custom.h>
#include "robot_display_custom.h"
#include "selection_3d_display_custom.h"
#include "map_display_custom.h"
#include "joint_marker_display_custom.h"

#include "flor_ocs_msgs/OCSTemplateAdd.h"
#include "flor_ocs_msgs/OCSTemplateRemove.h"
#include "flor_ocs_msgs/OCSWaypointAdd.h"
#include "flor_ocs_msgs/OCSControlMode.h"
#include "flor_perception_msgs/EnvironmentRegionRequest.h"
#include "flor_planning_msgs/TargetConfigIkRequest.h"
#include "flor_planning_msgs/CartesianMotionRequest.h"
#include "flor_planning_msgs/CircularMotionRequest.h"

#include <OgreHardwarePixelBuffer.h>
#include <OgreMaterialManager.h>

// local includes
#include "base_3d_view.h"
#include "base_context_menu.h"

// define used to determine interactive marker mode
#define IM_MODE_OFFSET 3

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
    , circular_marker_(0)
    , visualize_grid_map_(true)
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
        overlay_display_ = copy_from->getOverlayDisplay();
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

        // First remove all existing tools
        manager_->getToolManager()->removeAll();
        // Add support for interactive markers
        interactive_markers_tool_ = manager_->getToolManager()->addTool( "rviz/InteractionToolCustom" );
        // Add support for selection
        //selection_tool_ = manager_->getToolManager()->addTool( "rviz/Select" );
        // Add support for camera movement
        move_camera_tool_ = manager_->getToolManager()->addTool( "rviz/MoveCamera" );
        // Add support for goal specification/vector navigation
        set_goal_tool_ = manager_->getToolManager()->addTool( "rviz/SetGoal" );
        set_goal_tool_->getPropertyContainer()->subProp( "Topic" )->setValue( "/flor/ocs/footstep/goal_pose" );

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

        // footstep visualization manager initialization
        footstep_vis_manager_ = new FootstepVisManager(manager_);

        // F/T sensor displays
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

            if(left_hand_robot_model_->hasJointModelGroup("left_hand"))
            {
                left_hand_joint_names_.clear();
                left_hand_joint_names_ = left_hand_robot_model_->getJointModelGroup("left_hand")->getActiveJointModelNames();
            }else{
                ROS_INFO("NO JOINTS FOUND FOR LEFT HAND");
            }
            for(int i = 0; i < left_hand_joint_names_.size(); i++)
                ROS_INFO("Base 3d widget loading joint %d: %s",i,left_hand_joint_names_[i].c_str());
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

            if(right_hand_robot_model_->hasJointModelGroup("right_hand"))
            {
                right_hand_joint_names_.clear();
                right_hand_joint_names_ = right_hand_robot_model_->getJointModelGroup("right_hand")->getActiveJointModelNames();
            }else{
                ROS_INFO("NO JOINTS FOUND FOR RIGHT HAND");
            }
            for(int i = 0; i < right_hand_joint_names_.size(); i++)
                ROS_INFO("Base 3d widget loading joint %d: %s",i,right_hand_joint_names_[i].c_str());
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

        interactive_marker_add_pub_ = nh_.advertise<flor_ocs_msgs::OCSInteractiveMarkerAdd>( "/flor/ocs/interactive_marker_server/add", 5, false );
        interactive_marker_update_pub_ = nh_.advertise<flor_ocs_msgs::OCSInteractiveMarkerUpdate>( "/flor/ocs/interactive_marker_server/update", 1, false );
        interactive_marker_feedback_sub_ = nh_.subscribe( "/flor/ocs/interactive_marker_server/feedback", 5, &Base3DView::onMarkerFeedback, this );
        interactive_marker_remove_pub_ = nh_.advertise<std_msgs::String>( "/flor/ocs/interactive_marker_server/remove", 5, false );

        //Publisher/Subscriber to the IM mode
        interactive_marker_server_mode_pub_ = nh_.advertise<flor_ocs_msgs::OCSControlMode>("/flor/ocs/control_modes",1,false);
        interactive_marker_server_mode_sub_ = nh_.subscribe<flor_ocs_msgs::OCSControlMode>("/flor/ocs/control_modes",1, &Base3DView::processInteractiveMarkerMode, this);

        // subscribe to the moveit pose topics
        end_effector_sub_.push_back(nh_.subscribe<geometry_msgs::PoseStamped>( "/flor/ghost/pose/left_hand", 5, &Base3DView::processLeftArmEndEffector, this ));
        end_effector_sub_.push_back(nh_.subscribe<geometry_msgs::PoseStamped>( "/flor/ghost/pose/right_hand", 5, &Base3DView::processRightArmEndEffector, this ));
        end_effector_sub_.push_back(nh_.subscribe<geometry_msgs::PoseStamped>( "/flor/ghost/pose/robot", 5, &Base3DView::processPelvisEndEffector, this ));

        end_effector_pub_ = nh_.advertise<flor_planning_msgs::TargetConfigIkRequest>( "/flor/ghost/set_appendage_poses", 5, false );

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
        send_footstep_goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>( "/flor/ocs/footstep/goal_pose", 1, false );

        // subscribe to goal pose
        set_goal_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>( "/flor/ocs/footstep/goal_pose", 5, &Base3DView::processGoalPose, this );

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

        //sub to ghost joint states
        ghost_joint_state_sub_ = nh_.subscribe<sensor_msgs::JointState>( "/flor/ghost/get_joint_states", 5, &Base3DView::processGhostJointStates, this );

        //synchronize 3d views
        ocs_sync_sub_ = nh_.subscribe<flor_ocs_msgs::OCSSynchronize>( "/flor/ocs/synchronize", 5, &Base3DView::synchronizeViews, this );
        ocs_sync_pub_ = nh_.advertise<flor_ocs_msgs::OCSSynchronize>( "/flor/ocs/synchronize", 5, false);

        //create joint position error displays
        joint_arrows_ = manager_->createDisplay( "rviz/JointMarkerDisplayCustom", "Joint Position Markers", true );
        joint_arrows_->subProp("Topic")->setValue("/atlas/joint_states");
        joint_arrows_->subProp("Width")->setValue("0.015");
        joint_arrows_->subProp("Scale")->setValue("1.2");        

        ghost_joint_arrows_ = manager_->createDisplay( "rviz/JointMarkerDisplayCustom", "Ghost Joint Position Markers", false );
        ghost_joint_arrows_->subProp("Topic")->setValue("/flor/ghost/get_joint_states");
        ghost_joint_arrows_->subProp("Width")->setValue("0.015");
        ghost_joint_arrows_->subProp("Scale")->setValue("1.2");
        ghost_joint_arrows_->subProp("isGhost")->setValue(true);

        disable_joint_markers_ = false;
        occluded_robot_visible_ = false;        

        //setRobotOccludedRender();

        // update render order whenever objects are added/ display changed
        //connect(manager_,SIGNAL(statusUpdate(QString)),this,SLOT(setRenderOrder(QString)));
        // initialize Render Order correctly
        //setRenderOrder();

        // initialize notification system
        overlay_display_ = manager_->createDisplay( "jsk_rviz_plugin/OverlayTextDisplay", "Notification System", true );
        overlay_display_->subProp("Topic")->setValue("flor/ocs/overlay_text");      

        //create visualizations for camera frustum
        //initializeFrustums("/flor/ocs/camera/atlas");
       // initializeFrustums("/flor/ocs/camera/left_hand");
        //initializeFrustums("/flor/ocs/camera/right_hand");

        //left eye test, need to load topics on launch
        frustum_display_ = manager_->createDisplay("rviz/CameraFrustumDisplayCustom","Frustum Display", true);
        frustum_display_->subProp("Topic")->setValue("/multisense_sl/left/camera_info");
        frustum_display_->subProp("alpha")->setValue("0.05");
        frustum_display_->setEnabled(false);
    }

    //initialize overall context menu
    context_menu_manager_ = new ContextMenuManager(this);

    // Connect the 3D selection tool to
    QObject::connect(this, SIGNAL(queryContext(int,int)), selection_3d_display_, SLOT(queryContext(int,int)));
    QObject::connect(selection_3d_display_, SIGNAL(setContext(int,std::string)), this, SLOT(setContext(int,std::string)));

    // connect the 3d selection tool to its display
    QObject::connect(this, SIGNAL(setRenderPanel(rviz::RenderPanel*)), selection_3d_display_, SLOT(setRenderPanel(rviz::RenderPanel*)));
    Q_EMIT setRenderPanel(this->render_panel_);
    QObject::connect(selection_3d_display_, SIGNAL(newSelection(Ogre::Vector3)), this, SLOT(newSelection(Ogre::Vector3)));
    QObject::connect(selection_3d_display_, SIGNAL(setSelectionRay(Ogre::Ray)), this, SLOT(setSelectionRay(Ogre::Ray)));
    QObject::connect(this, SIGNAL(resetSelection()), selection_3d_display_, SLOT(resetSelection()));
    QObject::connect(this, SIGNAL(setMarkerPosition(float,float,float)), selection_3d_display_, SLOT(setMarkerPosition(float,float,float)));

    // handles mouse events without rviz::tool
    mouse_event_handler_ = new vigir_ocs::MouseEventHandler();
    QObject::connect(render_panel_, SIGNAL(signalMousePressEvent(QMouseEvent*)), mouse_event_handler_, SLOT(mousePressEvent(QMouseEvent*)));
    QObject::connect(render_panel_, SIGNAL(signalMouseReleaseEvent(QMouseEvent*)), mouse_event_handler_, SLOT(mouseReleaseEvent(QMouseEvent*)));
    QObject::connect(render_panel_, SIGNAL(signalMouseDoubleClickEvent(QMouseEvent*)), mouse_event_handler_, SLOT(mouseDoubleClick(QMouseEvent*)));
    QObject::connect(mouse_event_handler_, SIGNAL(mouseLeftButtonCtrl(bool,int,int)), selection_3d_display_, SLOT(raycastRequest(bool,int,int)));
    //QObject::connect(mouse_event_handler_, SIGNAL(mouseLeftButtonShift(bool,int,int)), selection_3d_display_, SLOT(raycastRequestROI(bool,int,int)));
    //initializes overall context menu
    QObject::connect(mouse_event_handler_, SIGNAL(mouseRightButton(bool,int,int)),context_menu_manager_, SLOT(createContextMenu(bool,int,int)));
    QObject::connect(mouse_event_handler_, SIGNAL(signalMouseLeftDoubleClick(int,int)), this, SLOT(selectOnDoubleClick(int,int)));

    Q_FOREACH( QWidget* sp, findChildren<QWidget*>() ) {
        sp->installEventFilter( this );
        sp->setMouseTracking( true );
    }

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
    camera_transform_pub_ = nh_.advertise<flor_ocs_msgs::OCSCameraTransform>( "/flor/ocs/camera_transform", 5, false );

    //initialize  base-specific context menu
    base_context_menu_ = new BaseContextMenu(this);

    //Initialize shift_pressed_ to false and set interactive_marker_mode_ to default
    shift_pressed_ = false;
    interactive_marker_mode_ = 0;

    // this is only used to make sure we close window if ros::shutdown has already been called
    timer.start(33, this);
}

// Destructor.
Base3DView::~Base3DView()
{
    delete manager_;
}

void Base3DView::initializeFrustums(std::string prefix)
{
    // get cameras parameters and create rviz::display
    XmlRpc::XmlRpcValue camera_topic_prefix;

    nh_.getParam(prefix+"/topic_prefix", camera_topic_prefix);
    for(int i = 0; i < camera_topic_prefix.size(); i++)
    {
        std::string topic = static_cast<std::string>(camera_topic_prefix[i]);
        //ROS_ERROR("topic %s",topic.c_str());
        rviz::Display* frustum_display = manager_->createDisplay("rviz/CameraFrustumDisplayCustom","Frustum Display", true);
        frustum_display->subProp("Topic")->setValue(topic.c_str());
        frustum_display->subProp("alpha")->setValue("0.05");
        frustum_display->setEnabled(false);
        frustum_map_[topic] = frustum_display;
    }
}
//NOTE: DOES NOT CURRENTLY WORK, TODO: figure a way to pass a rendered texture from shader program to shader program to accumulate effects
void Base3DView::blurRender()
{
    if(renderTexture1 == NULL)
    {
        Ogre::TexturePtr rtt_texture = Ogre::TextureManager::getSingleton().createManual("RttTex1",
                                       Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, Ogre::TEX_TYPE_2D, render_panel_->getRenderWindow()->getWidth(),
                                       render_panel_->getRenderWindow()->getHeight(), 0, Ogre::PF_R8G8B8, Ogre::TU_RENDERTARGET);

        renderTexture1 = rtt_texture->getBuffer()->getRenderTarget();

        Ogre::ColourValue transparent(1,0,1,0);

        renderTexture1->addViewport(render_panel_->getCamera());
        renderTexture1->getViewport(0)->setClearEveryFrame(true);
        renderTexture1->getViewport(0)->setBackgroundColour(transparent);
        renderTexture1->getViewport(0)->setOverlaysEnabled(false);

        const char *remove_magenta =
                   "uniform sampler2D mTexture1;\n"
                   "void main()\n"
                   "{\n"
                        " vec4 color = texture2D(mTexture1, gl_TexCoord[0].st);\n"
                        "if(color.x == 1.0f && color.y == 0 && color.z == 1.0f)\n"
                        " {  gl_FragColor = vec4(0,0,0,0);}\n"
                    "else\n"
                        "{color.w = 0.5; gl_FragColor = color;}\n"
                   "}\n";

        const char *horizontal_blur =
                "uniform sampler2D mTexture1; // the texture with the scene you want to blur\n"

                "const float blurSize = 1.0/800; \n"

                "void main(void)\n"
                "{\n"
                    "vec2 vTexCoord = gl_TexCoord[0].st;\n"
                   "vec4 sum = vec4(0.0);\n"

                   // blur in y (vertical)
                   // take nine samples, with the distance blurSize between them
                   "sum += texture2D(mTexture1, vec2(vTexCoord.x - 4.0*blurSize, vTexCoord.y)) * 0.05;\n"
                   "sum += texture2D(mTexture1, vec2(vTexCoord.x - 3.0*blurSize, vTexCoord.y)) * 0.09;\n"
                   "sum += texture2D(mTexture1, vec2(vTexCoord.x - 2.0*blurSize, vTexCoord.y)) * 0.12;\n"
                   "sum += texture2D(mTexture1, vec2(vTexCoord.x - blurSize, vTexCoord.y)) * 0.15;\n"
                   "sum += texture2D(mTexture1, vec2(vTexCoord.x, vTexCoord.y)) * 0.16;\n"
                   "sum += texture2D(mTexture1, vec2(vTexCoord.x + blurSize, vTexCoord.y)) * 0.15;\n"
                   "sum += texture2D(mTexture1, vec2(vTexCoord.x + 2.0*blurSize, vTexCoord.y)) * 0.12;\n"
                   "sum += texture2D(mTexture1, vec2(vTexCoord.x + 3.0*blurSize, vTexCoord.y)) * 0.09;\n"
                   "sum += texture2D(mTexture1, vec2(vTexCoord.x + 4.0*blurSize, vTexCoord.y)) * 0.05;\n"
                    "gl_FragColor = sum;\n"
                "}\n";

        const char *vertical_blur =
                "uniform sampler2D mTexture1; // the texture with the scene you want to blur\n"

                "const float blurSize = 1.0/400; \n"

                "void main(void)\n"
                "{\n"
                    "vec2 vTexCoord = gl_TexCoord[0].st;\n"
                   "vec4 sum = vec4(0.0);\n"

                "sum += texture2D(mTexture1, vec2(vTexCoord.x, vTexCoord.y - 4.0*blurSize)) * 0.05;\n"
                  "sum += texture2D(mTexture1, vec2(vTexCoord.x, vTexCoord.y - 3.0*blurSize)) * 0.09;\n"
                  "sum += texture2D(mTexture1, vec2(vTexCoord.x, vTexCoord.y - 2.0*blurSize)) * 0.12;\n"
                  "sum += texture2D(mTexture1, vec2(vTexCoord.x, vTexCoord.y - blurSize)) * 0.15;\n"
                  "sum += texture2D(mTexture1, vec2(vTexCoord.x, vTexCoord.y)) * 0.16;\n"
                  "sum += texture2D(mTexture1, vec2(vTexCoord.x, vTexCoord.y + blurSize)) * 0.15;\n"
                  "sum += texture2D(mTexture1, vec2(vTexCoord.x, vTexCoord.y + 2.0*blurSize)) * 0.12;\n"
                  "sum += texture2D(mTexture1, vec2(vTexCoord.x, vTexCoord.y + 3.0*blurSize)) * 0.09;\n"
                  "sum += texture2D(mTexture1, vec2(vTexCoord.x, vTexCoord.y + 4.0*blurSize)) * 0.05;\n"
                  "gl_FragColor = sum;\n"

                "}\n";






        Ogre::HighLevelGpuProgramPtr removeMagentaFP = Ogre::HighLevelGpuProgramManager::getSingleton()
                    .createProgram("removeMagenta", "General", "glsl", Ogre::GPT_FRAGMENT_PROGRAM);
        removeMagentaFP->setSource(remove_magenta);
        removeMagentaFP->load();
        Ogre::HighLevelGpuProgramPtr horizontalBlurFP = Ogre::HighLevelGpuProgramManager::getSingleton()
                    .createProgram("horizontal", "General", "glsl", Ogre::GPT_FRAGMENT_PROGRAM);
        horizontalBlurFP->setSource(horizontal_blur);
        horizontalBlurFP->load();
        Ogre::HighLevelGpuProgramPtr verticalBlurFP = Ogre::HighLevelGpuProgramManager::getSingleton()
                    .createProgram("vertical", "General", "glsl", Ogre::GPT_FRAGMENT_PROGRAM);
        verticalBlurFP->setSource(vertical_blur);
        verticalBlurFP->load();

        //render entire window
        Ogre::Rectangle2D* mMiniScreen = new Ogre::Rectangle2D(true);
        mMiniScreen->setCorners(-1.0f, 1.0f, 1.0f, -1.0f);
        mMiniScreen->setBoundingBox(Ogre::AxisAlignedBox::BOX_INFINITE);

        Ogre::SceneNode* miniScreenNode = manager_->getSceneManager()->getRootSceneNode()->createChildSceneNode("MiniScreenNode");
        miniScreenNode->attachObject(mMiniScreen);

        Ogre::MaterialPtr renderMaterial = Ogre::MaterialManager::getSingleton().create("RttMat", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
        renderMaterial->getTechnique(0)->getPass(0)->setLightingEnabled(false);
        renderMaterial->getTechnique(0)->getPass(0)->createTextureUnitState("RttTex1");
        renderMaterial->getTechnique(0)->getPass(0)->setFragmentProgram("removeMagenta");

        Ogre::GpuProgramParametersSharedPtr pParams = renderMaterial->getTechnique(0)->getPass(0)->getFragmentProgramParameters();
        //set texture for shader
        pParams->setNamedConstant("mTexture1", 0);        

    //    renderMaterial->getTechnique(0)->getPass(0)->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);

        Ogre::Pass* horizontalPass = renderMaterial->getTechnique(0)->createPass();
        horizontalPass->createTextureUnitState("RttTex1",0);
        horizontalPass->setFragmentProgram("horizontal");

        pParams = horizontalPass->getFragmentProgramParameters();
        pParams->setNamedConstant("mTexture1", 0);

  //      horizontalPass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);

        Ogre::Pass* verticalPass = renderMaterial->getTechnique(0)->createPass();
        verticalPass->createTextureUnitState("RttTex1",0);
        verticalPass->setFragmentProgram("vertical");

        pParams = verticalPass->getFragmentProgramParameters();
        pParams->setNamedConstant("mTexture1", 0);

//        verticalPass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);

        renderMaterial->getTechnique(0)->movePass(horizontalPass->getIndex(),0);
        renderMaterial->getTechnique(0)->movePass(verticalPass->getIndex(),1);
        renderMaterial->getTechnique(0)->movePass(renderMaterial->getTechnique(0)->getPass(0)->getIndex(),2);

        mMiniScreen->setMaterial("RttMat");
    }

    //set visibility of false for everything but robot
    std::vector<bool> visibilities;
    int num_displays = render_panel_->getManager()->getRootDisplayGroup()->numDisplays();
    for(int i = 0; i < num_displays; i++)
    {
        rviz::Display* display = render_panel_->getManager()->getRootDisplayGroup()->getDisplayAt(i);
        std::string display_name = display->getNameStd();
        //camera should be unaffected by render order
        if(display_name.find("Robot model") == std::string::npos)
            setChildrenVisibility(display->getSceneNode(),visibilities, false);
    }

    renderTexture1->update();
    //renderTexture2->update();
    //renderTexture3->update();

    //reset visibility back to normal
    for(int i = 0; i < num_displays; i++)
    {
        rviz::Display* display = render_panel_->getManager()->getRootDisplayGroup()->getDisplayAt(i);
        std::string display_name = display->getNameStd();
        //camera should be unaffected by render order
        if(display_name.find("Robot model") == std::string::npos)
            restoreChildrenVisibility(display->getSceneNode(),visibilities);
    }



}

void Base3DView::setChildrenVisibility(Ogre::SceneNode* node, std::vector<bool>& last_visibility, bool visibility)
{
    // traverse objects attached to this scene node
    Ogre::SceneNode::ObjectIterator it_object = node->getAttachedObjectIterator();
    while (it_object.hasMoreElements())
    {
        Ogre::MovableObject * obj = it_object.getNext();
        last_visibility.push_back(obj->getVisible());
        obj->setVisible(visibility);
    }

    // traverse all other child scene nodes
    Ogre::SceneNode::ChildNodeIterator it_children =  node->getChildIterator();
    while (it_children.hasMoreElements())
    {
        Ogre::SceneNode * child = (Ogre::SceneNode*)it_children.getNext();
        setChildrenVisibility(child, last_visibility, visibility);
    }

}

void Base3DView::restoreChildrenVisibility(Ogre::SceneNode* node, std::vector<bool>& last_visibility)
{
    Ogre::SceneNode::ObjectIterator it_object = node->getAttachedObjectIterator();
    while (it_object.hasMoreElements())
    {
        Ogre::MovableObject * obj = it_object.getNext();
        obj->setVisible(*last_visibility.begin());
        last_visibility.erase(last_visibility.begin());
    }
    Ogre::SceneNode::ChildNodeIterator it_children =  node->getChildIterator();
    while (it_children.hasMoreElements())
    {
        Ogre::SceneNode * child = (Ogre::SceneNode*)it_children.getNext();
        restoreChildrenVisibility(child, last_visibility);
    }
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
    Ogre::RenderTarget::FrameStats stats = render_panel_->getRenderWindow()->getStatistics();
    //std::cout << "View (" << view_id_ << "): " << stats.lastFPS << ", " << stats.avgFPS << ", " << stats.bestFrameTime << ", " << stats.worstFrameTime << ", " << stats.triangleCount << std::endl;

    int fps = int(stats.lastFPS + 0.5); //rounded
    // fps can be 0 for hidden views or invalid decimal values
    if(fps > 1)
        Q_EMIT sendFPS(fps);
    //ROS_ERROR("View: %d LastFps: %f  avgFps: %f ",view_id_,stats.lastFPS,stats.avgFPS);

    // no need to spin as rviz is already doing that for us.
    //ros::spinOnce();

    //Means that currently doing

    //if(is_primary_view_ && occluded_robot_visible_)
    //    setRenderOrder();
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
    cmd.vfov = camera->getFOVy().valueDegrees();
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
    flor_ocs_msgs::OCSSynchronize msg;
    msg.properties.push_back("Raycast Point Cloud");
    msg.reset.push_back(false);
    msg.visible.push_back(selected);
    ocs_sync_pub_.publish(msg);
}

void Base3DView::lidarPointCloudToggled( bool selected )
{
    flor_ocs_msgs::OCSSynchronize msg;
    msg.properties.push_back("LIDAR Point Cloud");
    msg.reset.push_back(false);
    msg.visible.push_back(selected);
    ocs_sync_pub_.publish(msg);
}

void Base3DView::stereoPointCloudToggled( bool selected )
{
    flor_ocs_msgs::OCSSynchronize msg;
    msg.properties.push_back("Stereo Point Cloud");
    msg.reset.push_back(false);
    msg.visible.push_back(selected);
    ocs_sync_pub_.publish(msg);
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
    flor_ocs_msgs::OCSSynchronize msg;
    msg.properties.push_back("Octomap");
    msg.reset.push_back(false);
    msg.visible.push_back(selected);
    ocs_sync_pub_.publish(msg);
}

void Base3DView::gridMapToggled( bool selected )
{
    flor_ocs_msgs::OCSSynchronize msg;
    msg.properties.push_back("Ground map");
    msg.reset.push_back(false);
    msg.visible.push_back(selected);
    ocs_sync_pub_.publish(msg);
}

void Base3DView::notificationSystemToggled(bool selected)
{
    flor_ocs_msgs::OCSSynchronize msg;
    msg.properties.push_back("Notification System");
    msg.reset.push_back(false);
    msg.visible.push_back(selected);
    ocs_sync_pub_.publish(msg);
}

void Base3DView::cameraFrustumToggled(bool selected)
{
    flor_ocs_msgs::OCSSynchronize msg;
    msg.properties.push_back("Frustum Display");
    msg.reset.push_back(false);
    msg.visible.push_back(selected);
    ocs_sync_pub_.publish(msg);
}

void Base3DView::footstepPlanningToggled( bool selected )
{
    footstep_vis_manager_->setEnabled(selected);
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

    //hide/show ghost joint Warning icons on ghost robot toggle
    for (std::map<std::string,rviz::Display*>::iterator it=jointDisplayMap.begin(); it!=jointDisplayMap.end(); ++it)
    {        
        if(it->second->subProp("Name")->getValue().toString().contains("ghost"))
           it->second->setEnabled(selected);
    }
}

void Base3DView::robotOcclusionToggled(bool selected)
{
    if (!selected && is_primary_view_)
    {
        occluded_robot_visible_ = false;
        //disableRobotOccludedRender();
    }
    else if (is_primary_view_)
    {
        occluded_robot_visible_ = true;
        //setRobotOccludedRender();
    }
}

void Base3DView::robotJointMarkerToggled(bool selected)
{
    if(!selected && is_primary_view_)
    {
        disable_joint_markers_ = true;
        //ghost state is only checked when ghost is manipulated, needs an additional call to refresh joint states
        if(ghost_robot_model_->isEnabled())
            processGhostJointStates(latest_ghost_joint_state_);

    }
    else if (is_primary_view_)
    {
        disable_joint_markers_ = false;
        if(ghost_robot_model_->isEnabled())
            processGhostJointStates(latest_ghost_joint_state_);

    }
}

void Base3DView::cameraToggled( bool selected )
{
    if(selected)
    {
        manager_->getToolManager()->setCurrentTool( move_camera_tool_ );

        // enable robot IK widget_name_.compare("MainView") == 0markers
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

void Base3DView::clearPointCloudRaycastRequests()
{
    flor_ocs_msgs::OCSSynchronize msg;
    msg.properties.push_back("Raycast Point Cloud");
    msg.reset.push_back(true);
    msg.visible.push_back(false);
    ocs_sync_pub_.publish(msg);
}

void Base3DView::clearPointCloudStereoRequests()
{
    flor_ocs_msgs::OCSSynchronize msg;
    msg.properties.push_back("Stereo Point Cloud");
    msg.reset.push_back(true);
    msg.visible.push_back(false);
    ocs_sync_pub_.publish(msg);
}

void Base3DView::clearPointCloudRegionRequests()
{
    flor_ocs_msgs::OCSSynchronize msg;
    msg.properties.push_back("LIDAR Point Cloud");
    msg.reset.push_back(true);
    msg.visible.push_back(false);
    ocs_sync_pub_.publish(msg);
}

void Base3DView::clearMapRequests()
{       
    flor_ocs_msgs::OCSSynchronize msg;
    msg.properties.push_back("Ground map");
    msg.reset.push_back(true);
    msg.visible.push_back(false);
    ocs_sync_pub_.publish(msg);
}

//when reset is pressed, reset all,
//when toggle is pressed , and sync is enabled, toggle all,
void Base3DView::synchronizeViews(const flor_ocs_msgs::OCSSynchronize::ConstPtr &msg)
{
    //check this msg contents across every rviz display
    for(int i=0;i<msg->properties.size();i++)
    {        
        bool groundMapReset = false;
        int num_displays = render_panel_->getManager()->getRootDisplayGroup()->numDisplays();
        for(int j = 0; j < num_displays; j++)
        {
            rviz::Display* display = render_panel_->getManager()->getRootDisplayGroup()->getDisplayAt(j);
            std::string display_name = display->getNameStd();

            if(msg->properties[i].compare(display_name) == 0)
            {                
                if(display_name.compare("Stereo Point Cloud") == 0)
                {
                    if(msg->reset[i]) //reset case
                    {
                        stereo_point_cloud_viewer_->setEnabled(false);
                        stereo_point_cloud_viewer_->setEnabled(true);
                    }
                    else // toggle case
                    {                        
                        stereo_point_cloud_viewer_->subProp("Alpha")->setValue(msg->visible[i] ? 1.0f : 0.0f);
                    }
                }
                else if(display_name.compare("Raycast Point Cloud") == 0)
                {
                    if(msg->reset[i]) //reset case
                    {
                        raycast_point_cloud_viewer_->setEnabled(false);
                        raycast_point_cloud_viewer_->setEnabled(true);
                    }
                    else  // toggle case
                    {
                        // we can't enable/disable point cloud requests, since the existing ones are lost if we disable them
                        raycast_point_cloud_viewer_->subProp("Alpha")->setValue(msg->visible[i] ? 1.0f : 0.0f);
                    }
                }
                else if(display_name.compare("LIDAR Point Cloud") == 0)
                {
                    if(msg->reset[i]) //reset case
                    {
                        region_point_cloud_viewer_->setEnabled(false);
                        region_point_cloud_viewer_->setEnabled(true);
                    }
                    else  // toggle case
                    {
                        //lidar_point_cloud_viewer_->setEnabled( selected );
                        region_point_cloud_viewer_->subProp("Alpha")->setValue(msg->visible[i] ? 1.0f : 0.0f);
                    }
                }
                else if(display_name.compare("Ground map") == 0)
                {                                        
                    if(msg->reset[i]) //reset case
                    {
                        groundMapReset = true;
                    }
                    else // toggle case
                    {
                        // we can't enable/disable gridmaps, since the existing ones are lost if we disable them
                        visualize_grid_map_ = msg->visible[i];
                        if(!msg->visible[i])
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
                }
                else if(display_name.compare("Octomap") == 0)
                {
                    //can only toggle octomap
                    octomap_->setEnabled(msg->visible[i]);
                }
                else if(display_name.compare("Notification System") == 0)
                {
                    //only toggle notification visibility
                    overlay_display_->setEnabled(msg->visible[i]);
                }
                else if(display_name.compare("Frustum Display") == 0)
                {
                    //toggle all frustums
                    for (std::map<std::string,rviz::Display*>::iterator it=frustum_map_.begin(); it!=frustum_map_.end(); ++it)
                    {
                        rviz::Display* frustum_display = it->second;
                        frustum_display->setEnabled(msg->visible[i]);
                    }
                    frustum_display_->setEnabled(msg->visible[i]);
                }

            }
        }
        if(groundMapReset)
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
    }
}

void Base3DView::defineFootstepGoal()
{
    manager_->getToolManager()->setCurrentTool( set_goal_tool_ );
}

void Base3DView::processGoalPose(const geometry_msgs::PoseStamped::ConstPtr &pose)
{
    // If a goal is set, need to go back to interactive marker tool
    manager_->getToolManager()->setCurrentTool( interactive_markers_tool_ );
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
    flor_atlas_current_mode_ = msg->control_mode;
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

    //notify on ui
    NotificationSystem::Instance()->notifyPassive("Template Inserted");
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

void Base3DView::selectOnDoubleClick(int x, int y)
{
    deselectAll();

    Q_EMIT queryContext(x,y);    
    if(active_context_name_.find("LeftArm") != std::string::npos)
        selectLeftArm();
    else if(active_context_name_.find("RightArm") != std::string::npos)
        selectRightArm();
    else if(active_context_name_.find("template") != std::string::npos)
        selectTemplate();
    else if(active_context_name_.find("footstep goal") != std::string::npos)
        selectFootstepGoal();
    else if(active_context_name_.find("footstep") != std::string::npos)
        selectFootstep();
}

//callback functions for context Menu
void Base3DView::insertTemplateContextMenu()
{
    if(!selected_template_path_.isEmpty())
    {
        insertTemplate(selected_template_path_);
    }
}



int Base3DView::findObjectContext(std::string obj_type)
{
    if(active_context_name_.find(obj_type) != std::string::npos)
    {
        // all selectable objects use the convention "object_type n",
        // so we look for the starting and ending indexes for n
        int start = obj_type.length()+1;
        int end = active_context_name_.size();
        QString number(active_context_name_.substr(start, end-start).c_str());
        //ROS_ERROR("%d %d %s",start,end,number.toStdString().c_str());
        bool ok;
        int t = number.toInt(&ok);
        if(ok) return t;
    }
    return -1;
}


void Base3DView::setStartingFootstep()
{
    int id;
    if((id = findObjectContext("footstep")) != -1)
        footstep_vis_manager_->setStartingFootstep(id/2); // divide by two since markers come in pairs of cube+text
}


void Base3DView::lockFootstep()
{
    int id;
    if((id = findObjectContext("footstep")) != -1)
        footstep_vis_manager_->lockFootstep(id/2); // divide by two since markers come in pairs of cube+text
}

void Base3DView::unlockFootstep()
{
    int id;
    if((id = findObjectContext("footstep")) != -1)
        footstep_vis_manager_->unlockFootstep(id/2); // divide by two since markers come in pairs of cube+text
}

void Base3DView::removeFootstep()
{
    int id;
    if((id = findObjectContext("footstep")) != -1)
        footstep_vis_manager_->removeFootstep(id/2); // divide by two since markers come in pairs of cube+text
}

void Base3DView::selectTemplate()
{
    int id;
    if((id = findObjectContext("template")) != -1)
    {
        deselectAll();

        flor_ocs_msgs::OCSObjectSelection cmd;
        cmd.type = flor_ocs_msgs::OCSObjectSelection::TEMPLATE;
        cmd.id = id;
        select_object_pub_.publish(cmd);
    }
}

void Base3DView::selectFootstep()
{
    int id;
    if((id = findObjectContext("footstep")) != -1)
    {
        deselectAll();

        flor_ocs_msgs::OCSObjectSelection cmd;
        cmd.type = flor_ocs_msgs::OCSObjectSelection::FOOTSTEP;
        cmd.id = id;
        select_object_pub_.publish(cmd);
    }
}

void Base3DView::selectFootstepGoal()
{
    int id;
    if((id = findObjectContext("footstep goal")) != -1)
    {
        deselectAll();

        flor_ocs_msgs::OCSObjectSelection cmd;
        cmd.type = flor_ocs_msgs::OCSObjectSelection::FOOTSTEP_GOAL;
        cmd.id = id;
        select_object_pub_.publish(cmd);
    }
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
        selectTemplate();

        ghost_pose_source_[flor_ocs_msgs::OCSObjectSelection::LEFT_ARM] = false;
        ghost_world_lock_[flor_ocs_msgs::OCSObjectSelection::LEFT_ARM] = false;


        ghost_pose_source_[flor_ocs_msgs::OCSObjectSelection::RIGHT_ARM] = false;
        ghost_world_lock_[flor_ocs_msgs::OCSObjectSelection::RIGHT_ARM] = false;
        return;
    }
    else if(id != -1) //locks arm
    {
        selectTemplate();

        ghost_pose_source_[arm] = true;
        ghost_world_lock_[arm] = true;
    }

}

void Base3DView::deselectAll()
{
    // disable all template markers
    Q_EMIT enableTemplateMarkers( false );

    // disable all footstep markers
    footstep_vis_manager_->enableFootstepMarkers( false );

    // disable all robot IK markers
    for( int i = 0; i < im_ghost_robot_.size(); i++ )
    {
        im_ghost_robot_[i]->setEnabled( false );
    }

    // enable stepplan markers
    footstep_vis_manager_->enableStepPlanMarkers( true );
}

void Base3DView::processObjectSelection(const flor_ocs_msgs::OCSObjectSelection::ConstPtr& msg)
{
    deselectAll();

    // enable loopback for both arms
    left_marker_moveit_loopback_ = true;
    right_marker_moveit_loopback_ = true;

    switch(msg->type)
    {
        case flor_ocs_msgs::OCSObjectSelection::END_EFFECTOR:
            // disable step plan markers
            footstep_vis_manager_->enableStepPlanMarkers( false );
            // enable template marker
            if(msg->id == flor_ocs_msgs::OCSObjectSelection::LEFT_ARM)
                left_marker_moveit_loopback_ = false;
            else if(msg->id == flor_ocs_msgs::OCSObjectSelection::RIGHT_ARM)
                right_marker_moveit_loopback_ = false;
            im_ghost_robot_[msg->id]->setEnabled( true );
            footstep_vis_manager_->enableStepPlanMarkers( false );
            break;
        case flor_ocs_msgs::OCSObjectSelection::TEMPLATE:
            // disable step plan markers
            footstep_vis_manager_->enableStepPlanMarkers( false );
            // enable template marker
            Q_EMIT enableTemplateMarker( msg->id, true );
            break;
        case flor_ocs_msgs::OCSObjectSelection::FOOTSTEP:
            // disable step plan markers
            footstep_vis_manager_->enableStepPlanMarkers( false );
            // id takes into account text marker as well, so we do this to find the real marker id
            footstep_vis_manager_->enableFootstepMarker( msg->id/2, true );
            break;
        case flor_ocs_msgs::OCSObjectSelection::FOOTSTEP_GOAL:
            // disable step plan markers
            footstep_vis_manager_->enableStepPlanMarkers( false );
            // id takes into account text marker as well, so we do this to find the real marker id
            footstep_vis_manager_->enableFootstepGoalMarker( msg->id/2, true );
            break;
        default:
            break;
    }
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

void Base3DView::removeTemplate(int id)
{
    flor_ocs_msgs::OCSTemplateRemove cmd;

    cmd.template_id = id;

    // publish template to be removed
    template_remove_pub_.publish( cmd );

    //notify
    NotificationSystem::Instance()->notifyPassive("Template Removed");
}

void Base3DView::emitQueryContext(int x,int y)
{
    Q_EMIT queryContext(x,y);
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

void Base3DView::processPelvisEndEffector(const geometry_msgs::PoseStamped::ConstPtr &pose)
{
    ghost_root_pose_ = pose->pose;
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
    sensor_msgs::JointState joint_states;

    joint_states.header.stamp = ros::Time::now();
    joint_states.header.frame_id = std::string("/")+hand+std::string("_hand_model/")+hand+"_palm";

    if(hand == "left")
        for(int i = 0; i < left_hand_joint_names_.size(); i++)
            joint_states.name.push_back(left_hand_joint_names_[i]);
    else
        for(int i = 0; i < right_hand_joint_names_.size(); i++)
            joint_states.name.push_back(right_hand_joint_names_[i]);

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

    // then we check the angle differeboundPercentnce
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
    else
        if(msg.topic == "/circular_pose")
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

void Base3DView::updateJointIcons(const std::string& name, const geometry_msgs::Pose& pose, double effortPercent, double boundPercent, bool ghost, int arrowDirection)
{
    std::string jointPositionIconName = name;

    if(ghost)
    {
        //joint icon plugin will not name joints with "ghost/" prefix, need to adjust
        jointPositionIconName = jointPositionIconName.substr(6,jointPositionIconName.size());
        //ghost joint marker still sends fingers, need to hide
        if(name.find("_f")!= std::string::npos && name.find("_j")!= std::string::npos)
        {
            ((rviz::JointMarkerDisplayCustom*)ghost_joint_arrows_)->setJointAlpha(0,jointPositionIconName);
            return;
        }
    }
    //want to disable a marker that has already been created
    if(disable_joint_markers_ )
    {
        //disable every marker
        if(jointDisplayMap.find(name) != jointDisplayMap.end())
            jointDisplayMap[name]->subProp( "Alpha" )->setValue( 0.0f );
        ((rviz::JointMarkerDisplayCustom*)ghost_joint_arrows_)->setJointAlpha(0,jointPositionIconName);
        ((rviz::JointMarkerDisplayCustom*)joint_arrows_)->setJointAlpha(0,jointPositionIconName);
        return;
    }

    //calculate error based on boundPercent- joint location relative to max or min
    int errorCode = 0;
    if(effortPercent >=.9 ) //effort error
        errorCode = 1;
    else if(effortPercent >=.75) //effort warn
        errorCode = 2;

    //handle effort Discs
    if(errorCode != 0 && !ghost)
    {
        //align bounding boxes to correct axis based on joint name
        float x = 0.0018;
        float y = 0.0018;
        float z = 0.0018;
        QString str(name.c_str());
        str = str.mid(str.length() - 1, 1); //last char
        if(str == "x")
            x = 0.0003f;
        else if(str == "y")
            y = 0.0003f;
        else
            z = 0.0003f;

        //only create joints bounding boxes if they haven't been created and on error
        if(jointDisplayMap.find(name) == jointDisplayMap.end())
        {
            //create bounding box
            jointDisplayMap[name] = manager_->createDisplay( "rviz/BoundingObjectDisplayCustom", (name + "_WarningIcon").c_str(), true );
            jointDisplayMap[name]->subProp( "Name" )->setValue( name.c_str() );
            jointDisplayMap[name]->subProp( "Alpha" )->setValue( 0.5f );
            jointDisplayMap[name]->subProp( "Color" )->setValue( QColor( 0, 255, 0 ) );
            ((rviz::VectorProperty*)jointDisplayMap[name]->subProp( "Scale" ))->setVector( Ogre::Vector3(x, y, z) );
        }

        Ogre::Vector3 jointPosition(pose.position.x,pose.position.y,pose.position.z);
        Ogre::Quaternion jointRotation(pose.orientation.w,pose.orientation.x,pose.orientation.y,pose.orientation.z);
        ((rviz::VectorProperty*)jointDisplayMap[name]->subProp("Position"))->setVector(jointPosition);
        ((rviz::QuaternionProperty*)jointDisplayMap[name]->subProp("Rotation"))->setQuaternion(jointRotation);

        if(errorCode == 1)
        {
            //red
            jointDisplayMap[name]->subProp( "Alpha" )->setValue( 0.5f );
            jointDisplayMap[name]->subProp("Color")->setValue( QColor( 255, 0, 0 ) );
        }
        else if(errorCode == 2)
        {
            //orange
            jointDisplayMap[name]->subProp( "Alpha" )->setValue( 0.5f );
            jointDisplayMap[name]->subProp("Color")->setValue( QColor( 255, 127, 0 ) );                      
        }      

    }
    else if (jointDisplayMap.find(name) != jointDisplayMap.end())
    {
        //make invisible
        jointDisplayMap[name]->subProp( "Alpha" )->setValue( 0 );
    }

    //handle bounds error joint arrows
    if(boundPercent <=.1)
    {
        //linearly interpolate color and alpha based on ratio of boundPercentage under .1
        float p = boundPercent / .1;
        float alpha = 1.0 - p; //want alpha to be atleast .3
        float green = p;
        //increase alpha as boundPercent decreases
        //yellow  to red as boundPercent decreases
        QColor color;
        color.setRedF(1);
        color.setGreenF(green);
        color.setBlueF(0);
        if(ghost)
        {
            ((rviz::JointMarkerDisplayCustom*)ghost_joint_arrows_)->setArrowDirection(jointPositionIconName,arrowDirection);
            ((rviz::JointMarkerDisplayCustom*)ghost_joint_arrows_)->setJointColor(color,jointPositionIconName);
            ((rviz::JointMarkerDisplayCustom*)ghost_joint_arrows_)->setJointAlpha(alpha,jointPositionIconName);
        }
        else
        {
            ((rviz::JointMarkerDisplayCustom*)joint_arrows_)->setArrowDirection(jointPositionIconName,arrowDirection);
            ((rviz::JointMarkerDisplayCustom*)joint_arrows_)->setJointColor(color,jointPositionIconName);
            ((rviz::JointMarkerDisplayCustom*)joint_arrows_)->setJointAlpha(alpha,jointPositionIconName);
        }
    }
    else
    {
        //no joint arrow should be shown when joint bounds is okay
        if(ghost)
            ((rviz::JointMarkerDisplayCustom*)ghost_joint_arrows_)->setJointAlpha(0,jointPositionIconName);
        else
            ((rviz::JointMarkerDisplayCustom*)joint_arrows_)->setJointAlpha(0,jointPositionIconName);
    }

}

void Base3DView::processJointStates(const sensor_msgs::JointState::ConstPtr &states)
{
    // get pelvis pose
    Ogre::Vector3 position(0,0,0);
    Ogre::Quaternion orientation(1,0,0,0);
    transform(position, orientation, "/pelvis", "/world");

    geometry_msgs::PoseStamped root_pose;
    root_pose.pose.position.x = position.x;
    root_pose.pose.position.y = position.y;
    root_pose.pose.position.z = position.z;
    root_pose.pose.orientation.x = orientation.x;
    root_pose.pose.orientation.y = orientation.y;
    root_pose.pose.orientation.z = orientation.z;
    root_pose.pose.orientation.w = orientation.w;
    root_pose.header.frame_id = "/world";
    root_pose.header.stamp = ros::Time::now();

    MoveItOcsModel* robot_state = RobotStateManager::Instance()->getRobotStateSingleton();

    // set robotstate joint states
    robot_state->setJointStates(*states);
    // and root transform
    robot_state->setRootTransform(root_pose);

    for(int i = 0; i < states->name.size(); i++)
    {
        const moveit::core::JointModel* joint =  robot_state->getJointModel(states->name[i]);
        //ignore unnecessary joints
        if (joint->getType() == moveit::core::JointModel::PLANAR || joint->getType() == moveit::core::JointModel::FLOATING)
          continue;
        if (joint->getType() == moveit::core::JointModel::REVOLUTE)
          if (static_cast<const moveit::core::RevoluteJointModel*>(joint)->isContinuous())
            continue;
        //calculate joint position percentage relative to max/min limit
        const moveit::core::JointModel::Bounds& bounds = joint->getVariableBounds();
        double distance = bounds[0].max_position_ - bounds[0].min_position_;
        double boundPercent = robot_state->getMinDistanceToPositionBounds(joint) / distance;

        //calculate which limit joint was closer to
        int direction;
        double max = bounds[0].max_position_;
        double min = bounds[0].min_position_;
        double position = states->position[i];
        //need to handle negative mins
        if(min < 0)
        {
            //offset everything by min to normalize difference calculation
            max += abs(min);
            position += abs(min);
            min = 0;
        }
        if(max - position <= position - min)
            direction = -1;
        else
            direction = 1;
        //ROS_ERROR("max: %f min: %f  pos: %f  direction:%d",bounds[0].max_position_ ,bounds[0].min_position_,states->position[i],direction );

        geometry_msgs::Pose pose;
        std::string link_name = ((rviz::RobotDisplayCustom*)robot_model_)->getChildLinkName(states->name[i]);
        robot_state->getLinkPose(link_name,pose);

        if(!robot_model_->isEnabled())
        {
            //hide all icons if robot is not enabled
            for (std::map<std::string,rviz::Display*>::iterator it=jointDisplayMap.begin(); it!=jointDisplayMap.end(); ++it)
            {
                if(!it->second->subProp("Name")->getValue().toString().contains("ghost"))
                    it->second->setEnabled(false);
            }
        }
        else
        {
            //show all icons
            for (std::map<std::string,rviz::Display*>::iterator it=jointDisplayMap.begin(); it!=jointDisplayMap.end(); ++it)
            {
                if(!it->second->subProp("Name")->getValue().toString().contains("ghost"))
                    it->second->setEnabled(true);
            }
        }

        double jointEffortPercent = ((robot_state->getJointEffortLimit(states->name[i]) != 0 && states->effort.size() > i) ?
                                    std::abs(states->effort[i]) / robot_state->getJointEffortLimit(states->name[i]) :
                                    0.0);
        updateJointIcons(states->name[i], pose, jointEffortPercent,boundPercent,false,direction);
    }

    if(snap_ghost_to_robot_)
    {
        ghost_joint_state_pub_.publish(states);
        ghost_root_pose_pub_.publish(root_pose);

        flor_ocs_msgs::OCSInteractiveMarkerUpdate cmd;
        cmd.topic = "/pelvis_pose_marker";
        cmd.pose = root_pose;
        interactive_marker_update_pub_.publish(cmd);

        pelvis_marker_pose_pub_.publish(root_pose);

        snap_ghost_to_robot_ = false;
    }
}

//goes through all scene nodes and sets position in render queue based on object type
/*void Base3DView::setSceneNodeRenderGroup(Ogre::SceneNode* sceneNode, int queueOffset)
{
    // warning: some rviz display may be disjointed and not have attached objects
    for(int i =0;i<sceneNode->numAttachedObjects();i++)
    {
        Ogre::MovableObject* obj =  sceneNode->getAttachedObject(i);
        obj->setRenderQueueGroupAndPriority(Ogre::RENDER_QUEUE_MAIN,100);
        if(obj->getMovableType().compare("Entity") == 0)
        {
            //only subentities have materials...
            for(int e = 0; e < ((Ogre::Entity*)obj)->getNumSubEntities(); e++)
            {
                //usually only 1 technique
                for(int t = 0; t < ((Ogre::Entity*)obj)->getSubEntity(e)->getMaterial()->getNumTechniques(); t++)
                {
                    for(int p = 0; p < ((Ogre::Entity*)obj)->getSubEntity(e)->getMaterial()->getTechnique(t)->getNumPasses(); p++)
                    {
                        //transparent object?
                        if(((Ogre::Entity*)obj)->getSubEntity(e)->getMaterial()->getTechnique(t)->getPass(p)->getAmbient().a < 0.95f ||
                           ((Ogre::Entity*)obj)->getSubEntity(e)->getMaterial()->getTechnique(t)->getPass(p)->getDiffuse().a < 0.95f)//Transparent
                        {
                            obj->setRenderQueueGroup(Ogre::RENDER_QUEUE_MAIN + queueOffset);
                        }
                        else // opaque object
                        {
                            obj->setRenderQueueGroup(Ogre::RENDER_QUEUE_MAIN);
                        }
                    }
                }
            }
        }
        //interactive markers/ other manual objects
        else if(obj->getMovableType().compare("ManualObject") == 0)
        {
            obj->setRenderQueueGroup(Ogre::RENDER_QUEUE_MAIN + queueOffset);
        }
    }
    for(int i=0;i<sceneNode->numChildren();i++)
    {
        //recurse for all children Scene nodes in this level
        setSceneNodeRenderGroup(((Ogre::SceneNode*)sceneNode->getChild(i)),queueOffset);
    }
}

void Base3DView::setRenderOrder()
{

    //  Render Queue Main |  PointClouds, Robot (opaque parts) ,opaque objects
    //                +1  |  Transparent Objects

    int num_displays = render_panel_->getManager()->getRootDisplayGroup()->numDisplays();
    for(int i = 0; i < num_displays; i++)
    {
        rviz::Display* display = render_panel_->getManager()->getRootDisplayGroup()->getDisplayAt(i);
        std::string display_name = display->getNameStd();
        //camera should be unaffected by render order
        if(display_name.find("Camera") == std::string::npos)
            setSceneNodeRenderGroup(display->getSceneNode(), 1);
    }
}

void Base3DView::resetRenderOrder()
{
    //  Render Queue Main |  PointClouds, Robot (opaque parts) ,opaque objects
    //                +1  |  Transparent Objects

    int num_displays = render_panel_->getManager()->getRootDisplayGroup()->numDisplays();
    for(int i = 0; i < num_displays; i++)
    {
        rviz::Display* display = render_panel_->getManager()->getRootDisplayGroup()->getDisplayAt(i);
        std::string display_name = display->getNameStd();
        //camera should be unaffected by render order
        if(display_name.find("Camera") == std::string::npos)
            setSceneNodeRenderGroup(display->getSceneNode(), 0);
    }
}

void Base3DView::setRobotOccludedRender()
{
    //configures robot to be seen behind objects by setting 2 render passes on all objects that draw regardless of depth in world
    //1 pass is outline, 2nd is shaded robot

    M_NameToLink links = ((rviz::RobotDisplayCustom*)robot_model_)->getRobotCustom()->getLinks();
    M_NameToLink::iterator it = links.begin();
    M_NameToLink::iterator end = links.end();
   //iterate over all links in robot
   for ( ; it != end; ++it )
   {
       //need to scale down robot then outline robot, must ensure that outline does not exceed scale of 1 on robot
       const char *vertex_outline_code =
               "void main(void){\n"
                  "vec4 tPos   = vec4(gl_Vertex + gl_Normal *0, 1.0);\n"
                  "gl_Position = gl_ModelViewProjectionMatrix * tPos;\n"
               "}\n";
       //black
       const char *fragment_outline_code =
           "void main()\n"
           "{\n"
           "    gl_FragColor = vec4(0,0,0,1);\n"
           "}\n";

       //scales down model uniformly using normals
       const char *vertex_solid_code =
                  "void main()\n"
                  "{\n"
                    "vec4 tPos   = vec4(gl_Vertex + gl_Normal *-0.008, 1.0);\n"
                    "gl_Position = gl_ModelViewProjectionMatrix * tPos;\n"
                  "}\n";

       // gray
       const char *fragment_solid_code =
              "void main()\n"
              "{\n"
              "    gl_FragColor = vec4(0.65,0.65,0.65,1);\n"
              "}\n";


       //fun testing shader
//               "uniform float time;\n"
//        "float length2(vec2 p) { return dot(p, p); }\n"

//        "float noise(vec2 p){return fract(sin(fract(sin(p.x) * (4313.13311)) + p.y) * 3131.0011);}\n"

//        "float worley(vec2 p) {\n"
//            "float d = 1e30;\n"
//            "for (int xo = -1; xo <= 1; ++xo)\n"
//            "for (int yo = -1; yo <= 1; ++yo) {\n"
//                "vec2 tp = floor(p) + vec2(xo, yo);\n"
//                "d = min(d, length2(p - tp - vec2(noise(tp))));}\n"
//            "return 3.*exp(-4.*abs(2.*d - 1.));}\n"

//        "float fworley(vec2 p) {\n"
//            "return sqrt(sqrt(sqrt(\n"
//                "worley(p*32. + 4.3 + time*.125) *\n"
//                "sqrt(worley(p * 64. + 5.3 + .25 * -.0625)) *\n"
//                "sqrt(sqrt(worley(p * -128. + 7.3))))));}\n"

//               "void main() {\n"
//               "vec2 resolution = vec2(1920,1080);\n"
//            "vec2 uv = gl_FragCoord.xy /resolution.xy;\n"
//            "float t = fworley(uv * resolution.xy / 1800.);\n"
//            "t *= exp(-length2(abs(2.*uv - 1.)));\n"
//            "gl_FragColor = vec4(t * vec3(.1 + pow(t, 1.-t), 1.8*t, 1.8), 1.);}\n";




       //create shader programs
       Ogre::HighLevelGpuProgramPtr vp = Ogre::HighLevelGpuProgramManager::getSingleton()
                   .createProgram("SolidVertex", "General", "glsl", Ogre::GPT_VERTEX_PROGRAM);
       vp->setSource(vertex_solid_code);
       vp->load();

       Ogre::HighLevelGpuProgramPtr fp = Ogre::HighLevelGpuProgramManager::getSingleton()
                   .createProgram("SolidFragment", "General", "glsl", Ogre::GPT_FRAGMENT_PROGRAM);
       fp->setSource(fragment_solid_code);
       fp->load();

       Ogre::HighLevelGpuProgramPtr vpOutline = Ogre::HighLevelGpuProgramManager::getSingleton()
                   .createProgram("OutlineVertex", "General", "glsl", Ogre::GPT_VERTEX_PROGRAM);
       vpOutline->setSource(vertex_outline_code);
       vpOutline->load();

       Ogre::HighLevelGpuProgramPtr fpOutline = Ogre::HighLevelGpuProgramManager::getSingleton()
                   .createProgram("OutlineFragment", "General", "glsl", Ogre::GPT_FRAGMENT_PROGRAM);
       fpOutline->setSource(fragment_outline_code);
       fpOutline->load();

       rviz::RobotLinkCustom* info = it->second;
       M_SubEntityToMaterial materials = info->getMaterials();
       M_SubEntityToMaterial::iterator iter = materials.begin();
       M_SubEntityToMaterial::iterator ender = materials.end();

       //for all materials in this link?
       for(;iter != ender; ++iter)
       {
           Ogre::MaterialPtr material =  iter->second;
           //create outline pass
           for(int i=0;i<material->getNumTechniques();i++)
           {
               //materials can be shared between different objects, not necessary to recreate passes if already created
               if(material->getTechnique(i)->getPass(0)->getName() == "OutlinePass")
                   continue;

               Ogre::Pass * outlinePass = material->getTechnique(i)->createPass();
                //will be written all the time, but will only show up when other object/passes aren't written due to depth checking
               //outlinePass->setPolygonMode(Ogre::PM_WIREFRAME);
               outlinePass->setDepthCheckEnabled(false);
               outlinePass->setDepthWriteEnabled(false);
               outlinePass->setVertexProgram("OutlineVertex");
               outlinePass->setFragmentProgram("OutlineFragment");
               outlinePass->setName("OutlinePass");
               //outlinePass->setFog(true,Ogre::FOG_LINEAR);
               material->getTechnique(i)->movePass(outlinePass->getIndex(),0);

               //create pass for occluded robot
               Ogre::Pass * occludedPass = material->getTechnique(i)->createPass();
               occludedPass->setDepthCheckEnabled(false);
               occludedPass->setDepthWriteEnabled(false);
               occludedPass->setVertexProgram("SolidVertex");
               occludedPass->setFragmentProgram("SolidFragment");
               occludedPass->setName("OccludedPass");
               material->getTechnique(i )->movePass(occludedPass->getIndex(),1);

    //           double r = (double)rand() / RAND_MAX;
    //           0 + r * (1 - 0);
    //           double g = (double)rand() / RAND_MAX;
    //           0 + g * (1 - 0);
    //           double b = (double)rand() / RAND_MAX;
    //           0 + b * (1 - 0);
               //candy bot mode on
    //           pass->setShininess(127);
    //           pass->setLightingEnabled(true);
    //           pass->setSelfIllumination(r,g,b);
           }
       }
   }

}

void Base3DView::disableRobotOccludedRender()
{
    M_NameToLink links = ((rviz::RobotDisplayCustom*)robot_model_)->getRobotCustom()->getLinks();
    M_NameToLink::iterator it = links.begin();
    M_NameToLink::iterator end = links.end();
   //iterate over all links in robot
   for ( ; it != end; ++it )
   {
       rviz::RobotLinkCustom* info = it->second;
       M_SubEntityToMaterial materials = info->getMaterials();
       M_SubEntityToMaterial::iterator iter = materials.begin();
       M_SubEntityToMaterial::iterator ender = materials.end();

       //for all materials in this link?
       for(;iter != ender; ++iter)
       {
           Ogre::MaterialPtr material =  iter->second;
           //delete occlusion passes
           for(int i=0;i<material->getNumTechniques();i++)
           {
               for(int j=0;j<material->getTechnique(i)->getNumPasses();j++)
               {
                   if(material->getTechnique(i)->getPass(j)->getName() == "OutlinePass")
                   {
                       material->getTechnique(i)->removePass(j);
                       break;
                   }
               }
               for(int j=0;j<material->getTechnique(i)->getNumPasses();j++)
               {
                   if( material->getTechnique(i)->getPass(j)->getName() == "OccludedPass")
                   {
                       material->getTechnique(i)->removePass(j);
                       break;
                   }
               }
           }
       }
   }

   //place everything in the same position in render queue
   int num_displays = render_panel_->getManager()->getRootDisplayGroup()->numDisplays();
   for(int i = 0; i < num_displays; i++)
   {
       rviz::Display* display = render_panel_->getManager()->getRootDisplayGroup()->getDisplayAt(i);
       setSceneNodeRenderGroup(display->getSceneNode(), 0);
   }
}*/

void Base3DView::processGhostJointStates(const sensor_msgs::JointState::ConstPtr& states)
{
    //store latest state for toggle updates
    latest_ghost_joint_state_ = states;

    MoveItOcsModel* ghost_robot_state = RobotStateManager::Instance()->getGhostRobotStateSingleton();

    ghost_robot_state->setJointStates(*states);

    for(int i = 0; i < states->name.size(); i++)
    {
        //ignore finger joints on atlas ghost
        if(states->name[i].find("_f") != std::string::npos && states->name[i].find("_j")!= std::string::npos && ghost_robot_state->getRobotName().find("atlas") != std::string::npos )
        {
            continue;
        }

        const moveit::core::JointModel* joint =  ghost_robot_state->getJointModel(states->name[i]);
        //ignore unnecessary joints
        if (joint->getType() == moveit::core::JointModel::PLANAR || joint->getType() == moveit::core::JointModel::FLOATING)
          continue;
        if (joint->getType() == moveit::core::JointModel::REVOLUTE)
          if (static_cast<const moveit::core::RevoluteJointModel*>(joint)->isContinuous())
            continue;

        //calculate joint position percentage relative to max/min limit
        const moveit::core::JointModel::Bounds& bounds = joint->getVariableBounds();
        double distance = bounds[0].max_position_ - bounds[0].min_position_;
        double boundPercent = ghost_robot_state->getMinDistanceToPositionBounds(joint) / distance;

        //calculate which limit joint was closer to
        int direction;
        double max = bounds[0].max_position_;
        double min = bounds[0].min_position_;
        double position = states->position[i];
        //need to handle negative mins
        if(min < 0)
        {
            //offset everything by min to normalize difference calculation
            max += abs(min);
            position += abs(min);
            min = 0;
        }
        if(max - position <= position - min)
            direction = -1;
        else
            direction = 1;


        geometry_msgs::Pose pose;
        std::string link_name = ((rviz::RobotDisplayCustom*)robot_model_)->getChildLinkName(states->name[i]);

        ghost_robot_state->getLinkPose(link_name,pose);

        // accumulate all the transforms for root pose
        Ogre::Vector3 rootPosition(ghost_root_pose_.position.x,ghost_root_pose_.position.y,ghost_root_pose_.position.z);
        Ogre::Quaternion rootRotation(ghost_root_pose_.orientation.w,ghost_root_pose_.orientation.x,ghost_root_pose_.orientation.y,ghost_root_pose_.orientation.z);
        Ogre::Vector3 linkPosition(pose.position.x,pose.position.y,pose.position.z);
        Ogre::Quaternion linkRotation(pose.orientation.w,pose.orientation.x,pose.orientation.y,pose.orientation.z);
        linkPosition = rootPosition + rootRotation * linkPosition;
        linkRotation = rootRotation * linkRotation;
        pose.position.x = linkPosition.x;
        pose.position.y = linkPosition.y;
        pose.position.z = linkPosition.z;
        pose.orientation.x = linkRotation.x;
        pose.orientation.y = linkRotation.y;
        pose.orientation.z = linkRotation.z;
        pose.orientation.w = linkRotation.w;

        if(!ghost_robot_model_->isEnabled())
        {
            //hide all icons if ghost robot is not enabled
            for (std::map<std::string,rviz::Display*>::iterator it=jointDisplayMap.begin(); it!=jointDisplayMap.end(); ++it)
            {
                if(it->second->subProp("Name")->getValue().toString().contains("ghost"))
                    it->second->setEnabled(false);
            }
        }

        updateJointIcons(std::string("ghost/")+states->name[i], pose, 0.0, boundPercent,true,direction);
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
    send_footstep_goal_pub_.publish(end_effector_pose_list_["/pelvis_pose_marker"]);
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

    marker_published_ = 3;
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
        keys_pressed_list_.push_back(key_event->keystr);
    else
        keys_pressed_list_.erase(std::remove(keys_pressed_list_.begin(), keys_pressed_list_.end(), key_event->keystr), keys_pressed_list_.end());

    // process hotkeys
    bool ctrl_is_pressed = (std::find(keys_pressed_list_.begin(), keys_pressed_list_.end(), "Control_") != keys_pressed_list_.end());
    bool shift_is_pressed = (std::find(keys_pressed_list_.begin(), keys_pressed_list_.end(), "Shift_") != keys_pressed_list_.end());
    bool alt_is_pressed = (std::find(keys_pressed_list_.begin(), keys_pressed_list_.end(), "Alt_") != keys_pressed_list_.end());

    if(key_event->keystr == "Escape" && key_event->state) // 'esc'
    {
        // reset everything
        deselectAll();
        manager_->getToolManager()->setCurrentTool( interactive_markers_tool_ );
    }
    else if(key_event->keystr == "q" && key_event->state && ctrl_is_pressed) // ctrl+q
    {
        // robot model visibility
        robotModelToggled(!robot_model_->isEnabled());
    }
    else if(key_event->keystr == "w" && key_event->state && ctrl_is_pressed) // ctrl+w
    {
        // ghost visibility
        simulationRobotToggled(!ghost_robot_model_->isEnabled());
    }
    else if(key_event->keystr == "1" && key_event->state && ctrl_is_pressed) // ctrl+1
    {
        // reset point clouds
        clearPointCloudRaycastRequests();
        clearPointCloudRegionRequests();
        clearPointCloudStereoRequests();
    }
    else if(key_event->keystr == "9" && key_event->state && ctrl_is_pressed) // ctrl+9
    {
        // rainbow color
        region_point_cloud_viewer_->subProp( "Color Transformer" )->setValue( "AxisColor" );
    }
    else if(key_event->keystr == "0" && key_event->state && ctrl_is_pressed) // ctrl+0
    {
        // intensity
        region_point_cloud_viewer_->subProp( "Color Transformer" )->setValue( "Intensity" );
    }
    else if(key_event->keystr == "g" && key_event->state && ctrl_is_pressed) // ctrl+g
    {
        // define a step goal
        defineFootstepGoal();
    }
    else if(key_event->keystr == "h" && key_event->state && ctrl_is_pressed) // ctrl+h
    {
        // request plan
        if(footstep_vis_manager_->hasGoal())
            footstep_vis_manager_->requestStepPlan();
    }
    else if(key_event->keystr == "j" && key_event->state && ctrl_is_pressed) // ctrl+j
    {
        // request plan
        if(footstep_vis_manager_->hasValidStepPlan())
            footstep_vis_manager_->requestExecuteStepPlan();
    }
    else if(ctrl_is_pressed && alt_is_pressed) //emergency stop
    {
        stop_button_->setVisible(true);
        stop_button_->setGeometry(this->geometry().bottomRight().x()/2 - 200,this->geometry().bottomRight().y()/2 - 150,400,300);
    }
    else if(shift_is_pressed && !shift_pressed_)
    {
        //Lock translation during rotation
        flor_ocs_msgs::OCSControlMode msgMode;
        if(interactive_marker_mode_ < IM_MODE_OFFSET)
            msgMode.manipulationMode = interactive_marker_mode_ + IM_MODE_OFFSET;
        else
            msgMode.manipulationMode = interactive_marker_mode_ - IM_MODE_OFFSET;
        interactive_marker_server_mode_pub_.publish(msgMode);
        shift_pressed_ = true;
    }
    else
    {
        stop_button_->setVisible(false);

        //Unclock translation during rotation
        if(shift_pressed_)
        {
            flor_ocs_msgs::OCSControlMode msgMode;
            if(interactive_marker_mode_ < IM_MODE_OFFSET)//Check if mode is 0, 1 or 2
                msgMode.manipulationMode = interactive_marker_mode_;
            else//means that shift is pressed
                msgMode.manipulationMode = interactive_marker_mode_ - IM_MODE_OFFSET;
            interactive_marker_server_mode_pub_.publish(msgMode);
            shift_pressed_ = false;
        }

    }

}

void Base3DView::processInteractiveMarkerMode(const flor_ocs_msgs::OCSControlMode::ConstPtr& msg)
{
    //Update the im to current
    interactive_marker_mode_ = msg->manipulationMode;
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
