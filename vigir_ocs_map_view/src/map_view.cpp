/* 
 * MapView class implementation.
 *
 * Author: Felipe Bacim.
 *
 * Based on librviz_tutorials.
 *
 * Latest changes (12/08/2012):
 */

#include <QLabel>
#include <QVBoxLayout>
#include <QHBoxLayout>

#include "rviz/visualization_manager.h"
#include "rviz/render_panel.h"
#include "rviz/display.h"
#include "rviz/frame_manager.h"
#include "rviz/tool_manager.h"
#include "rviz/properties/property_tree_widget.h"
#include "rviz/view_controller.h"
#include "rviz/view_manager.h"
#include "map_view.h"

// Constructor for MapView.  This does most of the work of the class.
MapView::MapView( QWidget* parent )
    : QWidget( parent )
{
    // Create a new label for this widget.
    //QLabel* robot_model_label = new QLabel( "rviz/RobotModel" );

    // Construct and lay out render panel.
    render_panel_ = new rviz::RenderPanel();
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
    manager_->setFixedFrame("/pelvis");

    // Add support for interactive markers
    interactive_markers_tool_ = manager_->getToolManager()->addTool( "rviz/Interact" );
    // Add support for selection
    selection_tool_ = manager_->getToolManager()->addTool( "rviz/Select" );
    // Add support for camera movement
    move_camera_tool_ = manager_->getToolManager()->addTool( "rviz/MoveCamera" );
    // Add support for goal specification/vector navigation
    set_goal_tool_ = manager_->getToolManager()->addTool( "rviz/SetGoal" );

    // Make the interaction tool the currently selected one
    //manager_->getToolManager()->setCurrentTool( interactive_markers_tool_ );
    manager_->getToolManager()->setCurrentTool( move_camera_tool_ );

    // Add interactive markers Stefan's markers and IK implementation
    interactive_marker_[0] = manager_->createDisplay( "rviz/InteractiveMarkers", "Interactive marker 1", true );
    interactive_marker_[0]->subProp( "Update Topic" )->setValue( "/l_arm_pose_marker/update" );
    interactive_marker_[1] = manager_->createDisplay( "rviz/InteractiveMarkers", "Interactive marker 2", true );
    interactive_marker_[1]->subProp( "Update Topic" )->setValue( "/l_leg_pose_marker/update" );
    interactive_marker_[2] = manager_->createDisplay( "rviz/InteractiveMarkers", "Interactive marker 3", true );
    interactive_marker_[2]->subProp( "Update Topic" )->setValue( "/r_arm_pose_marker/update" );
    interactive_marker_[3] = manager_->createDisplay( "rviz/InteractiveMarkers", "Interactive marker 4", true );
    interactive_marker_[3]->subProp( "Update Topic" )->setValue( "/r_leg_pose_marker/update" );

    // Create a LaserScan display.
    laser_scan_ = manager_->createDisplay( "rviz/LaserScan", "Laser Scan", false );
    ROS_ASSERT( laser_scan_ != NULL );

    laser_scan_->subProp( "Topic" )->setValue( "/multisense_sl/laser/scan" );
    laser_scan_->subProp( "Size (m)" )->setValue( 0.1 );
    laser_scan_->subProp( "Decay Time" )->setValue( 1 );

    // Create a MarkerArray display.
    marker_array_ = manager_->createDisplay( "rviz/MarkerArray", "MarkerArray", true );
    ROS_ASSERT( marker_array_ != NULL );

    marker_array_->subProp( "Marker Topic" )->setValue( "/occupied_cells_vis_array" );

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

    // Set topic that will be used as 0,0,0 -> reference for all the other transforms
    // IMPORTANT: WITHOUT THIS, ALL THE DIFFERENT PARTS OF THE ROBOT MODEL WILL BE DISPLAYED AT 0,0,0
    manager_->getFrameManager()->setFixedFrame("/pelvis");
    
    rviz::ViewManager* view_man_ = manager_->getViewManager();
    
    view_man_->setCurrentFrom( view_man_->create( "rviz/TopDownOrtho" ) );
    
    //Property* prop = view_man_->getPropertyModel()->getProp( index );
		//if( ViewController* view = qobject_cast<ViewController*>( prop ))
		//{
		//  view_man_->setCurrentFrom( view );
		//}
}

// Destructor.
MapView::~MapView()
{
    delete manager_;
}

void MapView::robotModelToggled( bool selected )
{
    robot_model_->setEnabled( selected );
}

void MapView::lidarPointCloudToggled( bool selected )
{
    lidar_point_cloud_viewer_->setEnabled( selected );
}

void MapView::stereoPointCloudToggled( bool selected )
{
    stereo_point_cloud_viewer_->setEnabled( selected );
}

void MapView::laserScanToggled( bool selected )
{
    laser_scan_->setEnabled( selected );
}

void MapView::markerArrayToggled( bool selected )
{
    marker_array_->setEnabled( selected );
}

void MapView::waypointToggled( bool selected )
{
    //if(selected)
    //    manager_->getToolManager()->setCurrentTool( move_camera_tool_ );
}

void MapView::vectorToggled( bool selected )
{
    if(selected)
        manager_->getToolManager()->setCurrentTool( set_goal_tool_ );
}

void MapView::joystickToggled( bool selected )
{
    if(selected)
        manager_->getToolManager()->setCurrentTool( selection_tool_ );
}
