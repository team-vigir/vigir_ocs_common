/* 
 * Main3DView class implementation.
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

#include "rviz/visualization_manager.h"
#include "rviz/render_panel.h"
#include "rviz/display.h"
#include "rviz/frame_manager.h"
#include "rviz/tool_manager.h"
#include "main_3d_view.h"

// Constructor for Main3DView.  This does most of the work of the class.
Main3DView::Main3DView( QWidget* parent )
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
  // Add support for selection
  move_camera_tool_ = manager_->getToolManager()->addTool( "rviz/MoveCamera" );

	// Make the interaction tool the currently selected one
  manager_->getToolManager()->setCurrentTool(interactive_markers_tool_);

	// Add interactive markers Stefan's markers and IK implementation
	interactive_marker_[0] = manager_->createDisplay( "rviz/InteractiveMarkers", "Interactive marker 1", true );
	interactive_marker_[0]->subProp( "Update Topic" )->setValue( "/l_arm_pose_marker/update" );
	interactive_marker_[1] = manager_->createDisplay( "rviz/InteractiveMarkers", "Interactive marker 2", true );
	interactive_marker_[1]->subProp( "Update Topic" )->setValue( "/l_leg_pose_marker/update" );
	interactive_marker_[2] = manager_->createDisplay( "rviz/InteractiveMarkers", "Interactive marker 3", true );
	interactive_marker_[2]->subProp( "Update Topic" )->setValue( "/r_arm_pose_marker/update" );
	interactive_marker_[3] = manager_->createDisplay( "rviz/InteractiveMarkers", "Interactive marker 4", true );
	interactive_marker_[3]->subProp( "Update Topic" )->setValue( "/r_leg_pose_marker/update" );
}

// Destructor.
Main3DView::~Main3DView()
{
  delete manager_;
}

