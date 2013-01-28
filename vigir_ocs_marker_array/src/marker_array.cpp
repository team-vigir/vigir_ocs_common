/* 
 * MarkerArray class implementation.
 * 
 * Author: Felipe Bacim.
 * 
 * Based on librviz_tutorials.
 * 
 * Latest changes (12/03/2012):
 * 
 */

#include <QLabel>
#include <QVBoxLayout>

#include "rviz/visualization_manager.h"
#include "rviz/render_panel.h"
#include "rviz/display.h"
#include "rviz/frame_manager.h"
#include "marker_array.h"

// Constructor for MarkerArray.  This does most of the work of the class.
MarkerArray::MarkerArray( QWidget* parent )
  : QWidget( parent )
{
  // Create a new label for this widget.
  QLabel* marker_array_label = new QLabel( "rviz/MarkerArray" );
  
  // Construct and lay out render panel.
  render_panel_ = new rviz::RenderPanel();
  QVBoxLayout* main_layout = new QVBoxLayout;
  main_layout->addWidget( marker_array_label );
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
  
  // Create a MarkerArray display.
  marker_array_ = manager_->createDisplay( "rviz/MarkerArray", "MarkerArray", true );
  ROS_ASSERT( marker_array_ != NULL );
  
	marker_array_->subProp( "Marker Topic" )->setValue( "/occupied_cells_vis_array" );

  // Set topic that will be used as 0,0,0 -> reference for all the other transforms
  // IMPORTANT: WITHOUT THIS, ALL THE DIFFERENT PARTS OF THE ROBOT MODEL WILL BE DISPLAYED AT 0,0,0
  manager_->getFrameManager()->setFixedFrame("/pelvis");
}

// Destructor.
MarkerArray::~MarkerArray()
{
  delete manager_;
}

