/* 
 * RobotModel class implementation.
 * 
 * Author: Felipe Bacim.
 * 
 * Based on librviz_tutorials.
 * 
 * Latest changes (12/03/2012):
 * - fixed transforms by using the FrameManager setFixedFrame function, may need to add UI for this
 */

#include <QLabel>
#include <QVBoxLayout>

#include "rviz/visualization_manager.h"
#include "rviz/render_panel.h"
#include "rviz/display.h"
#include "rviz/frame_manager.h"
#include "robot_model.h"

// Constructor for RobotModel.  This does most of the work of the class.
RobotModel::RobotModel( QWidget* parent )
  : QWidget( parent )
{
  // Create a new label for this widget.
  QLabel* robot_model_label = new QLabel( "rviz/RobotModel" );
  
  // Construct and lay out render panel.
  render_panel_ = new rviz::RenderPanel();
  QVBoxLayout* main_layout = new QVBoxLayout;
  main_layout->addWidget( robot_model_label );
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
  manager_->getFrameManager()->setFixedFrame("/pelvis");
}

// Destructor.
RobotModel::~RobotModel()
{
  delete manager_;
}

