/* 
 * RawSensorData class implementation.
 * 
 * Author: Felipe Bacim.
 * 
 * Based on librviz_tutorials.
 * 
 * Latest changes (12/17/2012):
 * - added QTableWidget
 */

#include <QLabel>
#include <QVBoxLayout>

#include "rviz/visualization_manager.h"
#include "rviz/render_panel.h"
#include "rviz/display.h"
#include "rviz/frame_manager.h"
#include "raw_sensor_data.h"

// Constructor for RawSensorData.  This does most of the work of the class.
RawSensorData::RawSensorData( QWidget* parent )
  : QWidget( parent )
{
  // Create a new label for this widget.
  QLabel* robot_model_label = new QLabel( "rviz/RobotModel links" );
  
  // Construct and lay out render panel.
  render_panel_ = new rviz::RenderPanel();
  table = new QTableWidget();
  QVBoxLayout* main_layout = new QVBoxLayout;
  main_layout->addWidget( robot_model_label );
  main_layout->addWidget( render_panel_ );
  main_layout->addWidget( table );

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
	
	// Set table row count 1 and column count 3
	table->setRowCount(1);
	table->setColumnCount(3);

	table->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Expanding);

	// Set Header Label Texts Here
	table->setHorizontalHeaderLabels(QString("Link;Position;Orientation").split(";"));	

	// Need to initialize update thread
}

// Destructor.
RawSensorData::~RawSensorData()
{
  // NEED TO STOP THREAD
  delete manager_;
  delete table;
}

void RawSensorData::updateTable()
{
	// Need to retrieve names of links from RobotModelDisplay or somewhere else... maybe one of the managers?
	//   RobotModelDisplay contains an object of Robot (private, no set/get), which is used for rendering (line 99: robot_model_display.cpp)
	//   That object does have all the links, but I don't have access to it
	
	// Add Table items here
	//for( int i = 0; i < links.size(); i++ )
	//{
	//table->setItem(i,0,new QTableWidgetItem("Link name"));
	//table->setItem(i,1,new QTableWidgetItem("Position"));
	//table->setItem(i,2,new QTableWidgetItem("Orientation"));
	//}
}

