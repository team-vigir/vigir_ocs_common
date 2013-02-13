/* 
 * RawSensorData class implementation.
 * 
 * Author: Felipe Bacim.
 * 
 * Based on librviz_tutorials.
 * 
 */

#include <QLabel>
#include <QVBoxLayout>
#include <QString>

#include <ros/ros.h>

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
  //main_layout->addWidget( render_panel_ );
  main_layout->addWidget( table );

  // Set the top-level layout for this MyViz widget.
  setLayout( main_layout );		

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
  manager_->getFrameManager()->setFixedFrame( "/pelvis" );
	
	// Set table row count 1 and column count 3
	table->setRowCount(100);
	table->setColumnCount(4);

	table->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Expanding);

	// Set Header Label Texts Here
	table->setHorizontalHeaderLabels(QString( "Name;Position;Velocity;Effort" ).split( ";" ));	

	// We first subscribe to the JointState messages
	ros::NodeHandle nh;
    joint_states_ = nh.subscribe<sensor_msgs::JointState>( "/atlas/joint_states", 2, &RawSensorData::updateTable, this );
	
	ros::spinOnce();
}

// Destructor.
RawSensorData::~RawSensorData()
{
  //delete manager_;
  delete table;
}

void RawSensorData::updateTable( const sensor_msgs::JointState::ConstPtr& joint_states )
{
    //std::cout << "updating table:" << std::endl;
	table->setRowCount(joint_states->name.size());
	// Add Table items here
	for( int i = 0; i < joint_states->name.size(); i++ )
  {
		table->setItem(i,0,new QTableWidgetItem(QString(joint_states->name[i].c_str())));
		table->setItem(i,1,new QTableWidgetItem(QString::number(joint_states->position[i])));
		table->setItem(i,2,new QTableWidgetItem(QString::number(joint_states->velocity[i])));
		table->setItem(i,3,new QTableWidgetItem(QString::number(joint_states->effort[i])));
        //std::cout << "  joint name: " << joint_states->name[i] << std::endl;
	}
}

