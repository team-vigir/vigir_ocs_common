/* 
 * RawSensorDataInput class implementation.
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
#include "raw_sensor_data_input.h"

// Constructor for RawSensorDataInput.  This does most of the work of the class.
RawSensorDataInput::RawSensorDataInput( QWidget* parent )
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
	table->setColumnCount(5);

	table->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Expanding);

	// Add slot for handling cell changed
	connect( table, SIGNAL( cellChanged ( int, int ) ), this, SLOT( positionChanged( int, int ) ) );

	// Set Header Label Texts Here
	table->setHorizontalHeaderLabels(QString( "Name;Current Position;Velocity;Effort;Desired Position" ).split( ";" ));	

	// We first subscribe to the JointState messages
    joint_states_ = n_.subscribe<sensor_msgs::JointState>( "atlas/joint_states", 2, &RawSensorDataInput::updateTable, this );

	// Spin once to register subscribe	
	ros::spinOnce();
}

// Destructor.
RawSensorDataInput::~RawSensorDataInput()
{
  //delete manager_;
  delete table;
}

void RawSensorDataInput::positionChanged( int row, int column )
{
	if( column == 4 ) // desired position column
	{
        topic = "/atlas/pos_cmd/";
		topic += joint_list_[row].c_str();
        //topic += "_position_controller/command";
		
		std::cout << topic << std::endl;;
		
		// convert string in the table to double
		bool ok;
		double val = table->item( row, column )->text().toDouble( &ok );
		
		// only publish if conversion was ok
		if( ok )
		{
			std::cout << "publishing message " << val << std::endl;

			// Initialize publisher
			// since creating a publisher takes time (on separate threads)
			// we need to enable latching so that the message is not lost
			pos_pub_ = n_.advertise<std_msgs::Float64>( topic, 1, true );
			
			// Spin once to register advertise
			ros::spinOnce();
  
			// publish message
			cmd.data = val;
	
			pos_pub_.publish( cmd );
		}
	}
}

void RawSensorDataInput::updateTable( const sensor_msgs::JointState::ConstPtr& joint_states )
{
	joint_list_.clear();
	//std::cout << "updating table:" << std::endl;
	table->setRowCount( joint_states->name.size() );
	// Add Table items here
	for( int i = 0; i < joint_states->name.size(); i++ )
  {
		table->setItem( i, 0, new QTableWidgetItem( QString( joint_states->name[i].c_str() ) ) );
        table->setItem( i, 1, new QTableWidgetItem( QString::number( joint_states->position[i] ) ) );
		table->setItem( i, 2, new QTableWidgetItem( QString::number( joint_states->velocity[i] ) ) );
		table->setItem( i, 3, new QTableWidgetItem( QString::number( joint_states->effort[i] ) ) );
        //std::cout << "  joint name: " << joint_states->name[i] << std::endl;
		joint_list_.push_back( joint_states->name[i] );
	}
}


