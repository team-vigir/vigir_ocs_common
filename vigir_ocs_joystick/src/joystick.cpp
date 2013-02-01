/* 
 * Main3DView class implementation.
 * 
 * Author: Felipe Bacim.
 * 
 * Based on librviz_tutorials and pr2_teleop.
 * 
 * Latest changes (12/08/2012):
 *
 */

#include <QLabel>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>

#include <ros/ros.h>

#include "joystick.h"

// Constructor for Joystick.  This does most of the work of the class.
Joystick::Joystick( QWidget* parent )
 : QWidget( parent )
{
  // Create a new label for this widget.
  QLabel* joystick_label = new QLabel( "Joystick" );
  
  // Construct and lay out render panel.
  QPushButton* turn_left = new QPushButton( "<" );
  QPushButton* turn_right = new QPushButton( ">" );
  QPushButton* forward = new QPushButton( "^" );
  QPushButton* backward = new QPushButton( "v" );
  // Use nested layouts to create the buttons
  QVBoxLayout* mid_layout = new QVBoxLayout;
  QHBoxLayout* main_layout = new QHBoxLayout;
  mid_layout->addWidget( forward );
  mid_layout->addWidget( backward );
  QWidget* mid = new QWidget;
  mid->setLayout( mid_layout );
  main_layout->addWidget( turn_left );
  main_layout->addWidget( mid );
  main_layout->addWidget( turn_right );

  // Set the top-level layout for this MyViz widget.
  setLayout( main_layout );		

  // Make signal/slot connections.
  connect( turn_left, SIGNAL( pressed() ), this, SLOT( turnLeftPressed() ) );
  connect( turn_left, SIGNAL( released() ), this, SLOT( turnLeftReleased() ) );
  connect( turn_right, SIGNAL( pressed() ), this, SLOT( turnRightPressed() ) );
  connect( turn_right, SIGNAL( released() ), this, SLOT( turnRightReleased() ) );
  connect( forward, SIGNAL( pressed() ), this, SLOT( forwardPressed() ) );
  connect( forward, SIGNAL( released() ), this, SLOT( forwardReleased() ) );
  connect( backward, SIGNAL( pressed() ), this, SLOT( backwardPressed() ) );
  connect( backward, SIGNAL( released() ), this, SLOT( backwardReleased() ) );

	// Initialize publisher  
  vel_pub_ = n_.advertise<geometry_msgs::Twist>( "atlas/cmd_vel", 1 );

	// Initialize velocity variables
  ros::NodeHandle n_private("~");
  n_private.param( "walk_vel", walk_vel, 0.5 );
  n_private.param( "run_vel", run_vel, 1.0 );
  n_private.param( "yaw_rate", yaw_rate, 1.0 );
  n_private.param( "yaw_run_rate", yaw_rate_run, 1.5 );
}

// Destructor.
Joystick::~Joystick()
{
}

// SLOTS implementation
void Joystick::turnLeftPressed()
{
	cmd.angular.z = yaw_rate;
	vel_pub_.publish( cmd );
}

void Joystick::turnLeftReleased()
{
	cmd.angular.z = 0.0;
	vel_pub_.publish( cmd );
}

void Joystick::turnRightPressed()
{
	cmd.angular.z = -yaw_rate;
	vel_pub_.publish( cmd );
}

void Joystick::turnRightReleased()
{
	cmd.angular.z = 0.0;
	vel_pub_.publish( cmd );
}

void Joystick::forwardPressed()
{
	cmd.linear.x = run_vel;
	vel_pub_.publish( cmd );
}

void Joystick::forwardReleased()
{
	cmd.linear.x = 0.0;
	vel_pub_.publish( cmd );
}

void Joystick::backwardPressed()
{
	cmd.linear.x = -run_vel;
	vel_pub_.publish( cmd );
}

void Joystick::backwardReleased()
{
	cmd.linear.x = 0.0;
	vel_pub_.publish( cmd );
}

