/*
 * Main3DView class implementation.
 * 
 * Author: Felipe Bacim.
 * 
 * Based on librviz_tutorials and pr2_teleop.
 * 
 * Latest changes (12/08/2012):
 *   -(03/28/2013)
 *   -(04/11/2013) changed to support for new joystick GUI and message
 *
 */

#include <QLabel>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>

#include <ros/ros.h>

#include "flor_ocs_msgs/OCSDrive.h"

#include "joystick.h"

// Constructor for Joystick.  This does most of the work of the class.
Joystick::Joystick( QWidget* parent )
 : QWidget( parent )
{
    //Initialize globals
    robot_steer = 0;
    robot_throttle = 0;

    // Initialize publisher
    drive_pub_ = n_.advertise<flor_ocs_msgs::OCSDrive>( "drive_cmd", 1, false );

    //Initialize subscriber
    sub_ = n_.subscribe<flor_ocs_msgs::OCSDrive>( "drive_status", 1, &Joystick::callback, this );
}

// Destructor.
Joystick::~Joystick()
{
}

void Joystick::callback(const flor_ocs_msgs::OCSDrive::ConstPtr& msg)
{
    robot_steer = msg->steer/*.data*/;
    robot_throttle = msg->throttle/*.data*/;
}

// SLOTS implementation
void Joystick::setRobotThrottle( unsigned char throttle )
{
    robot_throttle = throttle;
}

unsigned char Joystick::getRobotThrottle()
{
    return robot_throttle;
}

void Joystick::setRobotSteer( signed char steer )
{
    robot_steer = steer;
}

signed char Joystick::getRobotSteer()
{
    return robot_steer;
}

void Joystick::publish()
{
    drive_cmd.steer = robot_steer;
    drive_cmd.throttle = robot_throttle;
    drive_pub_.publish( drive_cmd );
}
