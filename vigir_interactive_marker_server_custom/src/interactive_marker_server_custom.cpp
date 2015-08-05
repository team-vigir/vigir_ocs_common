/*
 * InteractiveMarkerServerCustom class implementation.
 *
 * Author: Felipe Bacim.
 *
 * Based on hector interactive marker server. Customized for use with flor_ocs classes.
 */

//=================================================================================================
// Copyright (c) 2011, Stefan Kohlbrecher, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

// Based on interactive_maker_tutorials basic_controls tutorial:
/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


#include <vigir_interactive_marker_server_custom/interactive_marker_server_custom.h>

InteractiveMarkerServerCustom::InteractiveMarkerServerCustom( const std::string &marker_name,
                                                              const std::string &topic_name,
                                                              const unsigned char& mode,
                                                              const std::string &frame_id,
                                                              const float &scale,
                                                              const geometry_msgs::Point &initial_pos )
{
    marker_name_ = marker_name;
    topic_name_ = topic_name;

    p_frame_id_ = frame_id;
    p_marker_name_ = topic_name+std::string("/pose_marker");

    marker_scale_ = scale;

    server = new interactive_markers::InteractiveMarkerServer(p_marker_name_,p_marker_name_+"/server",false);

    ros::Duration(0.2).sleep();

    marker_mode_ = mode;
    make6DofMarker( initial_pos );

    //ROS_ERROR("Creating marker: %s",marker_name.c_str());

   // marker_invisible_ = false;
    server->applyChanges();
}

InteractiveMarkerServerCustom::~InteractiveMarkerServerCustom()
{
    delete server;
}

void InteractiveMarkerServerCustom::processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
    std::ostringstream s;
    s << "Feedback from marker '" << feedback->marker_name << "' "
      << " / control '" << feedback->control_name << "'"
      << " / type '" << feedback->event_type << "'";
    std::string client_id = feedback->client_id[feedback->client_id.size()-1] == '/' ? feedback->client_id.substr(0,feedback->client_id.size()-1) : feedback->client_id;
    //ROS_ERROR("marker_feedback: %s",feedback->marker_name.c_str());

    //std::cout << (unsigned int)id_ << std::endl;

    std::ostringstream mouse_point_ss;
    if( feedback->mouse_point_valid )
    {
        mouse_point_ss << " at " << feedback->mouse_point.x
                       << ", " << feedback->mouse_point.y
                       << ", " << feedback->mouse_point.z
                       << " in frame " << feedback->header.frame_id;
    }
    //std::cout << mouse_point_ss << std::endl;

    switch ( feedback->event_type )
    {
    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
        geometry_msgs::PoseStamped pose;
        pose.pose = feedback->pose;
        pose.header = feedback->header;
        last_pose_ = pose;
        onFeedback(feedback->event_type,topic_name_,last_pose_,client_id);
        break;
    }

    server->applyChanges();
}

void InteractiveMarkerServerCustom::make6DofMarker( const geometry_msgs::Point &initial_pos )
{
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = p_frame_id_;
    int_marker.pose.position.x = initial_pos.x;
    int_marker.pose.position.y = initial_pos.y;
    int_marker.pose.position.z = initial_pos.z;
    int_marker.scale = marker_scale_;

    int_marker.name = topic_name_;
    //int_marker.description = marker_name_;

    // create interative marker controls
    createMarkerControls(marker_mode_, int_marker);

    last_pose_.pose = int_marker.pose;    
    server->insert(int_marker, boost::bind(&InteractiveMarkerServerCustom::processFeedback, this, _1));
    server->applyChanges();
}

std::string InteractiveMarkerServerCustom::getTopicName()
{
    return topic_name_;
}

std::string InteractiveMarkerServerCustom::getMarkerName()
{
    return marker_name_;
}

void InteractiveMarkerServerCustom::setPose( const geometry_msgs::PoseStamped pose )
{
    last_pose_ = pose;
    server->setPose(topic_name_, pose.pose, pose.header);
    server->applyChanges();
}

void InteractiveMarkerServerCustom::setMode(unsigned char mode )
{        
    marker_mode_ = mode;
    // get interactive marker instance
    visualization_msgs::InteractiveMarker int_marker;
    if(server->get(topic_name_,int_marker))
    {        
        server->erase(topic_name_);
        server->applyChanges();
        createMarkerControls(marker_mode_, int_marker);
        server->insert(int_marker, boost::bind(&InteractiveMarkerServerCustom::processFeedback, this, _1));        
        server->applyChanges();        
    }
    else
    {
        ROS_ERROR("FAILED TO FIND INTERACTIVE MARKER IN SERVER FOR TOPIC: %s",topic_name_.c_str());
    }

}

void InteractiveMarkerServerCustom::setVisible(bool new_marker_visibility)
{

//    if(new_marker_visibility)
//    {
//        //restore marker to be visible only if it is currently invisible
//        if(marker_invisible_)
//        {
//            server->erase("invisible_marker");
//            server->applyChanges();
//            server->insert(saved_int_marker_, boost::bind(&InteractiveMarkerServerCustom::processFeedback, this, _1));
//            server->applyChanges();
//            marker_invisible_ = false;
//        }
//    }
//    else
//    {
//        //dont need to set invisible if already invisble
//        if(!marker_invisible_)
//        {
//            //set marker to disappear
//            visualization_msgs::InteractiveMarker int_marker;
//            //get existing marker
//            if(server->get(topic_name_,int_marker))
//            {
//                //save marker to restore later
//                saved_int_marker_ = int_marker;
//                server->erase(topic_name_);
//                server->applyChanges();

//                //make invisible marker and push to server
//                createMarkerControls(INVISIBLE, int_marker);
//                server->insert(int_marker, boost::bind(&InteractiveMarkerServerCustom::processFeedback, this, _1));
//                server->applyChanges();
//                marker_invisible_ = true;
//            }
//            else
//            {
//                ROS_ERROR("FAILED TO FIND INTERACTIVE MARKER IN SERVER FOR TOPIC: %s",topic_name_.c_str());
//            }
//        }
//    }
}

void InteractiveMarkerServerCustom::createMarkerControls( const unsigned char &mode, visualization_msgs::InteractiveMarker &int_marker)
{
    int_marker.controls.clear();
    visualization_msgs::InteractiveMarkerControl control;

    switch(mode)
    {
        case INHERIT:

        case FIXED:
            // create interative marker controls
            control.orientation_mode = mode;
            // X
            control.orientation.w = 1;
            control.orientation.x = 1;
            control.orientation.y = 0;
            control.orientation.z = 0;
            control.name = "rotate_x";
            control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE;
            int_marker.controls.push_back(control);
            control.name = "move_x";
            control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
            int_marker.controls.push_back(control);
            // Z
            control.orientation.w = 1;
            control.orientation.x = 0;
            control.orientation.y = 1;
            control.orientation.z = 0;
            control.name = "rotate_z";
            control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE;
            int_marker.controls.push_back(control);
            control.name = "move_z";
            control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
            int_marker.controls.push_back(control);
            // Y
            control.orientation.w = 1;
            control.orientation.x = 0;
            control.orientation.y = 0;
            control.orientation.z = 1;
            control.name = "rotate_y";
            control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE;
            int_marker.controls.push_back(control);
            control.name = "move_y";
            control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
            int_marker.controls.push_back(control);
            break;

        case VIEW_FACING:
            // create interative marker controls
            control.orientation_mode = mode;
            control.orientation.w = 1;
            control.orientation.x = 1;
            control.orientation.y = 0;
            control.orientation.z = 0;
            control.name = "move_xy_rotate_z";
            control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE;
            int_marker.controls.push_back(control);
            control.name = "move_z";
            control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
            int_marker.controls.push_back(control);
            break;

        case INHERIT_ROTATION_FIXED://Same as INHERIT but rotation is fixed to an axis with object orientation
            // create interative marker controls
            control.orientation_mode = visualization_msgs::InteractiveMarkerControl::INHERIT;//use the oreintation of object
            // X
            control.orientation.w = 1;
            control.orientation.x = 1;
            control.orientation.y = 0;
            control.orientation.z = 0;
            control.name = "rotate_x";
            control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
            int_marker.controls.push_back(control);
            control.name = "move_x";
            control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
            int_marker.controls.push_back(control);
            // Z
            control.orientation.w = 1;
            control.orientation.x = 0;
            control.orientation.y = 1;
            control.orientation.z = 0;
            control.name = "rotate_z";
            control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
            int_marker.controls.push_back(control);
            control.name = "move_z";
            control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
            int_marker.controls.push_back(control);
            // Y
            control.orientation.w = 1;
            control.orientation.x = 0;
            control.orientation.y = 0;
            control.orientation.z = 1;
            control.name = "rotate_y";
            control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
            int_marker.controls.push_back(control);
            control.name = "move_y";
            control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
            int_marker.controls.push_back(control);
            break;

        case FIXED_ROTATION_WORLD://Same as FIXED but rotation is fixed to an axis with world orientation
            // create interative marker controls
            control.orientation_mode = visualization_msgs::InteractiveMarkerControl::FIXED;//Use the orientation of world
            // X
            control.orientation.w = 1;
            control.orientation.x = 1;
            control.orientation.y = 0;
            control.orientation.z = 0;
            control.name = "rotate_x";
            control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
            int_marker.controls.push_back(control);
            control.name = "move_x";
            control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
            int_marker.controls.push_back(control);
            // Z
            control.orientation.w = 1;
            control.orientation.x = 0;
            control.orientation.y = 1;
            control.orientation.z = 0;
            control.name = "rotate_z";
            control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
            int_marker.controls.push_back(control);
            control.name = "move_z";
            control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
            int_marker.controls.push_back(control);
            // Y
            control.orientation.w = 1;
            control.orientation.x = 0;
            control.orientation.y = 0;
            control.orientation.z = 1;
            control.name = "rotate_y";
            control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
            int_marker.controls.push_back(control);
            control.name = "move_y";
            control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
            int_marker.controls.push_back(control);
            break;

        case VIEW_FACING_ROTATION_FIXED://Same as VIEW_FACING but rotation is fixed to an axis
            // create interative marker controls
            control.orientation_mode = visualization_msgs::InteractiveMarkerControl::VIEW_FACING;//Use camera orientation
            control.orientation.w = 1;
            control.orientation.x = 1;
            control.orientation.y = 0;
            control.orientation.z = 0;
            control.name = "move_xy_rotate_z";
            control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
            int_marker.controls.push_back(control);
            control.name = "move_z";
            control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
            int_marker.controls.push_back(control);
            break;

        case WAYPOINT: // waypoints
            // create interative marker controls
            // Z
            {
            visualization_msgs::InteractiveMarkerControl control;
            control.orientation_mode = visualization_msgs::InteractiveMarkerControl::INHERIT;
            control.orientation.w = 1;
            control.orientation.x = 0;
            control.orientation.y = 1;
            control.orientation.z = 0;
            control.name = "rotate_z";
            control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
            int_marker.controls.push_back(control);
            control.name = "move_z";
            control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
            int_marker.controls.push_back(control);
            visualization_msgs::Marker marker;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.color.r = 0.6;
            marker.color.g = 0.6;
            marker.color.b = 0.6;
            marker.color.a = 0.3;
            marker.scale.x = 0.5;
            marker.scale.y = 0.5;
            marker.scale.z = 0.5;
            control.markers.push_back(marker);
            control.name = "move_xy";
            control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
            int_marker.controls.push_back(control);
            }
            break;

    case INVISIBLE: //Invisible Marker
        {
        //just make new uninitialized marker with scale 0
        visualization_msgs::InteractiveMarker invis;
        //invis.scale = 0;
        invis.name = "invisible_marker";
        int_marker = invis;
        }
        break;
        default:
            break;
    }
}
