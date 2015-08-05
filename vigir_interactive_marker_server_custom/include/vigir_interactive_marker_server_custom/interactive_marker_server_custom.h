/*
 * InteractiveMarkerServerCustom class header.
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

#ifndef INTERACTIVE_MARKERS_CUSTOM_H__
#define INTERACTIVE_MARKERS_CUSTOM_H__

#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>

#include <vigir_ocs_msgs/OCSTemplateUpdate.h>
#include <vigir_ocs_msgs/OCSInteractiveMarkerAdd.h>

#include <math.h>

#include <boost/bind.hpp>

enum MarkerMode { INHERIT = visualization_msgs::InteractiveMarkerControl::INHERIT,
                  FIXED = visualization_msgs::InteractiveMarkerControl::FIXED,
                  VIEW_FACING =visualization_msgs::InteractiveMarkerControl::VIEW_FACING,
                  INHERIT_ROTATION_FIXED,
                  FIXED_ROTATION_WORLD,
                  VIEW_FACING_ROTATION_FIXED,
                  WAYPOINT = vigir_ocs_msgs::OCSInteractiveMarkerAdd::WAYPOINT_3DOF,
                  INVISIBLE
                };

class InteractiveMarkerServerCustom
{
public:
    InteractiveMarkerServerCustom( const std::string &marker_name, const std::string &topic_name, const unsigned char& mode, const std::string &frame_id, const float &scale, const geometry_msgs::Point &initial_pos );
    ~InteractiveMarkerServerCustom();

    void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
    void make6DofMarker( const geometry_msgs::Point &initial_pos );
    void setPose(const geometry_msgs::PoseStamped pose );
    geometry_msgs::PoseStamped getPose() { return last_pose_; }
    void setMode( unsigned char mode );
    unsigned char getMode() { return marker_mode_; }

    std::string getTopicName();
    std::string getMarkerName();
    void setVisible(bool visible);

    boost::function<void(unsigned char, std::string, geometry_msgs::PoseStamped, std::string)> onFeedback;

private:
    void createMarkerControls( const unsigned char &mode, visualization_msgs::InteractiveMarker &int_marker );

    interactive_markers::InteractiveMarkerServer *server;

    visualization_msgs::InteractiveMarker saved_int_marker_;
    bool marker_invisible_;

    std::string p_frame_id_;
    std::string p_marker_name_;

    std::string topic_name_;
    std::string marker_name_;    

    geometry_msgs::PoseStamped last_pose_;

    float marker_scale_;

    unsigned char marker_mode_;

};

#endif
