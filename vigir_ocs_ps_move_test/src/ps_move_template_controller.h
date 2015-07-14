/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013-2015, TORC Robotics, LLC ( Team ViGIR )
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Team ViGIR, TORC Robotics, nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
@TODO_ADD_AUTHOR_INFO

#ifndef PS_MOVE_CONTROLLER_H
#define PS_MOVE_CONTROLLER_H

#include <QApplication>
#include <QVector3D>
#include <QQuaternion>
#include <ros/ros.h>

#include <sensor_msgs/Joy.h>

#include <vigir_ocs_msgs/OCSInteractiveMarkerUpdate.h>
#include <vigir_ocs_msgs/OCSCameraTransform.h>
#include <vigir_ocs_msgs/OCSObjectSelection.h>
#include <vigir_ocs_msgs/OCSSelectedObjectUpdate.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>

#include<math.h>

#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "moveclient.h"
#include <boost/bind.hpp>

namespace vigir_ocs
{

    class PSMoveTemplateController// : public QObject
    {
        //Q_OBJECT
      public:

        PSMoveTemplateController();
        ~PSMoveTemplateController();

        void processCameraTransform(const vigir_ocs_msgs::OCSCameraTransform::ConstPtr& msg);
        void processSelectedObjectUpdate(const vigir_ocs_msgs::OCSSelectedObjectUpdate::ConstPtr& msg);
        geometry_msgs::PoseStamped updatePose(geometry_msgs::PoseStamped p, MoveServerPacket *move_server_packet, float normalized_scale);

        int updateSuccess(MoveServerPacket *move_server_packet);
        int updateFailure(int error);
        int updateCameraSuccess(MoveServerCameraFrameSlicePacket *move_server_camera_frame_slice_packet);
        int updateCameraFailure(int error);

      private:
        ros::NodeHandle nh_;

        ros::Subscriber joy_sub_;
        ros::Publisher interactive_marker_update_pub_;
        ros::Subscriber camera_sub_;
        ros::Subscriber selected_object_update_sub_;

        std::string selected_object_topic_;

        geometry_msgs::PoseStamped object_pose_;
        geometry_msgs::PoseStamped camera_pose_;

        vigir_ocs_msgs::OCSCameraTransform camera_update_;

        bool received_pose_;

        QVector3D camera_position_;
        QQuaternion camera_orientation_;

        QVector3D old_move_position_;
        QQuaternion old_move_orientation_;

        QQuaternion last_object_orientation_ ;
        QQuaternion camera_T_object;
        QQuaternion last_move_orientation_;
        bool first_time_;
        int degrees_;
        unsigned short last_analog_trigger_;
    };
}
#endif
