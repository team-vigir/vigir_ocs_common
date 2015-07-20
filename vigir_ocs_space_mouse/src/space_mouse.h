/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013-2015, Team ViGIR ( TORC Robotics LLC, TU Darmstadt, Virginia Tech, Oregon State University, Cornell University, and Leibniz University Hanover )
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
//@TODO_ADD_AUTHOR_INFO

#ifndef SPACE_MOUSE_H
#define SPACE_MOUSE_H

#include <QApplication>
#include <QVector3D>
#include <QQuaternion>
#include <ros/ros.h>

#include <sensor_msgs/Joy.h>

#include <vigir_ocs_msgs/OCSTemplateList.h>
#include <vigir_ocs_msgs/OCSObjectSelection.h>
#include <vigir_ocs_msgs/OCSTemplateUpdate.h>
#include <vigir_ocs_msgs/OCSCameraTransform.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>

#include<math.h>

#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

namespace vigir_ocs
{

    class SpaceMouse// : public QObject
    {
        //Q_OBJECT
      public:

        SpaceMouse();
        ~SpaceMouse();

        struct Quaternion
        {
            double x, y, z, w;
        };

        struct Vector
        {
            double x, y, z;
        };

        //void processKeyEvent(const vigir_ocs_msgs::OCSKeyEvent::ConstPtr& msg);

        void processTemplateList(const vigir_ocs_msgs::OCSTemplateList::ConstPtr &list);
        void processObjectSelection(const vigir_ocs_msgs::OCSObjectSelection::ConstPtr &obj);
        void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
        void cameraCb(const vigir_ocs_msgs::OCSCameraTransform::ConstPtr& msg);
        geometry_msgs::PoseStamped updatePose(geometry_msgs::PoseStamped p, const sensor_msgs::Joy::ConstPtr& joy);



        //Vector convertToEuler(QQuaternion q1);
        //QQuaternion convertToQuaternion(double heading, double attitude, double bank);

      private:
        ros::NodeHandle nh_;

        ros::Subscriber template_list_sub_;

        ros::Subscriber select_object_sub_;

        ros::Subscriber joy_sub_;

        ros::Publisher template_update_pub_;

        std::vector<unsigned char> template_id_list;

        geometry_msgs::PoseStamped pose_;

        geometry_msgs::PoseStamped camera_pose;

        ros::Subscriber camera_sub_;
        ros::Publisher camera_pub_;

        bool recieved_pose;

        int id;

        QVector3D cameraPosition;
        QQuaternion cameraOrientation;
        vigir_ocs_msgs::OCSCameraTransform cameraUpdate;
        vigir_ocs_msgs::OCSCameraTransform update;

    };
}
#endif
