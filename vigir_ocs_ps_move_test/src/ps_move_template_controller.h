
#ifndef PS_MOVE_CONTROLLER_H
#define PS_MOVE_CONTROLLER_H

#include <QApplication>
#include <QVector3D>
#include <QQuaternion>
#include <ros/ros.h>

#include <sensor_msgs/Joy.h>

#include <flor_ocs_msgs/OCSTemplateList.h>
#include <flor_ocs_msgs/OCSObjectSelection.h>
#include <flor_ocs_msgs/OCSTemplateUpdate.h>
#include <flor_ocs_msgs/OCSCameraTransform.h>
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

        struct Quaternion
        {
            double x, y, z, w;
        };

        struct Vector
        {
            double x, y, z;
        };

        void processTemplateList(const flor_ocs_msgs::OCSTemplateList::ConstPtr &list);
        void processObjectSelection(const flor_ocs_msgs::OCSObjectSelection::ConstPtr &obj);
        void cameraCb(const flor_ocs_msgs::OCSCameraTransform::ConstPtr& msg);
        geometry_msgs::PoseStamped updatePose(geometry_msgs::PoseStamped p, MoveServerPacket *move_server_packet, float normalized_scale);

        int updateSuccess(MoveServerPacket *move_server_packet);
        int updateFailure(int error);
        int updateCameraSuccess(MoveServerCameraFrameSlicePacket *move_server_camera_frame_slice_packet);
        int updateCameraFailure(int error);

      private:
        ros::NodeHandle nh_;

        ros::Subscriber template_list_sub_;

        ros::Subscriber select_object_sub_;

        ros::Subscriber joy_sub_;

        ros::Publisher template_update_pub_;

        std::vector<unsigned char> template_id_list;

        geometry_msgs::PoseStamped pose;

        geometry_msgs::PoseStamped camera_pose;

        ros::Subscriber camera_sub;
        ros::Publisher camera_pub;

        bool received_pose;

        int id;

        QVector3D cameraPosition;
        QQuaternion cameraOrientation;
        flor_ocs_msgs::OCSCameraTransform cameraUpdate;
        flor_ocs_msgs::OCSCameraTransform update;

        QVector3D oldMovePosition;
        QQuaternion oldMoveOrientation;
    };
}
#endif
