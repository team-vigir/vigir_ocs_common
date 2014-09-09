
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

        std::vector<unsigned char> template_id_list_;

        geometry_msgs::PoseStamped template_pose_;

        geometry_msgs::PoseStamped camera_pose_;

        ros::Subscriber camera_sub_;

        bool received_pose_;

        int template_selected_id_;

        QVector3D camera_position_;
        QQuaternion camera_orientation_;
        flor_ocs_msgs::OCSCameraTransform camera_update_;

        QVector3D old_move_position_;
        QQuaternion old_move_orientation_;
    };
}
#endif
