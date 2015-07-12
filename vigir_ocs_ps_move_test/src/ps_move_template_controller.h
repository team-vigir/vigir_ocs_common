
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
