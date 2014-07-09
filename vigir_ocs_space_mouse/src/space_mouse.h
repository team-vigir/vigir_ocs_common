#include <ros/ros.h>

#include <sensor_msgs/Joy.h>

#include <flor_ocs_msgs/OCSTemplateList.h>
#include <flor_ocs_msgs/OCSObjectSelection.h>
#include<flor_ocs_msgs/OCSTemplateUpdate.h>
#include<geometry_msgs/Pose.h>
#include<geometry_msgs/Point.h>

#include<math.h>

#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#pragma once
namespace vigir_ocs
{

    class SpaceMouse
    {
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

        //void processKeyEvent(const flor_ocs_msgs::OCSKeyEvent::ConstPtr& msg);

        void processTemplateList(const flor_ocs_msgs::OCSTemplateList::ConstPtr &list);
        void processObjectSelection(const flor_ocs_msgs::OCSObjectSelection::ConstPtr &obj);
        void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);





        Vector convertToEuler(Quaternion q1);
        Quaternion convertToQuaternion(double heading, double attitude, double bank);

      private:
        ros::NodeHandle nh_;

        ros::Subscriber template_list_sub_;

        ros::Subscriber select_object_sub_;

        ros::Subscriber joy_sub_;

        ros::Publisher template_update_pub_;

        std::vector<unsigned char> template_id_list;

        geometry_msgs::PoseStamped pose;

        bool recieved_pose;

        unsigned char id;

    };
}
