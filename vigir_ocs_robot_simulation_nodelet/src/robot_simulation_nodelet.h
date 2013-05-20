#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <boost/thread.hpp>

#include <vector>
#include <string>

#include <tf/tfMessage.h>

namespace ocs_robot_simulation
{
    class RobotSimulationNodelet : public nodelet::Nodelet
    {
      public:
        virtual void onInit();

        void updateTFCb(const tf::tfMessage::ConstPtr& msg);
        void publishRobotSimulationTF(const tf::tfMessage::ConstPtr& msg);

      protected:
        ros::Subscriber robot_tf_sub_;
        ros::Publisher robot_simulation_tf_pub_;
    };
}
