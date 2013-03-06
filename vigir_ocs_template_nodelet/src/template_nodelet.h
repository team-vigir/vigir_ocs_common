#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <boost/thread.hpp>

#include <vector>
#include <string>

#include <flor_ocs_msgs/OCSTemplateAdd.h>
#include <flor_ocs_msgs/OCSTemplateList.h>
#include <flor_ocs_msgs/OCSTemplateUpdate.h>

#include <geometry_msgs/Pose.h>

namespace ocs_template
{
    class TemplateNodelet : public nodelet::Nodelet
    {
      public:
        virtual void onInit();

        void addTemplateCb(const flor_ocs_msgs::OCSTemplateAdd::ConstPtr& msg);
        void updateTemplateCb(const flor_ocs_msgs::OCSTemplateUpdate::ConstPtr& msg);
        void publishTemplateList();

      protected:
        ros::Subscriber template_update_sub_;
        ros::Subscriber template_add_sub_;
        ros::Publisher template_list_pub_;

        ros::Timer image_publish_timer_;

      private:
        std::vector<std::string> template_list_;
        std::vector<geometry_msgs::Pose> pose_list_;
    };
}
