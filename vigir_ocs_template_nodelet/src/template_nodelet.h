#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <boost/thread.hpp>

#include <vector>
#include <string>

#include <vigir_ocs_msg2/OCSTemplateAdd.h>
#include <vigir_ocs_msg2/OCSTemplateList.h>
#include <vigir_ocs_msg2/OCSTemplateUpdate.h>

#include <geometry_msgs/Pose.h>

namespace ocs_template
{
    class TemplateNodelet : public nodelet::Nodelet
    {
      public:
        virtual void onInit();

        void addTemplateCb(const vigir_ocs_msg2::OCSTemplateAdd::ConstPtr& msg);
        void updateTemplateCb(const vigir_ocs_msg2::OCSTemplateUpdate::ConstPtr& msg);
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
