#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <boost/thread.hpp>

#include <vector>
#include <string>

#include <flor_ocs_msgs/OCSTemplateAdd.h>
#include <flor_ocs_msgs/OCSTemplateRemove.h>
#include <flor_ocs_msgs/OCSTemplateList.h>
#include <flor_ocs_msgs/OCSTemplateUpdate.h>
#include <flor_grasp_msgs/GraspSelection.h>
#include <flor_grasp_msgs/GraspState.h>
#include <flor_grasp_msgs/TemplateSelection.h>

#include <geometry_msgs/PoseStamped.h>

namespace ocs_template
{
    class TemplateNodelet : public nodelet::Nodelet
    {
      public:
        virtual void onInit();

        void addTemplateCb(const flor_ocs_msgs::OCSTemplateAdd::ConstPtr& msg);
        void removeTemplateCb(const flor_ocs_msgs::OCSTemplateRemove::ConstPtr& msg);
        void updateTemplateCb(const flor_ocs_msgs::OCSTemplateUpdate::ConstPtr& msg);
        void graspRequestCb(const flor_grasp_msgs::GraspSelection::ConstPtr& msg);
        void graspStateFeedbackCb(const flor_grasp_msgs::GraspState::ConstPtr& msg);
        void templateMatchRequestCb(const flor_grasp_msgs::TemplateSelection::ConstPtr& msg);
        void templateMatchFeedbackCb(const flor_grasp_msgs::TemplateSelection::ConstPtr& msg);
        void publishTemplateList();

      protected:
        ros::Subscriber template_update_sub_;
        ros::Subscriber template_add_sub_;
        ros::Subscriber template_remove_sub_;
        ros::Subscriber grasp_request_sub_;
        ros::Subscriber grasp_state_feedback_sub_;
        ros::Subscriber template_match_request_sub_;
        ros::Subscriber template_match_feedback_sub_;
        ros::Publisher template_list_pub_;
        ros::Publisher template_selected_pub_;
        ros::Publisher grasp_selected_pub_;
        ros::Publisher grasp_selected_state_pub_;

        ros::Timer image_publish_timer_;

      private:
        std::vector<unsigned char> template_id_list_;
        std::vector<std::string> template_list_;
        std::vector<geometry_msgs::PoseStamped> pose_list_;
        unsigned char id_counter_;
    };
}
