#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <boost/thread.hpp>

#include <vector>
#include <string>

#include <tf/transform_broadcaster.h>

#include <flor_ocs_msgs/OCSImageAdd.h>
#include <flor_ocs_msgs/OCSImageRemove.h>
#include <flor_ocs_msgs/OCSImageList.h>
#include <flor_ocs_msgs/OCSImageUpdate.h>
#include <flor_grasp_msgs/GraspSelection.h>
#include <flor_grasp_msgs/GraspState.h>
#include <flor_grasp_msgs/ImageSelection.h>

#include <geometry_msgs/PoseStamped.h>

namespace ocs_image
{
    class ImageNodelet : public nodelet::Nodelet
    {
      public:
        virtual void onInit();

        void addImageCb(const flor_ocs_msgs::OCSImageAdd::ConstPtr& msg);
        void removeImageCb(const flor_ocs_msgs::OCSImageRemove::ConstPtr& msg);
        void updateImageCb(const flor_ocs_msgs::OCSImageUpdate::ConstPtr& msg);
        void graspRequestCb(const flor_grasp_msgs::GraspSelection::ConstPtr& msg);
        void graspStateFeedbackCb(const flor_grasp_msgs::GraspState::ConstPtr& msg);
        void imageMatchRequestCb(const flor_grasp_msgs::ImageSelection::ConstPtr& msg);
        void imageMatchFeedbackCb(const flor_grasp_msgs::ImageSelection::ConstPtr& msg);
        void publishImageList();
        void timerCallback(const ros::TimerEvent& event);

      protected:
        ros::Subscriber image_update_sub_;
        ros::Subscriber image_add_sub_;
        ros::Subscriber image_remove_sub_;
        ros::Subscriber grasp_request_sub_;
        ros::Subscriber grasp_state_feedback_sub_;
        ros::Subscriber image_match_request_sub_;
        ros::Subscriber image_match_feedback_sub_;
        ros::Publisher image_list_pub_;
        ros::Publisher image_selected_pub_;
        ros::Publisher grasp_selected_pub_;
        ros::Publisher grasp_selected_state_pub_;

        ros::Timer image_publish_timer_;

      private:
        std::vector<unsigned char> image_id_list_;
        std::vector<std::string> image_list_;
        std::vector<geometry_msgs::PoseStamped> pose_list_;
        unsigned char id_counter_;

        ros::Timer timer;

        tf::TransformBroadcaster tfb_;
    };
}
