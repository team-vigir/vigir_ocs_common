#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <boost/thread.hpp>

#include <vector>
#include <string>

#include <geometry_msgs/PoseStamped.h>

#include <geometry_msgs/Vector3.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>

#include <vigir_footstep_planning_msgs/StepPlan.h>
#include <flor_ocs_msgs/OCSFootstepList.h>
#include <flor_ocs_msgs/OCSFootstepUpdate.h>

namespace ocs_footstep
{
    class FootstepManager : public nodelet::Nodelet
    {
    public:
        virtual void onInit();

        void processFootstepArray(const visualization_msgs::MarkerArray::ConstPtr& msg);
        void processFootstepBodyBBArray(const visualization_msgs::MarkerArray::ConstPtr& msg);
        void processFootstepPathArray(const nav_msgs::Path::ConstPtr& msg);

        void processFootstepPoseUpdate(const flor_ocs_msgs::OCSFootstepUpdate::ConstPtr& msg);

        void timerCallback(const ros::TimerEvent& event);

    private:
        void publishFootstepVis();
        void publishFootstepList();

        void stepPlanToFootMarkerArray(vigir_footstep_planning_msgs::StepPlan& input, visualization_msgs::MarkerArray& foot_array_msg);
        void stepPlanToBodyMarkerArray(vigir_footstep_planning_msgs::StepPlan& input, visualization_msgs::MarkerArray& body_array_msg);
        void stepPlanToFootPath(vigir_footstep_planning_msgs::StepPlan& input, nav_msgs::Path& foot_path_msg);
        void stepToMarker(const vigir_footstep_planning_msgs::Step &step, visualization_msgs::Marker &marker);

        ros::Timer timer;

        ros::Publisher footstep_array_pub_;
        ros::Publisher footstep_body_bb_array_pub_;
        ros::Publisher footstep_path_pub_;
        ros::Subscriber footstep_array_sub_;
        ros::Subscriber footstep_body_bb_array_sub_;
        ros::Subscriber footstep_path_sub_;

        ros::Publisher footstep_list_pub_;
        ros::Subscriber footstep_update_sub_;

        vigir_footstep_planning_msgs::StepPlan footstep_plan_;
        visualization_msgs::MarkerArray footstep_array_;
        visualization_msgs::MarkerArray footstep_body_array_;
        nav_msgs::Path footstep_path_;

        // Parameters
        geometry_msgs::Vector3 foot_size;
        geometry_msgs::Vector3 foot_origin_shift;
        double foot_separation;

        geometry_msgs::Vector3 upper_body_size;
        geometry_msgs::Vector3 upper_body_origin_shift;
    };
}
