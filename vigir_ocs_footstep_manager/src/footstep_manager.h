#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <boost/thread.hpp>

#include <vector>
#include <stack>
#include <string>

#include <geometry_msgs/PoseStamped.h>

#include <geometry_msgs/Vector3.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>

#include <vigir_footstep_planning_msgs/StepPlan.h>
#include <flor_ocs_msgs/OCSFootstepList.h>
#include <flor_ocs_msgs/OCSFootstepUpdate.h>
#include <flor_state_msgs/LowerBodyState.h>

#include <vigir_footstep_planning_msgs/StepPlanRequestAction.h>
#include <vigir_footstep_planning_msgs/EditStepAction.h>
#include <actionlib/client/simple_action_client.h>

namespace ocs_footstep
{
    typedef actionlib::SimpleActionClient<vigir_footstep_planning_msgs::StepPlanRequestAction> StepPlanRequestClient;
    typedef actionlib::SimpleActionClient<vigir_footstep_planning_msgs::EditStepAction> EditStepClient;

    class FootstepManager : public nodelet::Nodelet
    {
    public:
        virtual void onInit();

        // placeholders for ros action to request plan
        void processFootstepArray(const visualization_msgs::MarkerArray::ConstPtr& msg);
        void processFootstepBodyBBArray(const visualization_msgs::MarkerArray::ConstPtr& msg);
        void processFootstepPathArray(const nav_msgs::Path::ConstPtr& msg);

        // feedback look for interaction, should update stepplan and use actions to edit/update
        void processFootstepPoseUpdate(const flor_ocs_msgs::OCSFootstepUpdate::ConstPtr& msg);

        // get the current and goal poses to be used when requesting a footstep plan
        void processLowerBodyState(const flor_state_msgs::LowerBodyStateConstPtr &lower_body_state);
        void processFootstepGoalPose(const geometry_msgs::PoseStampedConstPtr &goal_pose); // triggers footstep plan

        void timerCallback(const ros::TimerEvent& event);

    private:
        void publishFootstepVis();
        void publishFootstepList();

        // plan requests
        void requestStepPlan();
        void requestStepPlanFromStep(vigir_footstep_planning_msgs::Step &step);
        void requestStepPlan(vigir_footstep_planning_msgs::Feet &start, vigir_footstep_planning_msgs::Feet &goal);
        void updateVisualizationMsgs();

        // auxiliary functions that create visualizations based on step plans
        void stepPlanToFootMarkerArray(vigir_footstep_planning_msgs::StepPlan& input, visualization_msgs::MarkerArray& foot_array_msg);
        void stepPlanToBodyMarkerArray(vigir_footstep_planning_msgs::StepPlan& input, visualization_msgs::MarkerArray& body_array_msg);
        void stepPlanToFootPath(vigir_footstep_planning_msgs::StepPlan& input, nav_msgs::Path& foot_path_msg);
        void stepToMarker(const vigir_footstep_planning_msgs::Step &step, visualization_msgs::Marker &marker);

        // functions that control the undo/redo stacks
        void addNewPlanList();
        void undo();
        void redo();

        // helper functions that return current step plan list and step plan, to reduce clutter
        inline std::vector<vigir_footstep_planning_msgs::StepPlan>& getStepPlanList() { return footstep_plans_undo_stack_.top(); }
        inline vigir_footstep_planning_msgs::StepPlan& getStepPlan() { return getStepPlanList().back(); }

        ros::Timer timer;

        ros::Publisher footstep_array_pub_;
        ros::Publisher footstep_body_bb_array_pub_;
        ros::Publisher footstep_path_pub_;
        ros::Subscriber footstep_array_sub_;
        ros::Subscriber footstep_body_bb_array_sub_;
        ros::Subscriber footstep_path_sub_;

        ros::Publisher footstep_list_pub_;
        ros::Subscriber footstep_update_sub_;

        ros::Subscriber footstep_plan_request_sub_;
        ros::Subscriber lower_body_state_sub_;

        // footstep plan request
        ros::Subscriber set_goal_sub_;

        std::stack< std::vector<vigir_footstep_planning_msgs::StepPlan> > footstep_plans_undo_stack_;
        std::stack< std::vector<vigir_footstep_planning_msgs::StepPlan> > footstep_plans_redo_stack_;
        visualization_msgs::MarkerArray footstep_array_;
        visualization_msgs::MarkerArray footstep_body_array_;
        nav_msgs::Path footstep_path_;

        // Parameters
        geometry_msgs::Vector3 foot_size;
        geometry_msgs::Vector3 foot_origin_shift;
        double foot_separation;

        geometry_msgs::Vector3 upper_body_size;
        geometry_msgs::Vector3 upper_body_origin_shift;

        // used to calculate feet poses for start/end of footstep plan
        flor_state_msgs::LowerBodyState lower_body_state_;
        geometry_msgs::PoseStamped goal_pose_;

        StepPlanRequestClient* step_plan_request_client_;
        EditStepClient* edit_step_client_;
    };
}
