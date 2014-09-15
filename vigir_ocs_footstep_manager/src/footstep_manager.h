#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <boost/thread.hpp>

#include <vector>
#include <stack>
#include <string>

#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>

#include <vigir_footstep_planning_msgs/StepPlan.h>
#include <flor_ocs_msgs/OCSFootstepList.h>
#include <flor_ocs_msgs/OCSFootstepUpdate.h>
#include <flor_ocs_msgs/OCSFootstepPlanRequest.h>
#include <flor_state_msgs/LowerBodyState.h>
#include <vigir_footstep_planning_msgs/StepPlanRequestAction.h>
#include <vigir_footstep_planning_msgs/EditStepAction.h>
#include <vigir_footstep_planning_msgs/ExecuteStepPlanAction.h>

#include <actionlib/client/simple_action_client.h>

namespace ocs_footstep
{
    typedef actionlib::SimpleActionClient<vigir_footstep_planning_msgs::StepPlanRequestAction> StepPlanRequestClient;
    typedef actionlib::SimpleActionClient<vigir_footstep_planning_msgs::EditStepAction> EditStepClient;
    typedef actionlib::SimpleActionClient<vigir_footstep_planning_msgs::ExecuteStepPlanAction> ExecuteStepPlanClient;

    class FootstepManager : public nodelet::Nodelet
    {
    public:
        virtual void onInit();

        // triggers footstep plan calls
        void processFootstepPlanRequest(const flor_ocs_msgs::OCSFootstepPlanRequest::ConstPtr& plan_request);

        // feedback look for interaction, should update stepplan and use actions to edit/update
        void processFootstepPoseUpdate(const flor_ocs_msgs::OCSFootstepUpdate::ConstPtr& msg);
        void processUndoRequest(const std_msgs::Bool::ConstPtr& msg);
        void processRedoRequest(const std_msgs::Bool::ConstPtr& msg);
        void processExecuteFootstepRequest(const std_msgs::Bool::ConstPtr& msg);

        // get the current and goal poses to be used when requesting a footstep plan
        void processLowerBodyState(const flor_state_msgs::LowerBodyState::ConstPtr& lower_body_state);

        // callbacks for actions
        //stepplanrequest
        void activeStepPlanRequest();
        void feedbackStepPlanRequest(const vigir_footstep_planning_msgs::StepPlanRequestFeedbackConstPtr& feedback);
        void doneStepPlanRequest(const actionlib::SimpleClientGoalState& state, const vigir_footstep_planning_msgs::StepPlanRequestResultConstPtr& result);
        //editstep
        void activeEditStep();
        void feedbackEditStep(const vigir_footstep_planning_msgs::EditStepFeedbackConstPtr& feedback);
        void doneEditStep(const actionlib::SimpleClientGoalState& state, const vigir_footstep_planning_msgs::EditStepResultConstPtr& result);
        //executestep
        void activeExecuteStepPlan();
        void feedbackExecuteStepPlan(const vigir_footstep_planning_msgs::ExecuteStepPlanFeedbackConstPtr& feedback);
        void doneExecuteStepPlan(const actionlib::SimpleClientGoalState& state, const vigir_footstep_planning_msgs::ExecuteStepPlanResultConstPtr& result);

        void timerCallback(const ros::TimerEvent& event);

    private:
        // send action goals
        void sendStepPlanRequestGoal(vigir_footstep_planning_msgs::Feet& start, vigir_footstep_planning_msgs::Feet& goal, const unsigned int start_index = 0);
        void sendEditStepGoal(vigir_footstep_planning_msgs::StepPlan& step_plan, vigir_footstep_planning_msgs::Step& step);
        void sendExecuteStepPlanGoal();

        // for visualization
        void publishFootsteps();
        void publishFootstepList();

        // plan requests
        void requestStepPlanFromRobot();
        void requestStepPlanFromStep(vigir_footstep_planning_msgs::Step &step);
        void updateVisualizationMsgs();

        // auxiliary functions that create visualizations based on step plans
        void stepPlanToFootMarkerArray(std::vector<vigir_footstep_planning_msgs::StepPlan>& input, visualization_msgs::MarkerArray& foot_array_msg);
        void stepPlanToBodyMarkerArray(std::vector<vigir_footstep_planning_msgs::StepPlan>& input, visualization_msgs::MarkerArray& body_array_msg);
        void stepPlanToFootPath(std::vector<vigir_footstep_planning_msgs::StepPlan>& input, nav_msgs::Path& foot_path_msg);
        void stepToMarker(const vigir_footstep_planning_msgs::Step &step, visualization_msgs::Marker &marker);

        // clears footstep visualizations based on the last messages sent
        void cleanMarkerArray(visualization_msgs::MarkerArray& old_array, visualization_msgs::MarkerArray& new_array);

        // functions that control the undo/redo stacks
        void addNewPlanList();
        void addCopyPlanList();
        void cleanStacks();

        // helper functions that return current step plan list and step plan, to reduce clutter
        inline std::vector<vigir_footstep_planning_msgs::StepPlan>& getStepPlanList() { return footstep_plans_undo_stack_.top(); }
        inline vigir_footstep_planning_msgs::StepPlan& getStepPlan() { return getStepPlanList().back(); }

        // helper function for finding step based on step_index
        bool findStep(const int& step_index, vigir_footstep_planning_msgs::StepPlan& step_plan, vigir_footstep_planning_msgs::Step& step);

        ros::Timer timer;

        ros::Publisher footstep_array_pub_;
        ros::Publisher footstep_body_bb_array_pub_;
        ros::Publisher footstep_path_pub_;
        ros::Subscriber footstep_array_sub_;
        ros::Subscriber footstep_body_bb_array_sub_;
        ros::Subscriber footstep_path_sub_;

        ros::Publisher footstep_list_pub_;
        ros::Subscriber footstep_update_sub_;
        ros::Subscriber footstep_undo_req_sub_;
        ros::Subscriber footstep_redo_req_sub_;
        ros::Subscriber footstep_exec_req_sub_;

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
        ExecuteStepPlanClient* execute_step_plan_client_;
    };
}
