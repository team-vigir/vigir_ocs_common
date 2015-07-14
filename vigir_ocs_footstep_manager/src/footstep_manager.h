/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013-2015, TORC Robotics, LLC ( Team ViGIR )
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Team ViGIR, TORC Robotics, nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
@TODO_ADD_AUTHOR_INFO
#include <ros/ros.h>
#include <tf/tf.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <boost/thread.hpp>

#include <vector>
#include <stack>
#include <string>

#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int8.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>

#include <vigir_footstep_planning_msgs/footstep_planning_msgs.h>
#include <vigir_footstep_planning_msgs/visualization.h>

#include <vigir_ocs_msgs/OCSFootstepList.h>
#include <vigir_ocs_msgs/OCSFootstepUpdate.h>
#include <vigir_ocs_msgs/OCSFootstepPlanGoal.h>
#include <vigir_ocs_msgs/OCSFootstepPlanGoalUpdate.h>
#include <vigir_ocs_msgs/OCSFootstepPlanParameters.h>
#include <vigir_ocs_msgs/OCSFootstepPlanUpdate.h>
#include <vigir_ocs_msgs/OCSFootstepParamSetList.h>
#include <vigir_ocs_msgs/OCSFootstepStatus.h>
#include <vigir_ocs_msgs/OCSFootstepPlanRequest.h>
#include <vigir_ocs_msgs/OCSFootstepSyncStatus.h>

#include <vigir_footstep_planning_msgs/footstep_planning_msgs.h>
#include <vigir_footstep_planning_msgs/parameter_set.h>

//#include <vigir_footstep_planning_lib/helper.h>
#include <vigir_foot_pose_transformer/foot_pose_transformer.h>

#include <actionlib/client/simple_action_client.h>

namespace ocs_footstep
{
    typedef actionlib::SimpleActionClient<vigir_footstep_planning_msgs::UpdateFeetAction> UpdateFeetClient;
    typedef actionlib::SimpleActionClient<vigir_footstep_planning_msgs::StepPlanRequestAction> StepPlanRequestClient;
    typedef actionlib::SimpleActionClient<vigir_footstep_planning_msgs::EditStepAction> EditStepClient;
    typedef actionlib::SimpleActionClient<vigir_footstep_planning_msgs::StitchStepPlanAction> StitchStepPlanClient;
    typedef actionlib::SimpleActionClient<vigir_footstep_planning_msgs::UpdateStepPlanAction> UpdateStepPlanClient;
    typedef actionlib::SimpleActionClient<vigir_footstep_planning_msgs::ExecuteStepPlanAction> ExecuteStepPlanClient;
    typedef actionlib::SimpleActionClient<vigir_footstep_planning_msgs::GetAllParameterSetsAction> GetAllParameterSetsClient;

    class FootstepManager : public nodelet::Nodelet
    {
    public:
        virtual void onInit();

        // triggers footstep plan calls
        void processFootstepPlanGoal(const vigir_ocs_msgs::OCSFootstepPlanGoal::ConstPtr& plan_goal);
        void processFootstepPlanGoalFeedback(const vigir_ocs_msgs::OCSFootstepPlanGoalUpdate::ConstPtr& plan_goal);
        void processFootstepPlanRequest(const std_msgs::Int8::ConstPtr& plan_request);
        void processFootstepPlanUpdate(const vigir_ocs_msgs::OCSFootstepPlanUpdate::ConstPtr& msg);
        void processFootstepPlanParameters(const vigir_ocs_msgs::OCSFootstepPlanParameters::ConstPtr& msg);

        // feedback look for interaction, should update stepplan and use actions to edit/update
        void processFootstepPoseUpdate(const vigir_ocs_msgs::OCSFootstepUpdate::ConstPtr& msg);
        void processUndoRequest(const std_msgs::Int8::ConstPtr& msg);
        void processRedoRequest(const std_msgs::Int8::ConstPtr& msg);
        void processSetStartIndex(const std_msgs::Int32::ConstPtr& msg);
        void processValidatePlanRequest(const std_msgs::Int8::ConstPtr& msg);
        void processExecuteFootstepRequest(const std_msgs::Int8::ConstPtr& msg);
        void processSendOCSStepPlanRequest(const std_msgs::Int8::ConstPtr& msg);
        void processStitchPlansRequest(const std_msgs::Int8::ConstPtr& msg);
        void processFootstepParamSetSelected(const std_msgs::String::ConstPtr& msg);

        // used in the obfsm interaction
        void processExecuteStepPlanResult(vigir_footstep_planning_msgs::ExecuteStepPlanResult result);
        void processExecuteStepPlanResult(const vigir_footstep_planning_msgs::ExecuteStepPlanActionResult::ConstPtr& msg);
        void processUpdateFeetResult(vigir_footstep_planning_msgs::UpdateFeetResult result);
        void processUpdateFeetResult(const vigir_footstep_planning_msgs::UpdateFeetActionResult::ConstPtr& msg);
        void processGenerateFeetPoseResult(const vigir_footstep_planning_msgs::GenerateFeetPoseActionResult::ConstPtr& msg);

        // used to process new step plans whenever and however they arrive
        void processNewStepPlanGoal(vigir_footstep_planning_msgs::Feet& goal);
        void processNewStepPlan(vigir_footstep_planning_msgs::StepPlan& step_plan);
        // callbacks for onboard actions
        void processOnboardStepPlanRequest(const vigir_footstep_planning_msgs::StepPlanRequest::ConstPtr& step_plan_request);
        void processOnboardStepPlan(const vigir_footstep_planning_msgs::StepPlan::ConstPtr& step_plan);
        // callback for the ocs planner feedback
        void processOCSStepPlanRequest(const vigir_footstep_planning_msgs::StepPlanRequest::ConstPtr& step_plan_request);
        void processOCSStepPlan(const vigir_footstep_planning_msgs::StepPlan::ConstPtr& step_plan);

        void timerCallback(const ros::TimerEvent& event);

        // callbacks for actions
        //updatefeet
        void activeUpdateFeet();
        void feedbackUpdateFeet(const vigir_footstep_planning_msgs::UpdateFeetFeedbackConstPtr& feedback);
        void doneUpdateFeet(const actionlib::SimpleClientGoalState& state, const vigir_footstep_planning_msgs::UpdateFeetResultConstPtr& result);
        //stepplanrequest
        void activeStepPlanRequest();
        void feedbackStepPlanRequest(const vigir_footstep_planning_msgs::StepPlanRequestFeedbackConstPtr& feedback);
        void doneStepPlanRequest(const actionlib::SimpleClientGoalState& state, const vigir_footstep_planning_msgs::StepPlanRequestResultConstPtr& result);
        //editstep
        void activeEditStep();
        void feedbackEditStep(const vigir_footstep_planning_msgs::EditStepFeedbackConstPtr& feedback);
        void doneEditStep(const actionlib::SimpleClientGoalState& state, const vigir_footstep_planning_msgs::EditStepResultConstPtr& result);
        //stitchstepplan
        void activeStitchStepPlan();
        void feedbackStitchStepPlan(const vigir_footstep_planning_msgs::StitchStepPlanFeedbackConstPtr& feedback);
        void doneStitchStepPlan(const actionlib::SimpleClientGoalState& state, const vigir_footstep_planning_msgs::StitchStepPlanResultConstPtr& result);
        //updatestepplan
        void activeUpdateStepPlan();
        void feedbackUpdateStepPlan(const vigir_footstep_planning_msgs::UpdateStepPlanFeedbackConstPtr& feedback);
        void doneUpdateStepPlan(const actionlib::SimpleClientGoalState& state, const vigir_footstep_planning_msgs::UpdateStepPlanResultConstPtr& result);
        //executestep
        void activeExecuteStepPlan();
        void feedbackExecuteStepPlan(const vigir_footstep_planning_msgs::ExecuteStepPlanFeedbackConstPtr& feedback);
        void doneExecuteStepPlan(const actionlib::SimpleClientGoalState& state, const vigir_footstep_planning_msgs::ExecuteStepPlanResultConstPtr& result);
        //getallparametersets
        void activeGetAllParameterSets();
        void feedbackGetAllParameterSets(const vigir_footstep_planning_msgs::GetAllParameterSetsFeedbackConstPtr& feedback);
        void doneGetAllParameterSets(const actionlib::SimpleClientGoalState& state, const vigir_footstep_planning_msgs::GetAllParameterSetsResultConstPtr& result);

    private:
        // send action goals
        void sendUpdateFeetGoal(vigir_footstep_planning_msgs::Feet feet);
        void sendStepPlanRequestGoal(vigir_footstep_planning_msgs::Feet start, vigir_footstep_planning_msgs::Feet goal, const unsigned int start_step_index = 0, const unsigned char start_foot = vigir_footstep_planning_msgs::StepPlanRequest::AUTO);
        void sendEditStepGoal(vigir_footstep_planning_msgs::StepPlan step_plan, vigir_footstep_planning_msgs::Step step);
        void sendStitchStepPlanGoal(std::vector<vigir_footstep_planning_msgs::StepPlan>& step_plan_list);
        void sendUpdateStepPlanGoal(vigir_footstep_planning_msgs::StepPlan step_plan);
        void sendExecuteStepPlanGoal();
        void sendGetAllParameterSetsGoal();

        // OBFSM
        void sendEditedSteps();
        void sendCurrentStepPlan();
        void sendStepPlanGoal();
        void sendStepPlanGoalFeet();

        // sync status with onboard manager
        void publishSyncStatus();

        // for visualization
        void publishFootsteps();
        void publishFootstepList();
        void publishFootstepParameterSetList();

        // interactive markers for goal
        void publishGoalMarkerClear();
        void publishGoalMarkerFeedback();

        // plan requests
        void calculateGoal(); // calculate initial goal pose
        void requestStepPlanFromRobot();
        void requestStepPlanFromStep(vigir_footstep_planning_msgs::Step &step);
        void updateGoalVisMsgs();
        void updateStepPlanVisMsgs();

        // auxiliary functions that create visualizations based on step plans
        void feetToFootMarkerArray(vigir_footstep_planning_msgs::Feet& input, visualization_msgs::MarkerArray& foot_array_msg);
        void stepPlanToFootMarkerArray(std::vector<vigir_footstep_planning_msgs::StepPlan>& input, visualization_msgs::MarkerArray& foot_array_msg);
        void stepPlanToBodyMarkerArray(std::vector<vigir_footstep_planning_msgs::StepPlan>& input, visualization_msgs::MarkerArray& body_array_msg);
        void stepPlanToFootPath(std::vector<vigir_footstep_planning_msgs::StepPlan>& input, nav_msgs::Path& foot_path_msg);

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
        bool findStep(const unsigned int& step_index, vigir_footstep_planning_msgs::Step& step, unsigned int& step_plan_index);
        bool findStepPlan(const unsigned int& step_index, unsigned int& step_plan_index);
        void extendPlanList(const vigir_footstep_planning_msgs::StepPlan &new_step_plan);

        ros::Timer timer;

        ros::Publisher footstep_array_pub_;
        ros::Publisher footstep_body_bb_array_pub_;
        ros::Publisher footstep_path_pub_;
        ros::Publisher plan_goal_array_pub_;

        ros::Publisher footstep_list_pub_;
        ros::Subscriber footstep_update_sub_;
        ros::Subscriber footstep_undo_req_sub_;
        ros::Publisher footstep_has_undo_pub_;
        ros::Subscriber footstep_redo_req_sub_;
        ros::Publisher footstep_has_redo_pub_;
        ros::Subscriber footstep_start_index_pub_;
        ros::Subscriber footstep_execute_req_sub_;
        ros::Subscriber footstep_send_ocs_plan_req_sub_;
        ros::Subscriber footstep_stitch_req_sub_;
        ros::Publisher footstep_plan_parameters_pub_;
        ros::Subscriber footstep_plan_parameters_sub_;
        ros::Publisher footstep_param_set_list_pub_;
        ros::Subscriber footstep_param_set_selected_sub_;
        ros::Publisher footstep_param_set_selected_pub_;
        ros::Publisher footstep_param_set_selected_ocs_pub_;

        ros::Publisher footstep_goal_pose_pub_;
        ros::Subscriber footstep_goal_pose_sub_;
        ros::Subscriber footstep_plan_goal_sub_;
        ros::Subscriber footstep_plan_request_sub_;
        ros::Subscriber footstep_plan_update_sub_;

        ros::Subscriber validate_step_plan_sub_;
        // obfsm topics
        ros::Publisher obfsm_step_update_pub_;
        ros::Publisher obfsm_update_step_plan_goal_pub_;
        ros::Publisher obfsm_plan_goal_pub_;
        ros::Publisher obfsm_plan_parameters_goal_pub_;
        ros::Publisher obfsm_set_active_parameter_set_pub_;
        ros::Publisher obfsm_updated_step_plan_pub_;
        ros::Publisher obfsm_execute_step_plan_pub_;
        ros::Publisher obfsm_replan_request_pub_;
        ros::Subscriber obfsm_execute_step_plan_sub_;
        ros::Subscriber obfsm_current_step_plan_sub_;
        ros::Subscriber obfsm_generate_feet_pose_sub_;
        ros::Subscriber obfsm_active_parameter_set_sub_;

        // used exclusively to check what should be sent to the obfsm
        std::map<ros::Time,std::set<unsigned char> > updated_steps_;
        bool updated_goal_;

        // onboard planners feedback
        ros::Subscriber onboard_step_plan_request_sub_;
        ros::Subscriber onboard_step_plan_sub_;

        // ocs planner feedback
        ros::Subscriber ocs_step_plan_request_sub_;
        ros::Subscriber ocs_step_plan_sub_;

        // step plan request feedback
        ros::Publisher planner_plan_request_feedback_cloud_pub_;

        // step plan starting steps
        ros::Publisher start_step_index_feet_pub_;

        // update footstep planner feedback
        ros::Publisher planner_status_pub_;

        // sync status between footstep managers
        ros::Publisher sync_status_pub_;

        // feet pose generator client
        ros::ServiceClient generate_feet_pose_client;

        std::stack< std::vector<vigir_footstep_planning_msgs::StepPlan> > footstep_plans_undo_stack_;
        std::stack< std::vector<vigir_footstep_planning_msgs::StepPlan> > footstep_plans_redo_stack_;
        visualization_msgs::MarkerArray footstep_array_;
        visualization_msgs::MarkerArray footstep_body_array_;
        visualization_msgs::MarkerArray footstep_goal_array_;
        nav_msgs::Path footstep_path_;

        // Footstep parameter sets
        std::vector<vigir_footstep_planning_msgs::ParameterSet> footstep_parameter_set_list_;
        std::string selected_footstep_parameter_set_;

        // Parameters
        geometry_msgs::Vector3 foot_size;

        geometry_msgs::Vector3 upper_body_size;
        geometry_msgs::Vector3 upper_body_origin_shift;

        // used to calculate feet poses for start/end of footstep plan
        geometry_msgs::PoseStamped goal_pose_;
        vigir_footstep_planning_msgs::Feet goal_;

        // last step plan request received, saved and used mostly for message parameters
        vigir_ocs_msgs::OCSFootstepPlanParameters planner_config_;

        // specifies which footstep will be used as starting point for the planner, -1 to start a new one
        int start_step_index_;

        UpdateFeetClient* update_feet_client_;
        StepPlanRequestClient* step_plan_request_client_;
        EditStepClient* edit_step_client_;
        StitchStepPlanClient* stitch_step_plan_client_;
        UpdateStepPlanClient* update_step_plan_client_;
        ExecuteStepPlanClient* execute_step_plan_client_;
        GetAllParameterSetsClient* get_all_parameter_sets_client_;

        // local instance of foot pose transformer
        vigir_footstep_planning::FootPoseTransformer::Ptr foot_pose_transformer_;

        boost::recursive_mutex step_plan_mutex_;
        boost::recursive_mutex param_mutex_;
        boost::recursive_mutex goal_mutex_;

        ros::Time last_ocs_step_plan_stamp_;
        ros::Time last_onboard_step_plan_stamp_;
        ros::Time last_validated_step_plan_stamp_;
        ros::Time last_executed_step_plan_stamp_;
    };
}
