/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013-2015, Team ViGIR ( TORC Robotics LLC, TU Darmstadt, Virginia Tech, Oregon State University, Cornell University, and Leibniz University Hanover )
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
//@TODO_ADD_AUTHOR_INFO
#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <boost/thread.hpp>

#include <vector>
#include <string>
#include <fstream>

#include <std_msgs/Empty.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <vigir_ocs_msgs/OCSTemplateAdd.h>
#include <vigir_ocs_msgs/OCSTemplateRemove.h>
#include <vigir_ocs_msgs/OCSTemplateList.h>
#include <vigir_ocs_msgs/OCSTemplateUpdate.h>
#include <vigir_grasp_msgs/GraspSelection.h>
#include <vigir_grasp_msgs/GraspState.h>
#include <vigir_grasp_msgs/TemplateSelection.h>
#include <vigir_object_template_msgs/GetTemplateStateAndTypeInfo.h>
#include <vigir_object_template_msgs/GetGraspInfo.h>
#include <vigir_object_template_msgs/GetInstantiatedGraspInfo.h>
#include <vigir_object_template_msgs/SetAttachedObjectTemplate.h>
#include <vigir_object_template_msgs/GetUsabilityInWristFrame.h>
#include <vigir_object_template_msgs/Affordance.h>
#include <vigir_object_template_msgs/Usability.h>
#include <vigir_object_template_msgs/TemplateGraspUpdate.h>
#include <vigir_object_template_msgs/TemplateAffordanceUpdate.h>
#include <vigir_object_template_msgs/TemplateStandPoseUpdate.h>


#include <geometry_msgs/PoseStamped.h>
#include <geometric_shapes/mesh_operations.h>
#include <shape_msgs/Mesh.h>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_messages.h>
#include <geometric_shapes/shape_operations.h>
#include <shape_tools/solid_primitive_dims.h>
#include <moveit_msgs/Grasp.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/PlanningScene.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model/link_model.h>

#include <tinyxml.h>

namespace ocs_template
{

    struct VigirObjectTemplate
    {
        uint16_t                                                     id;
        uint16_t                                                     type;
        std::string                                                  name;
        float                                                        mass;
        geometry_msgs::Point                                         com;
        geometry_msgs::Point                                         b_max;
        geometry_msgs::Point                                         b_min;
        shape_msgs::Mesh                                             mesh;
        std::string                                                  path;
        std::map<unsigned int,moveit_msgs::Grasp>                    grasps;
        std::map<unsigned int,vigir_object_template_msgs::StandPose> stand_poses;
        std::map<unsigned int,vigir_object_template_msgs::Usability> usabilities;
        std::map<unsigned int,vigir_object_template_msgs::Affordance>affordances;

        VigirObjectTemplate() : id(0),
                                type(0),
                                mass(0.0)
        {
            //com   = geometry_msgs::Point(0.0,0.0,0.0);
        }
    };

    class TemplateNodelet : public nodelet::Nodelet
    {
      public:
        virtual void onInit();

        void clearTemplateCb(const std_msgs::Empty);
        void addTemplateCb(const vigir_ocs_msgs::OCSTemplateAdd::ConstPtr msg);
        void removeTemplateCb(const vigir_ocs_msgs::OCSTemplateRemove::ConstPtr msg);
        void updateTemplateCb(const vigir_ocs_msgs::OCSTemplateUpdate::ConstPtr msg);
        void stitchTemplateFwdCb(const vigir_object_template_msgs::TemplateStateInfo::ConstPtr msg);
        void detachTemplateFwdCb(const vigir_object_template_msgs::TemplateStateInfo::ConstPtr msg);
        void snapTemplateCb(const vigir_grasp_msgs::TemplateSelection::ConstPtr msg);
        void graspStateFeedbackCb(const vigir_grasp_msgs::GraspState::ConstPtr msg);
        void templateMatchFeedbackCb(const vigir_grasp_msgs::TemplateSelection::ConstPtr msg);
        void publishTemplateList();
        void loadObjectTemplateDatabaseXML(std::string& file_name);
        void loadGraspDatabaseXML(std::string& file_name, std::string hand_side);
        void loadStandPosesDatabaseXML(std::string& file_name);
        int  worldPoseTransform(const geometry_msgs::PoseStamped& template_pose, const geometry_msgs::Pose &input_pose, geometry_msgs::PoseStamped &target_pose);
        int  poseTransform(geometry_msgs::Pose& first_pose, geometry_msgs::Pose& second_pose, geometry_msgs::Pose &target_pose);
        int  staticTransform(geometry_msgs::Pose& palm_pose, tf::Transform gp_T_hand);
        void gripperTranslationToPreGraspPose(geometry_msgs::Pose& pose, moveit_msgs::GripperTranslation& trans);
        void timerCallback(const ros::TimerEvent& event);
        bool templateInfoSrv(vigir_object_template_msgs::GetTemplateStateAndTypeInfo::Request& req,
                             vigir_object_template_msgs::GetTemplateStateAndTypeInfo::Response& res);

        bool graspInfoSrv(vigir_object_template_msgs::GetGraspInfo::Request& req,
                          vigir_object_template_msgs::GetGraspInfo::Response& res);

        bool instantiatedGraspInfoSrv(vigir_object_template_msgs::GetInstantiatedGraspInfo::Request& req,
                                      vigir_object_template_msgs::GetInstantiatedGraspInfo::Response& res);

        bool stitchObjectTemplateSrv(vigir_object_template_msgs::SetAttachedObjectTemplate::Request& req,
                                     vigir_object_template_msgs::SetAttachedObjectTemplate::Response& res);

        bool detachObjectTemplateSrv(vigir_object_template_msgs::SetAttachedObjectTemplate::Request& req,
                                     vigir_object_template_msgs::SetAttachedObjectTemplate::Response& res);

        bool usabilityPoseSrv(vigir_object_template_msgs::GetUsabilityInWristFrame::Request& req,
                              vigir_object_template_msgs::GetUsabilityInWristFrame::Response& res);

        //Planning Scene
        void addCollisionObject(int type, int index, std::string mesh_name, geometry_msgs::Pose pose);
        void moveCollisionObject(int template_id, geometry_msgs::Pose pose);
        void removeCollisionObject(int template_id);
        void updateStandPose(vigir_object_template_msgs::TemplateStandPoseUpdate::ConstPtr msg);
        void updateAffordance(vigir_object_template_msgs::TemplateAffordanceUpdate::ConstPtr msg);
        void updateGrasp(vigir_object_template_msgs::TemplateGraspUpdate::ConstPtr msg);

      protected:
        ros::Subscriber template_update_sub_;
        ros::Subscriber template_clear_sub_;
        ros::Subscriber template_add_sub_;
        ros::Subscriber template_remove_sub_;
        ros::Subscriber grasp_state_feedback_sub_;
        ros::Subscriber template_match_feedback_sub_;
        ros::Subscriber template_snap_sub_;
        ros::Publisher  template_list_pub_;
        ros::Publisher  grasp_selected_pub_;
        ros::Publisher  grasp_selected_state_pub_;
        ros::Publisher  co_pub_;
        ros::Publisher  aco_pub_;
        ros::Publisher  planning_scene_diff_pub_;

        //Forward topics through comms
        ros::Publisher  template_update_pub_;
        ros::Publisher  template_add_pub_;
        ros::Publisher  template_remove_pub_;
        ros::Publisher  stitch_template_pub_;
        ros::Subscriber stitch_template_sub_;
        ros::Publisher  detach_template_pub_;
        ros::Subscriber detach_template_sub_;
        ros::Subscriber stand_update_sub_;
        ros::Subscriber affordance_update_sub_;
        ros::Subscriber grasp_update_sub_;

        ros::ServiceServer template_info_server_;
        ros::ServiceServer grasp_info_server_;
        ros::ServiceServer inst_grasp_info_server_;
        ros::ServiceServer stitch_object_server_;
        ros::ServiceServer detach_object_server_;
        ros::ServiceServer usability_pose_server_;

        ros::Timer image_publish_timer_;

      private:
        std::vector<unsigned char>                 template_id_list_;
        std::vector<unsigned char>                 template_type_list_;
        std::vector<std::string>                   template_name_list_;
        std::vector<geometry_msgs::PoseStamped>    template_pose_list_;
        std::vector<unsigned char>                 template_status_list_; //0-normal, 1-attached
        unsigned char                              id_counter_;
        std::vector<geometry_msgs::PoseStamped>    template_last_pose_list_;
        // Filename of the grasping library
        std::string                                r_grasps_filename_;
        std::string                                l_grasps_filename_;
        // Filename of the object template library
        std::string                                ot_filename_;
        // Filename of the stand pose library
        std::string                                stand_filename_;
        // Hand parameters
        std::string palm_link_;
        std::string wrist_link_;
        std::string hand_group_;
        std::string left_wrist_link_, right_wrist_link_;
        std::string left_palm_link_,  right_palm_link_;
        std::string left_hand_group_, right_hand_group_;

        //Server mode parameter
        bool master_mode_;

        tf::Transform                              palm_T_lhand_;
        tf::Transform                              palm_T_rhand_;
        typedef std::map<unsigned int,VigirObjectTemplate> VigirObjectTemplateMap;
        typedef VigirObjectTemplateMap::iterator VigirObjectTemplateMapIter;
        typedef VigirObjectTemplateMap::const_iterator VigirObjectTemplateMapCIter;
        VigirObjectTemplateMap object_template_map_;

        robot_model_loader::RobotModelLoaderPtr    robot_model_loader_;
        robot_model::RobotModelPtr                 robot_model_;
        std::vector<std::string>                   hand_joint_names_;
        std::vector<std::string>                   hand_link_names_;

        boost::recursive_mutex template_list_mutex_;
        boost::recursive_mutex object_template_map_mutex_;

        ros::Timer timer;

        tf::TransformBroadcaster tfb_;
    };
}
