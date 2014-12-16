#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <boost/thread.hpp>

#include <vector>
#include <string>
#include <fstream>

#include <tf/transform_broadcaster.h>

#include <flor_ocs_msgs/OCSTemplateAdd.h>
#include <flor_ocs_msgs/OCSTemplateRemove.h>
#include <flor_ocs_msgs/OCSTemplateList.h>
#include <flor_ocs_msgs/OCSTemplateUpdate.h>
#include <flor_grasp_msgs/GraspSelection.h>
#include <flor_grasp_msgs/GraspState.h>
#include <flor_grasp_msgs/TemplateSelection.h>
#include <vigir_object_template_msgs/GetTemplateStateAndTypeInfo.h>
#include <vigir_object_template_msgs/GetGraspInfo.h>

#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/Grasp.h>
#include <geometric_shapes/mesh_operations.h>
#include <shape_msgs/Mesh.h>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_messages.h>
#include <geometric_shapes/shape_operations.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>

namespace ocs_template
{

    struct VigirObjectTemplate
    {
        uint16_t                                  id;
        uint16_t                                  type;
        std::string                               name;
        float                                     mass;
        geometry_msgs::Point                      com;
        geometry_msgs::Point                      b_max;
        geometry_msgs::Point                      b_min;
        shape_msgs::Mesh                          mesh;
        std::map<unsigned int,moveit_msgs::Grasp> grasps;
        std::vector<geometry_msgs::PoseStamped>   stand_poses;

        VigirObjectTemplate() : id(0),
                                mass(0.0)
        {
            //com   = geometry_msgs::Point(0.0,0.0,0.0);
        }
    };

    class TemplateNodelet : public nodelet::Nodelet
    {
      public:
        virtual void onInit();

        void addTemplateCb(const flor_ocs_msgs::OCSTemplateAdd::ConstPtr& msg);
        void removeTemplateCb(const flor_ocs_msgs::OCSTemplateRemove::ConstPtr& msg);
        void updateTemplateCb(const flor_ocs_msgs::OCSTemplateUpdate::ConstPtr& msg);
        void graspRequestCb(const flor_grasp_msgs::GraspSelection::ConstPtr& msg);
        void graspStateFeedbackCb(const flor_grasp_msgs::GraspState::ConstPtr& msg);
        void templateMatchFeedbackCb(const flor_grasp_msgs::TemplateSelection::ConstPtr& msg);
        void publishTemplateList();
        std::vector< std::vector <std::string> > readCSVFile(std::string& file_name);
        void loadObjectTemplateDatabase(std::string& file_name);
        void loadGraspDatabase(std::string& file_name, std::string hand_side);
        void loadStandPosesDatabase(std::string& file_name);
        int  staticTransform(geometry_msgs::Pose& palm_pose);
        void gripperTranslationToPreGraspPose(geometry_msgs::Pose& pose, moveit_msgs::GripperTranslation& trans);
        void timerCallback(const ros::TimerEvent& event);
        bool templateInfoSrv(vigir_object_template_msgs::GetTemplateStateAndTypeInfo::Request& req,
                             vigir_object_template_msgs::GetTemplateStateAndTypeInfo::Response& res);

        bool graspInfoSrv(vigir_object_template_msgs::GetGraspInfo::Request& req,
                          vigir_object_template_msgs::GetGraspInfo::Response& res);

      protected:
        ros::Subscriber template_update_sub_;
        ros::Subscriber template_add_sub_;
        ros::Subscriber template_remove_sub_;
        ros::Subscriber grasp_request_sub_;
        ros::Subscriber grasp_state_feedback_sub_;
        ros::Subscriber template_match_feedback_sub_;
        ros::Publisher template_list_pub_;
        ros::Publisher grasp_selected_pub_;
        ros::Publisher grasp_selected_state_pub_;

        ros::ServiceServer template_info_server_;
        ros::ServiceServer grasp_info_server_;

        ros::Timer image_publish_timer_;

      private:
        std::vector<unsigned char>                  template_id_list_;
        std::vector<std::string>                    template_list_;
        std::vector<geometry_msgs::PoseStamped>     pose_list_;
        unsigned char id_counter_;
        // Filename of the grasping library
        std::string   r_grasps_filename_;
        std::string   l_grasps_filename_;
        // Filename of the object template library
        std::string   ot_filename_;
        // Filename of the object template library
        std::string   stand_filename_;
        tf::Transform gp_T_hand_;
        std::map<unsigned int,VigirObjectTemplate> object_template_map_;

        robot_model_loader::RobotModelLoaderPtr    hand_model_loader_;
        robot_model::RobotModelPtr                 hand_robot_model_;

        ros::Timer timer;

        tf::TransformBroadcaster tfb_;
    };
}
