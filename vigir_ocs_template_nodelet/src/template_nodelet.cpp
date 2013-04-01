#include "template_nodelet.h"

namespace ocs_template
{
void TemplateNodelet::onInit()
{
    ros::NodeHandle& nh         = getNodeHandle();
    ros::NodeHandle nh_out(nh, "template");

    // also create a publisher to set parameters of cropped image
    template_list_pub_         = nh_out.advertise<flor_ocs_msgs::OCSTemplateList>( "list", 1, false );
    template_selected_pub_     = nh_out.advertise<flor_grasp_msgs::TemplateSelection>( "template_selected", 1, false );
    grasp_selected_pub_        = nh_out.advertise<flor_grasp_msgs::GraspSelection>( "grasp_selected", 1, false );
    grasp_selected_state_pub_  = nh_out.advertise<flor_grasp_msgs::GraspState>( "grasp_selected_state", 1, false );

    // then, subscribe to the resulting cropped image
    template_add_sub_            = nh_out.subscribe<flor_ocs_msgs::OCSTemplateAdd>( "add", 1, &TemplateNodelet::addTemplateCb, this );
    template_remove_sub_         = nh_out.subscribe<flor_ocs_msgs::OCSTemplateRemove>( "remove", 1, &TemplateNodelet::removeTemplateCb, this );
    template_update_sub_         = nh_out.subscribe<flor_ocs_msgs::OCSTemplateUpdate>( "update", 1, &TemplateNodelet::updateTemplateCb, this );
    template_match_request_sub_  = nh_out.subscribe<flor_grasp_msgs::TemplateSelection>( "template_match_request", 1, &TemplateNodelet::templateMatchRequestCb, this );
    template_match_feedback_sub_ = nh_out.subscribe<flor_grasp_msgs::TemplateSelection>( "template_match_feedback", 1, &TemplateNodelet::templateMatchFeedbackCb, this );
    grasp_request_sub_           = nh_out.subscribe<flor_grasp_msgs::GraspSelection>( "grasp_request", 1, &TemplateNodelet::graspRequestCb, this );
    grasp_state_feedback_sub_    = nh_out.subscribe<flor_grasp_msgs::GraspState>( "grasp_state_feedback", 1, &TemplateNodelet::graspStateFeedbackCb, this );
    
    id_counter_ = 0;
}

void TemplateNodelet::addTemplateCb(const flor_ocs_msgs::OCSTemplateAdd::ConstPtr& msg)
{
    std::cout << "Added template to list (id: " << (int)id_counter_ << ")" << std::endl;
    template_id_list_.push_back(id_counter_++);
    template_list_.push_back(msg->template_path);
    pose_list_.push_back(msg->pose);
    this->publishTemplateList();
}

void TemplateNodelet::removeTemplateCb(const flor_ocs_msgs::OCSTemplateRemove::ConstPtr& msg)
{
    std::cout << "Removing template " << msg->template_id << " from list... ";
    int index = 0;
    for(; index < template_id_list_.size(); index++)
        if(template_id_list_[index] == msg->template_id)
            break;
    if(index < template_id_list_.size())
    {
        std::cout << "Removed!" << std::endl;
        template_id_list_.erase(template_id_list_.begin()+index);
        template_list_.erase(template_list_.begin()+index);
        pose_list_.erase(pose_list_.begin()+index);
        this->publishTemplateList();
    }
}

void TemplateNodelet::updateTemplateCb(const flor_ocs_msgs::OCSTemplateUpdate::ConstPtr& msg)
{
    std::cout << "Updating template " << msg->template_id << "... ";
    int index = 0;
    for(; index < template_id_list_.size(); index++)
        if(template_id_list_[index] == msg->template_id)
            break;
    if(index < template_id_list_.size())
    {
        std::cout << "Updated!" << std::endl;
        pose_list_[index] = msg->pose;
    }
    this->publishTemplateList();
}

void TemplateNodelet::graspRequestCb(const flor_grasp_msgs::GraspSelection::ConstPtr& msg)
{
    std::cout << "Grasp request (id: " << msg->grasp_id << ")" << std::endl;
    {
    flor_grasp_msgs::GraspState cmd;

    //#typedef enum
    //#{
    //#    GRASP_MODE_NONE     = 0,
    //#    TEMPLATE_GRASP_MODE = 1,
    //#    MANUAL_GRASP_MODE   = 2,
    //#    NUM_GRASP_MODES
    //#
    //#} GraspControlModes;
    //#typedef enum
    //#{
    //#   GRASP_STATE_NONE   = 0, // unknown state
    //#    GRASP_INIT        = 1,
    //#    APPROACHING       = 2,
    //#    SURROUNDING       = 3,
    //#    GRASPING          = 4,
    //#    MONITORING        = 5,
    //#    OPENING           = 6,
    //#    GRASP_ERROR       = 7,
    //#    NUM_GRASP_STATES
    //#
    //#} GraspControlStates;

    unsigned char grasp_control_mode = 1;
    unsigned char grasp_control_state = 0;

    cmd.grasp_state.data = (grasp_control_mode <<4) + grasp_control_state;

    grasp_selected_state_pub_.publish(cmd);
    }
    {
    flor_grasp_msgs::GraspSelection cmd;

    cmd.template_id.data = msg->template_id.data;
    cmd.template_type.data = msg->template_type.data;
    cmd.grasp_id.data = msg->grasp_id.data;
    cmd.header.frame_id = "/pelvis";

    grasp_selected_pub_.publish(cmd);
    }
}

void TemplateNodelet::graspStateFeedbackCb(const flor_grasp_msgs::GraspState::ConstPtr& msg)
{
    std::cout << "Grasp feedback" << std::endl;
    std::cout << "Grasp control mode" << ((msg->grasp_state.data & 0xf0) >> 4) << std::endl;
    std::cout << "Grasp control state" << (msg->grasp_state.data & 0x0f) << std::endl;
}

void TemplateNodelet::templateMatchRequestCb(const flor_grasp_msgs::TemplateSelection::ConstPtr& msg)
{
    std::cout << "Template match request (id: " << (unsigned int)msg->template_id.data << ")" << std::endl;
    flor_grasp_msgs::TemplateSelection cmd;

    cmd.template_id.data = msg->template_id.data;
    cmd.template_type.data = msg->template_type.data;
    int index = 0;
    for(; index < template_id_list_.size(); index++)
        if(template_id_list_[index] == msg->template_id.data)
            break;
    cmd.pose = pose_list_[index];

    template_selected_pub_.publish(cmd);
}

void TemplateNodelet::templateMatchFeedbackCb(const flor_grasp_msgs::TemplateSelection::ConstPtr& msg)
{
    std::cout << "Template feedback" << std::endl;
    int index = 0;
    for(; index < template_id_list_.size(); index++)
        if(template_id_list_[index] == msg->template_id.data)
            break;
    pose_list_[index] = msg->pose;
    publishTemplateList();
}

void TemplateNodelet::publishTemplateList()
{
    flor_ocs_msgs::OCSTemplateList cmd;

    cmd.template_id_list = template_id_list_;
    cmd.template_list = template_list_;
    cmd.pose = pose_list_;

    // publish complete list of templates and poses
    template_list_pub_.publish( cmd );
}
}

PLUGINLIB_DECLARE_CLASS (vigir_ocs_template_nodelet, TemplateNodelet, ocs_template::TemplateNodelet, nodelet::Nodelet);
