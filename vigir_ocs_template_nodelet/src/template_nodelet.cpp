#include "template_nodelet.h"

namespace ocs_template
{
void TemplateNodelet::onInit()
{
    ros::NodeHandle& nh         = getNodeHandle();
    ros::NodeHandle nh_out(nh, "template");

    // also create a publisher to set parameters of cropped image
    template_list_pub_   = nh_out.advertise<vigir_ocs_msg2::OCSTemplateList>( "list", 1, false );
    // then, subscribe to the resulting cropped image
    template_add_sub_    = nh_out.subscribe<vigir_ocs_msg2::OCSTemplateAdd>( "add", 1, &TemplateNodelet::addTemplateCb, this );
    template_update_sub_ = nh_out.subscribe<vigir_ocs_msg2::OCSTemplateUpdate>( "update", 1, &TemplateNodelet::updateTemplateCb, this );
}

void TemplateNodelet::addTemplateCb(const vigir_ocs_msg2::OCSTemplateAdd::ConstPtr& msg)
{
    std::cout << "Adding template to list" << std::endl;
    template_list_.push_back(msg->template_path);
    pose_list_.push_back(msg->pose);
    this->publishTemplateList();
}

void TemplateNodelet::updateTemplateCb(const vigir_ocs_msg2::OCSTemplateUpdate::ConstPtr& msg)
{
    std::cout << "Updating template" << std::endl;
    if(msg->template_id >= 0 && msg->template_id < template_list_.size())
        pose_list_[msg->template_id] = msg->pose;
    this->publishTemplateList();
}

void TemplateNodelet::publishTemplateList()
{
    vigir_ocs_msg2::OCSTemplateList cmd;

    cmd.template_list = template_list_;
    cmd.pose = pose_list_;

    // publish complete list of templates and poses
    template_list_pub_.publish( cmd );
}
}

PLUGINLIB_DECLARE_CLASS (vigir_ocs_template_nodelet, TemplateNodelet, ocs_template::TemplateNodelet, nodelet::Nodelet);
