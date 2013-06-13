#include "image_nodelet.h"

namespace ocs_image
{
void ImageNodelet::onInit()
{
    ros::NodeHandle& nh         = getNodeHandle();
    ros::NodeHandle nh_out(nh, "image");

    // also create a publisher to set parameters of cropped image
    image_list_pub_         = nh_out.advertise<flor_ocs_msgs::OCSImageList>( "list", 1, false );
    image_selected_pub_     = nh_out.advertise<flor_grasp_msgs::ImageSelection>( "image_selected", 1, false );
    grasp_selected_pub_        = nh_out.advertise<flor_grasp_msgs::GraspSelection>( "grasp_selected", 1, false );
    grasp_selected_state_pub_  = nh_out.advertise<flor_grasp_msgs::GraspState>( "grasp_selected_state", 1, false );

    // then, subscribe to the resulting cropped image
    image_add_sub_            = nh_out.subscribe<flor_ocs_msgs::OCSImageAdd>( "add", 1, &ImageNodelet::addImageCb, this );
    image_remove_sub_         = nh_out.subscribe<flor_ocs_msgs::OCSImageRemove>( "remove", 1, &ImageNodelet::removeImageCb, this );
    image_update_sub_         = nh_out.subscribe<flor_ocs_msgs::OCSImageUpdate>( "update", 1, &ImageNodelet::updateImageCb, this );
    image_match_request_sub_  = nh_out.subscribe<flor_grasp_msgs::ImageSelection>( "image_match_request", 1, &ImageNodelet::imageMatchRequestCb, this );
    image_match_feedback_sub_ = nh_out.subscribe<flor_grasp_msgs::ImageSelection>( "image_match_feedback", 1, &ImageNodelet::imageMatchFeedbackCb, this );
    grasp_request_sub_           = nh_out.subscribe<flor_grasp_msgs::GraspSelection>( "grasp_request", 1, &ImageNodelet::graspRequestCb, this );
    grasp_state_feedback_sub_    = nh_out.subscribe<flor_grasp_msgs::GraspState>( "grasp_state_feedback", 1, &ImageNodelet::graspStateFeedbackCb, this );
    
    id_counter_ = 0;

    timer = nh_out.createTimer(ros::Duration(0.066), &ImageNodelet::timerCallback, this);
}

void ImageNodelet::timerCallback(const ros::TimerEvent& event)
{
    this->publishImageList();
}


void ImageNodelet::addImageCb(const flor_ocs_msgs::OCSImageAdd::ConstPtr& msg)
{
    std::cout << "Added image to list (id: " << (int)id_counter_ << ")" << std::endl;
    image_id_list_.push_back(id_counter_++);
    image_list_.push_back(msg->image_path);
    pose_list_.push_back(msg->pose);
    this->publishImageList();
}

void ImageNodelet::removeImageCb(const flor_ocs_msgs::OCSImageRemove::ConstPtr& msg)
{
    std::cout << "Removing image " << (unsigned int)msg->image_id << " from list... ";
    int index = 0;
    for(; index < image_id_list_.size(); index++)
        if(image_id_list_[index] == msg->image_id)
            break;
    if(index < image_id_list_.size())
    {
        std::cout << "Removed!" << std::endl;
        image_id_list_.erase(image_id_list_.begin()+index);
        image_list_.erase(image_list_.begin()+index);
        pose_list_.erase(pose_list_.begin()+index);
        this->publishImageList();
    }
}

void ImageNodelet::updateImageCb(const flor_ocs_msgs::OCSImageUpdate::ConstPtr& msg)
{
    std::cout << "Updating image " << (unsigned int)msg->image_id << "... ";
    int index = 0;
    for(; index < image_id_list_.size(); index++)
        if(image_id_list_[index] == msg->image_id)
            break;
    if(index < image_id_list_.size())
    {
        std::cout << "Updated!" << std::endl;
        pose_list_[index] = msg->pose;
    }
    this->publishImageList();
}

void ImageNodelet::graspRequestCb(const flor_grasp_msgs::GraspSelection::ConstPtr& msg)
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

    cmd.image_id.data = msg->image_id.data;
    cmd.image_type.data = msg->image_type.data;
    cmd.grasp_id.data = msg->grasp_id.data;
    cmd.header.frame_id = "/world";
    cmd.header.stamp = ros::Time::now();

    grasp_selected_pub_.publish(cmd);
    }
}

void ImageNodelet::graspStateFeedbackCb(const flor_grasp_msgs::GraspState::ConstPtr& msg)
{
    std::cout << "Grasp feedback" << std::endl;
    std::cout << "Grasp control mode" << ((msg->grasp_state.data & 0xf0) >> 4) << std::endl;
    std::cout << "Grasp control state" << (msg->grasp_state.data & 0x0f) << std::endl;
}

void ImageNodelet::imageMatchRequestCb(const flor_grasp_msgs::ImageSelection::ConstPtr& msg)
{
    std::cout << "Image match request (id: " << (unsigned int)msg->image_id.data << ")" << std::endl;
    flor_grasp_msgs::ImageSelection cmd;

    cmd.image_id.data = msg->image_id.data;
    cmd.image_type.data = msg->image_type.data;
    int index = 0;
    for(; index < image_id_list_.size(); index++)
        if(image_id_list_[index] == msg->image_id.data)
            break;
    cmd.pose = pose_list_[index];
    cmd.pose.header.stamp = ros::Time::now();

    image_selected_pub_.publish(cmd);
}

void ImageNodelet::imageMatchFeedbackCb(const flor_grasp_msgs::ImageSelection::ConstPtr& msg)
{
    std::cout << "Image feedback" << std::endl;
    int index = 0;
    for(; index < image_id_list_.size(); index++)
        if(image_id_list_[index] == msg->image_id.data)
            break;
    pose_list_[index] = msg->pose;
    publishImageList();
}

void ImageNodelet::publishImageList()
{
    //std::cout << "timer" << std::endl;
    flor_ocs_msgs::OCSImageList cmd;

    cmd.image_id_list = image_id_list_;
    cmd.image_list = image_list_;
    cmd.pose = pose_list_;

    // publish complete list of images and poses
    image_list_pub_.publish( cmd );

    // publish to TF
    ros::Time now = ros::Time::now();

    std::vector<tf::StampedTransform> transforms;

    for(int i = 0; i < pose_list_.size(); i++)
    {
        tf::StampedTransform image_pose_to_tf;

        image_pose_to_tf.frame_id_ = "/world";
        std::stringstream ss;
        ss << "/image_tf_" << (unsigned int)image_id_list_[i];
        image_pose_to_tf.child_frame_id_ = ss.str();

        image_pose_to_tf.stamp_ = now;

        const geometry_msgs::Point& vec_l (pose_list_[i].pose.position);
        image_pose_to_tf.setOrigin(tf::Vector3(vec_l.x, vec_l.y, vec_l.z));

        tf::Quaternion orientation;
        tf::quaternionMsgToTF(pose_list_[i].pose.orientation, orientation);

        image_pose_to_tf.setRotation(orientation);

        transforms.push_back(image_pose_to_tf);
    }

    if(transforms.size() > 0)
        tfb_.sendTransform(transforms);
}
}

PLUGINLIB_DECLARE_CLASS (vigir_ocs_image_nodelet, ImageNodelet, ocs_image::ImageNodelet, nodelet::Nodelet);
