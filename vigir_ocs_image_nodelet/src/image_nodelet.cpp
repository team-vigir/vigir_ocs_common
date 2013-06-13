#include "image_nodelet.h"

namespace ocs_image
{
void ImageNodelet::onInit()
{
    // initialize publishers for the image manager
    image_list_pub_ = nh_.advertise<flor_ocs_msgs::OCSImageList>( "/flor/ocs/image_history/list", 1, false );
    image_added_pub_ = nh_.advertise<flor_ocs_msgs::OCSImageAdd>( "/flor/ocs/image_history/add", 1, false );

    // initialize image topic publishers
    image_topic_pub_list_["/l_image_full/image_raw"]      = nh_.advertise<sensor_msgs::Image>( "/l_image_full/image_raw", 1, false );
    image_topic_pub_list_["/l_image_cropped/image_raw"]   = nh_.advertise<sensor_msgs::Image>( "/l_image_cropped/image_raw", 1, false );
    image_topic_pub_list_["/r_image_full/image_raw"]      = nh_.advertise<sensor_msgs::Image>( "/r_image_full/image_raw", 1, false );
    image_topic_pub_list_["/r_image_cropped/image_raw"]   = nh_.advertise<sensor_msgs::Image>( "/r_image_cropped/image_raw", 1, false );
    image_topic_pub_list_["/lhl_image_full/image_raw"]    = nh_.advertise<sensor_msgs::Image>( "/lhl_image_full/image_raw", 1, false );
    image_topic_pub_list_["/lhl_image_cropped/image_raw"] = nh_.advertise<sensor_msgs::Image>( "/lhl_image_cropped/image_raw", 1, false );
    image_topic_pub_list_["/lhr_image_full/image_raw"]    = nh_.advertise<sensor_msgs::Image>( "/lhr_image_full/image_raw", 1, false );
    image_topic_pub_list_["/lhr_image_cropped/image_raw"] = nh_.advertise<sensor_msgs::Image>( "/lhr_image_cropped/image_raw", 1, false );
    image_topic_pub_list_["/rhl_image_full/image_raw"]    = nh_.advertise<sensor_msgs::Image>( "/rhl_image_full/image_raw", 1, false );
    image_topic_pub_list_["/rhl_image_cropped/image_raw"] = nh_.advertise<sensor_msgs::Image>( "/rhl_image_cropped/image_raw", 1, false );
    image_topic_pub_list_["/rhr_image_full/image_raw"]    = nh_.advertise<sensor_msgs::Image>( "/rhr_image_full/image_raw", 1, false );
    image_topic_pub_list_["/rhr_image_cropped/image_raw"] = nh_.advertise<sensor_msgs::Image>( "/rhr_image_cropped/image_raw", 1, false );

    // and the camera info
    image_info_pub_list_["/l_image_full/image_raw/camera_info"]      = nh_.advertise<sensor_msgs::CameraInfo>( "/l_image_full/image_raw/camera_info", 1, false );
    image_info_pub_list_["/l_image_cropped/image_raw/camera_info"]   = nh_.advertise<sensor_msgs::CameraInfo>( "/l_image_cropped/image_raw/camera_info", 1, false );
    image_info_pub_list_["/r_image_full/image_raw/camera_info"]      = nh_.advertise<sensor_msgs::CameraInfo>( "/r_image_full/image_raw/camera_info", 1, false );
    image_info_pub_list_["/r_image_cropped/image_raw/camera_info"]   = nh_.advertise<sensor_msgs::CameraInfo>( "/r_image_cropped/image_raw/camera_info", 1, false );
    image_info_pub_list_["/lhl_image_full/image_raw/camera_info"]    = nh_.advertise<sensor_msgs::CameraInfo>( "/lhl_image_full/image_raw/camera_info", 1, false );
    image_info_pub_list_["/lhl_image_cropped/image_raw/camera_info"] = nh_.advertise<sensor_msgs::CameraInfo>( "/lhl_image_cropped/image_raw/camera_info", 1, false );
    image_info_pub_list_["/lhr_image_full/image_raw/camera_info"]    = nh_.advertise<sensor_msgs::CameraInfo>( "/lhr_image_full/image_raw/camera_info", 1, false );
    image_info_pub_list_["/lhr_image_cropped/image_raw/camera_info"] = nh_.advertise<sensor_msgs::CameraInfo>( "/lhr_image_cropped/image_raw/camera_info", 1, false );
    image_info_pub_list_["/rhl_image_full/image_raw/camera_info"]    = nh_.advertise<sensor_msgs::CameraInfo>( "/rhl_image_full/image_raw/camera_info", 1, false );
    image_info_pub_list_["/rhl_image_cropped/image_raw/camera_info"] = nh_.advertise<sensor_msgs::CameraInfo>( "/rhl_image_cropped/image_raw/camera_info", 1, false );
    image_info_pub_list_["/rhr_image_full/image_raw/camera_info"]    = nh_.advertise<sensor_msgs::CameraInfo>( "/rhr_image_full/image_raw/camera_info", 1, false );
    image_info_pub_list_["/rhr_image_cropped/image_raw/camera_info"] = nh_.advertise<sensor_msgs::CameraInfo>( "/rhr_image_cropped/image_raw/camera_info", 1, false );

    // initialize subscribers for image manager topics
    ros::Subscriber image_list_request_sub_ = nh_.subscribe<std_msgs::Bool>( "/flor/ocs/image_history/list_request", 1, &ImageNodelet::processImageListRequest, this );
    ros::Subscriber image_selected_sub_ = nh_.subscribe<std_msgs::UInt64>( "/flor/ocs/image_history/select_image", 1, &ImageNodelet::processImageSelected, this );

    // initialize subscribers to the image topics coming from robot
    image_topic_sub_list_["/l_image_full/image_raw"]      = nh_.subscribe( "/l_image_full/image_raw", 1, &ImageNodelet::processImage, this );
    image_topic_sub_list_["/l_image_cropped/image_raw"]   = nh_.subscribe( "/l_image_cropped/image_raw", 1, &ImageNodelet::processImage, this );
    image_topic_sub_list_["/r_image_full/image_raw"]      = nh_.subscribe( "/r_image_full/image_raw", 1, &ImageNodelet::processImage, this );
    image_topic_sub_list_["/r_image_cropped/image_raw"]   = nh_.subscribe( "/r_image_cropped/image_raw", 1, &ImageNodelet::processImage, this );
    image_topic_sub_list_["/lhl_image_full/image_raw"]    = nh_.subscribe( "/lhl_image_full/image_raw", 1, &ImageNodelet::processImage, this );
    image_topic_sub_list_["/lhl_image_cropped/image_raw"] = nh_.subscribe( "/lhl_image_cropped/image_raw", 1, &ImageNodelet::processImage, this );
    image_topic_sub_list_["/lhr_image_full/image_raw"]    = nh_.subscribe( "/lhr_image_full/image_raw", 1, &ImageNodelet::processImage, this );
    image_topic_sub_list_["/lhr_image_cropped/image_raw"] = nh_.subscribe( "/lhr_image_cropped/image_raw", 1, &ImageNodelet::processImage, this );
    image_topic_sub_list_["/rhl_image_full/image_raw"]    = nh_.subscribe( "/rhl_image_full/image_raw", 1, &ImageNodelet::processImage, this );
    image_topic_sub_list_["/rhl_image_cropped/image_raw"] = nh_.subscribe( "/rhl_image_cropped/image_raw", 1, &ImageNodelet::processImage, this );
    image_topic_sub_list_["/rhr_image_full/image_raw"]    = nh_.subscribe( "/rhr_image_full/image_raw", 1, &ImageNodelet::processImage, this );
    image_topic_sub_list_["/rhr_image_cropped/image_raw"] = nh_.subscribe( "/rhr_image_cropped/image_raw", 1, &ImageNodelet::processImage, this );

    // initialize subscribers to the camerainfo topics coming from robot
    image_info_sub_list_["/l_image_full/image_raw/camera_info"]      = nh_.subscribe( "/l_image_full/image_raw/camera_info", 1, &ImageNodelet::processCameraInfo, this );
    image_info_sub_list_["/l_image_cropped/image_raw/camera_info"]   = nh_.subscribe( "/l_image_cropped/image_raw/camera_info", 1, &ImageNodelet::processCameraInfo, this );
    image_info_sub_list_["/r_image_full/image_raw/camera_info"]      = nh_.subscribe( "/r_image_full/image_raw/camera_info", 1, &ImageNodelet::processCameraInfo, this );
    image_info_sub_list_["/r_image_cropped/image_raw/camera_info"]   = nh_.subscribe( "/r_image_cropped/image_raw/camera_info", 1, &ImageNodelet::processCameraInfo, this );
    image_info_sub_list_["/lhl_image_full/image_raw/camera_info"]    = nh_.subscribe( "/lhl_image_full/image_raw/camera_info", 1, &ImageNodelet::processCameraInfo, this );
    image_info_sub_list_["/lhl_image_cropped/image_raw/camera_info"] = nh_.subscribe( "/lhl_image_cropped/image_raw/camera_info", 1, &ImageNodelet::processCameraInfo, this );
    image_info_sub_list_["/lhr_image_full/image_raw/camera_info"]    = nh_.subscribe( "/lhr_image_full/image_raw/camera_info", 1, &ImageNodelet::processCameraInfo, this );
    image_info_sub_list_["/lhr_image_cropped/image_raw/camera_info"] = nh_.subscribe( "/lhr_image_cropped/image_raw/camera_info", 1, &ImageNodelet::processCameraInfo, this );
    image_info_sub_list_["/rhl_image_full/image_raw/camera_info"]    = nh_.subscribe( "/rhl_image_full/image_raw/camera_info", 1, &ImageNodelet::processCameraInfo, this );
    image_info_sub_list_["/rhl_image_cropped/image_raw/camera_info"] = nh_.subscribe( "/rhl_image_cropped/image_raw/camera_info", 1, &ImageNodelet::processCameraInfo, this );
    image_info_sub_list_["/rhr_image_full/image_raw/camera_info"]    = nh_.subscribe( "/rhr_image_full/image_raw/camera_info", 1, &ImageNodelet::processCameraInfo, this );
    image_info_sub_list_["/rhr_image_cropped/image_raw/camera_info"] = nh_.subscribe( "/rhr_image_cropped/image_raw/camera_info", 1, &ImageNodelet::processCameraInfo, this );

    id_counter_ = 0;

}

void ImageNodelet::publishImageAdded()
{

}

void ImageNodelet::publishImageList()
{

}

void ImageNodelet::publishImageToOCS(const unsigned long &id)
{
    image_topic_pub_list_[image_history_[id].topic].publish(image_history_[id].image);
    image_info_pub_list_[image_history_[id].topic].publish(image_history_[id].camera_info);
}

void ImageNodelet::processImage(const ros::MessageEvent<sensor_msgs::Image const>& event)
{
    std::string publisher_name = event.getPublisherName();
    ros::M_string connection_header = event.getConnectionHeader();
    std::string topic = connection_header["topic"];

    ROS_ERROR("Received image from %s from %s",topic.c_str(),publisher_name.c_str());
}

void ImageNodelet::processCameraInfo(const ros::MessageEvent<sensor_msgs::CameraInfo const>& event)
{
    std::string publisher_name = event.getPublisherName();
    ros::M_string connection_header = event.getConnectionHeader();
    std::string topic = connection_header["topic"];

    ROS_ERROR("Received image from %s from %s",topic.c_str(),publisher_name.c_str());
}

void ImageNodelet::processImageListRequest(const std_msgs::Bool::ConstPtr &msg)
{
    ROS_ERROR("Received image list request");

    //publishImageList();
}

void ImageNodelet::processImageSelected(const std_msgs::UInt64::ConstPtr &msg)
{
    ROS_ERROR("Received image selected");

    //publishImageToOCS(msg->data);
}


}

PLUGINLIB_DECLARE_CLASS (vigir_ocs_image_nodelet, ImageNodelet, ocs_image::ImageNodelet, nodelet::Nodelet);
