#include "image_nodelet.h"



#include<string.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


namespace ocs_image
{
void ImageNodelet::onInit()
{
    // initialize publishers for the image manager
    image_list_pub_ = nh_.advertise<flor_ocs_msgs::OCSImageList>( "/flor/ocs/image_history/list", 1, false );
    image_added_pub_ = nh_.advertise<flor_ocs_msgs::OCSImageAdd>( "/flor/ocs/image_history/add", 1, false );

    // initialize subscribers for image manager topics
    image_list_request_sub_ = nh_.subscribe<std_msgs::Bool>( "/flor/ocs/image_history/list_request", 5, &ImageNodelet::processImageListRequest, this );
    image_selected_sub_ = nh_.subscribe<std_msgs::UInt64>( "/flor/ocs/image_history/select_image", 5, &ImageNodelet::processImageSelected, this );

    // should probably read this from file, but vrc hacking mode now
    std::vector<std::string> topics;
    topics.push_back("/l_image_full");
    topics.push_back("/l_image_cropped");
    topics.push_back("/r_image_full");
    topics.push_back("/r_image_cropped");
    topics.push_back("/lhl_image_full");
    topics.push_back("/lhl_image_cropped");
    topics.push_back("/lhr_image_full");
    topics.push_back("/lhr_image_cropped");
    topics.push_back("/rhl_image_full");
    topics.push_back("/rhl_image_cropped");
    topics.push_back("/rhr_image_full");
    topics.push_back("/rhr_image_cropped");

    for(int i = 0; i < topics.size(); i++)
    {
        // initialize image topic publishers
        image_topic_pub_list_[topics[i]+"/history/image_raw"]  = nh_.advertise<sensor_msgs::Image>( topics[i]+"/history/image_raw", 1, false );
        // initialize camerainfo topic publishers
        image_info_pub_list_[topics[i]+"/history/camera_info"] = nh_.advertise<sensor_msgs::CameraInfo>( topics[i]+"/history/camera_info", 1, false );
        // initialize subscribers to the image topics coming from robot
        image_topic_sub_list_[topics[i]+"/image_raw"]  = nh_.subscribe<sensor_msgs::Image>( topics[i]+"/image_raw", 100, boost::bind(&ImageNodelet::processImage, this, _1, topics[i]+"/image_raw"));
        // initialize subscribers to the camerainfo topics coming from robot
        image_info_sub_list_[topics[i]+"/camera_info"] = nh_.subscribe<sensor_msgs::CameraInfo>( topics[i]+"/camera_info", 100, boost::bind(&ImageNodelet::processCameraInfo, this, _1, topics[i]+"/camera_info"));
    }

    // this are the extra topic that publishes all images
    image_topic_pub_ = nh_.advertise<sensor_msgs::Image>( "/flor/ocs/history/image_raw", 1, false );
    image_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>( "/flor/ocs/history/camera_info", 1, false );

    id_counter_ = 0;

}

void ImageNodelet::publishImageAdded(const unsigned long &id)
{
    flor_ocs_msgs::OCSImageAdd msg;
    std::vector<int> qualitytype;
    qualitytype.push_back(CV_IMWRITE_JPEG_QUALITY);
    qualitytype.push_back(90);
    msg.id = image_history_[id].id;
    msg.topic = image_history_[id].topic;
    msg.image = image_history_[id].image;

    msg.camera_info = image_history_[id].camera_info;
    double aspect_ratio = (double)image_history_[id].image.width/(double)image_history_[id].image.height;
   // ROS_ERROR("Size: %dx%d aspect %f", image.width, image.height, aspect_ratio);

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(image_history_[id].image, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
       // ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Size2i img_size;
    if(aspect_ratio > 1)
    {
        img_size.width = 50.0f;
        img_size.height = 50.0f/aspect_ratio;
    }
    else
    {
        img_size.width = 50.0f/aspect_ratio;
        img_size.height = 50.0f;

    }
    cv::resize(cv_ptr->image, cv_ptr->image, img_size, 0, 0, cv::INTER_NEAREST);

    std::stringstream stream;

    stream.str("");


    stream << image_history_[id].id ;

    //string filename = "/home/vigir/image"+(image_history_[id].topic)+".jpg";
    imwrite("/home/vigir/image/"+stream.str()+".jpg",cv_ptr->image,qualitytype);
   // QImage tmp = Mat2QImage(cv_ptr->image);
   // QPixmap pixmap = QPixmap::fromImage(tmp);
   // tmp.save("/home/vigir/image/"+QString::number(image_history_[id].id),"BMP");

    image_added_pub_.publish(msg);
}

void ImageNodelet::publishImageList()
{
    flor_ocs_msgs::OCSImageList msg;

    for(int i = 0; i < image_history_.size(); i++)
    {
        msg.id.push_back(image_history_[i].id);
        msg.topic.push_back(image_history_[i].topic);
        msg.image.push_back(image_history_[i].image);
        msg.camera_info.push_back(image_history_[i].camera_info);
    }

    image_list_pub_.publish(msg);
}

void ImageNodelet::publishImageToOCS(const unsigned long &id)
{
    if(id >= image_history_.size())
        return;

    //ROS_ERROR("(%ld) Sending to topic %s",id,(image_history_[id].topic+"/history/image_raw").c_str());
    //ROS_ERROR("(%ld) Sending to topic %s",id,(image_history_[id].topic+"/history/camera_info").c_str());

    // update timestamp and publish
    ros::Time now = ros::Time::now();

    sensor_msgs::Image tmp_img = image_history_[id].image;
    tmp_img.header.stamp = now;
    image_topic_pub_list_[image_history_[id].topic+"/history/image_raw"].publish(tmp_img);
    image_topic_pub_.publish(tmp_img);

    sensor_msgs::CameraInfo tmp_info = image_history_[id].camera_info;
    tmp_info.header.stamp = now;
    image_info_pub_list_[image_history_[id].topic+"/history/camera_info"].publish(tmp_info);
    image_info_pub_.publish(tmp_info);
}

void ImageNodelet::processImage( const ros::MessageEvent<sensor_msgs::Image const>& event, const std::string& topic )
{
    boost::mutex::scoped_lock lock(image_history_mutex_);

    const std::string& publisher_name = event.getPublisherName();

    // publishing on my own topic; return
    if(publisher_name == this->getName())
        return;

    //ROS_ERROR("Received image from %s on topic %s",publisher_name.c_str(),topic.c_str());

    const sensor_msgs::Image::ConstPtr& msg = event.getMessage();

    unsigned int image_index;
    for(image_index = 0; image_index < image_history_.size(); image_index++)
        if(msg->header.stamp == image_history_[image_index].camera_info.header.stamp)
            break;

    // if it doesn't exist
    if(image_index == image_history_.size())
    {
        std::cout << "Adding new image to history (" << id_counter_ << "); using Image." << std::endl;
        ImageStruct newimg;
        newimg.id = id_counter_++;
        newimg.image = *msg;
        newimg.topic = topic;
        newimg.topic = newimg.topic.erase(newimg.topic.find("/image_raw"),strlen("/image_raw"));
        image_history_.push_back(newimg);
    }
    else
    {
        std::cout << "Adding Image to existing history image (" << image_history_[image_index].id << ")." << std::endl;
        image_history_[image_index].image = *msg;
        publishImageAdded(image_index);
    }
}

void ImageNodelet::processCameraInfo( const ros::MessageEvent<sensor_msgs::CameraInfo const>& event, const std::string& topic )
{
    boost::mutex::scoped_lock lock(image_history_mutex_);

    const std::string& publisher_name = event.getPublisherName();

    // publishing on my own topic; return
    if(publisher_name == this->getName())
        return;

    //ROS_ERROR("Received camera info from %s on topic %s",publisher_name.c_str(),topic.c_str());

    const sensor_msgs::CameraInfo::ConstPtr& msg = event.getMessage();

    unsigned int image_index;
    for(image_index = 0; image_index < image_history_.size(); image_index++)
        if(msg->header.stamp == image_history_[image_index].image.header.stamp)
            break;

    // if it doesn't exist
    if(image_index == image_history_.size())
    {
        std::cout << "Adding new image to history (" << id_counter_ << "); using CameraInfo." << std::endl;
        ImageStruct newimg;
        newimg.id = id_counter_++;
        newimg.camera_info = *msg;
        newimg.topic = topic;
        newimg.topic = newimg.topic.erase(newimg.topic.find("/camera_info"),strlen("/camera_info"));
        image_history_.push_back(newimg);
    }
    else
    {
        std::cout << "Adding CameraInfo to existing history image (" << image_history_[image_index].id << ")." << std::endl;
        image_history_[image_index].camera_info = *msg;
        publishImageAdded(image_index);
    }
}

void ImageNodelet::processImageListRequest(const std_msgs::Bool::ConstPtr &msg)
{
    //ROS_ERROR("Received image list request");

    publishImageList();
}

void ImageNodelet::processImageSelected(const std_msgs::UInt64::ConstPtr &msg)
{
    //ROS_ERROR("Received image selected");

    publishImageToOCS(msg->data);
}
}

PLUGINLIB_DECLARE_CLASS (vigir_ocs_image_nodelet, ImageNodelet, ocs_image::ImageNodelet, nodelet::Nodelet);
