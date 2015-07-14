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
//@TODO_ADD_AUTHOR_INFO
#include "image_nodelet.h"

#include <iostream>
#include <boost/filesystem.hpp>

#include<string.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>


namespace ocs_image
{
void ImageNodelet::onInit()
{
    // initialize publishers for the image manager
    image_list_pub_ = nh_.advertise<vigir_ocs_msgs::OCSImageList>( "/flor/ocs/image_history/list", 1, false );
    image_added_pub_ = nh_.advertise<vigir_ocs_msgs::OCSImageAdd>( "/flor/ocs/image_history/add", 1, false );

    // initialize subscribers for image manager topics
    image_list_request_sub_ = nh_.subscribe( "/flor/ocs/image_history/list_request", 5, &ImageNodelet::processImageListRequest, this );
    image_selected_sub_ = nh_.subscribe( "/flor/ocs/image_history/select_image", 5, &ImageNodelet::processImageSelected, this );
    image_transport::ImageTransport it(nh_);
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
    vigir_ocs_msgs::OCSImageAdd msg;
    std::stringstream stream;

    msg.id = image_history_[id].id;
    msg.topic = image_history_[id].topic;
    // remove to store image on disk, instead add code to read from disk and then convert to thumbnail
    //msg.image = image_history_[id].image;
    msg.image.header.stamp=image_history_[id].header.stamp;
    msg.camera_info = image_history_[id].camera_info;
    cv::Mat image;
    image = cv::imread( "/home/vigir/image/"+stream.str()+".jpg", 1 );
    cv_bridge::CvImage out_msg;
    //out_msg.header   = in_msg->header; // Same timestamp and tf frame as input image
    out_msg.encoding = sensor_msgs::image_encodings::BGR8; // Or whatever
    out_msg.image    = image; // Your cv::Mat
    sensor_msgs::Image tmp_img;
    out_msg.toImageMsg(tmp_img);
    msg.image = tmp_img;
    /*double aspect_ratio = (double)image_history_[id].image.width/(double)image_history_[id].image.height;
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
    img_size.width=(double)image_history_[id].image.width;
    img_size.height=image_history_[id].image.height;

   /* if(aspect_ratio > 1)
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
    std::cout<<"Iamge size:"<<img_size.width;
    const char dir_path[] = "/home/vigir/image";
    boost::filesystem::path dir(dir_path);
    if(boost::filesystem::create_directory(dir))
      {
       std::cout << "Successfully created directory /home/vigir/image" << "\n";
      }
    else
        std::cout<<"\nFolder not created!!";
    imwrite("/home/vigir/image/"+stream.str()+".jpg",cv_ptr->image,qualitytype);*/
    image_added_pub_.publish(msg);
}

void ImageNodelet::publishImageList()
{
    vigir_ocs_msgs::OCSImageList msg;

    for(int i = 0; i < image_history_.size(); i++)
    {
        msg.id.push_back(image_history_[i].id);
        msg.topic.push_back(image_history_[i].topic);
        //msg.image.push_back(image_history_[i].image);
        msg.camera_info.push_back(image_history_[i].camera_info);
    }

    image_list_pub_.publish(msg);
}
// make change in this function to read image from disk and send to image manager
void ImageNodelet::publishImageToOCS(const unsigned long &id)
{
    if(id >= image_history_.size())
        return;

    //ROS_ERROR("(%ld) Sending to topic %s",id,(image_history_[id].topic+"/history/image_raw").c_str());
    //ROS_ERROR("(%ld) Sending to topic %s",id,(image_history_[id].topic+"/history/camera_info").c_str());

    // update timestamp and publish
    ros::Time now = ros::Time::now();

   // sensor_msgs::Image tmp_img = image_history_[id].image;

    std::stringstream stream;
    stream.str("");
    stream << id;

    cv::Mat image;
    image = cv::imread( "/home/vigir/image/"+stream.str()+".jpg", 1 );
    cv_bridge::CvImage out_msg;
    //out_msg.header   = in_msg->header; // Same timestamp and tf frame as input image
    out_msg.encoding = sensor_msgs::image_encodings::BGR8; // Or whatever
    out_msg.image    = image; // Your cv::Mat
    sensor_msgs::Image tmp_img;
    out_msg.toImageMsg(tmp_img);
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
    std::vector<int> qualitytype;
    qualitytype.push_back(CV_IMWRITE_JPEG_QUALITY);
    qualitytype.push_back(90);
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
        newimg.header.stamp = msg->header.stamp;
        //newimg.image = *msg;
        newimg.topic = topic;
        newimg.topic = newimg.topic.erase(newimg.topic.find("/image_raw"),strlen("/image_raw"));
        image_history_.push_back(newimg);
    }
    else
    {
        std::cout << "Adding Image to existing history image (" << image_history_[image_index].id << ")." << std::endl;
       // image_history_[image_index].image = *msg;
        image_history_[image_index].header.stamp = msg->header.stamp;
        publishImageAdded(image_index);
    }


        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
           // ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        cv::Size2i img_size;
        img_size.width=(double)msg->width;
        img_size.height=msg->height;
        cv::resize(cv_ptr->image, cv_ptr->image, img_size, 0, 0, cv::INTER_NEAREST);
        std::stringstream stream;
        stream.str("");
        stream << id_counter_-1;
        std::cout<<"Image size:"<<img_size.width;
        const char dir_path[] = "/home/vigir/image";
        boost::filesystem::path dir(dir_path);
        if(boost::filesystem::create_directory(dir))
          {
           std::cout << "Successfully created directory /home/vigir/image" << "\n";
          }
        else
            std::cout<<"\nFolder not created!!";
        imwrite("/home/vigir/image/"+stream.str()+".jpg",cv_ptr->image,qualitytype);
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
        if(msg->header.stamp == image_history_[image_index].header.stamp)
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

void ImageNodelet::processImageListRequest(const std_msgs::Bool::ConstPtr msg)
{
    //ROS_ERROR("Received image list request");

    publishImageList();
}

void ImageNodelet::processImageSelected(const std_msgs::UInt64::ConstPtr msg)
{
    //ROS_ERROR("Received image selected");

    publishImageToOCS(msg->data);
}
}

PLUGINLIB_DECLARE_CLASS (vigir_ocs_image_nodelet, ImageNodelet, ocs_image::ImageNodelet, nodelet::Nodelet);
