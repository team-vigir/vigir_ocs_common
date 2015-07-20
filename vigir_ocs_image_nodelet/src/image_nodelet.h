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
#include <ros/message_event.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <boost/thread.hpp>

#include <vector>
#include <string>

#include <tf/transform_broadcaster.h>

#include <vigir_ocs_msgs/OCSImageAdd.h>
#include <vigir_ocs_msgs/OCSImageList.h>

#include <std_msgs/Bool.h>
#include <std_msgs/UInt64.h>

#include <sensor_msgs/Image.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/CameraInfo.h>

namespace ocs_image
{
    class ImageNodelet : public nodelet::Nodelet
    {
      public:
        virtual void onInit();

        void processImage( const ros::MessageEvent<sensor_msgs::Image const>& event, const std::string& topic );
        void processCameraInfo( const ros::MessageEvent<sensor_msgs::CameraInfo const>& event, const std::string& topic );
        void processImageListRequest( const std_msgs::Bool::ConstPtr msg );
        void processImageSelected( const std_msgs::UInt64::ConstPtr msg );

        void publishImageList();
        void publishImageAdded( const unsigned long& id );
        void publishImageToOCS( const unsigned long& id );

      private:
        ros::NodeHandle nh_;

        ros::Publisher image_list_pub_;
        ros::Publisher image_added_pub_;
        std::map<std::string,ros::Publisher> image_topic_pub_list_;
        std::map<std::string,ros::Publisher> image_info_pub_list_;

        ros::Subscriber image_list_request_sub_;
        ros::Subscriber image_selected_sub_;
        std::map<std::string,ros::Subscriber> image_topic_sub_list_;
        std::map<std::string,ros::Subscriber> image_info_sub_list_;

        ros::Publisher image_topic_pub_;
        ros::Publisher image_info_pub_;

        typedef struct
        {
            unsigned long id;
            std::string topic;
            //sensor_msgs::Image image;
            std_msgs::Header header;
            sensor_msgs::CameraInfo camera_info;
        } ImageStruct;
        std::vector<ImageStruct> image_history_;

        unsigned long id_counter_;

        boost::mutex image_history_mutex_;
    };
}
