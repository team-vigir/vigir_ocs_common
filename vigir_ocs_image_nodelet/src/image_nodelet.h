#include <ros/ros.h>
#include <ros/message_event.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <boost/thread.hpp>

#include <vector>
#include <string>

#include <tf/transform_broadcaster.h>

#include <flor_ocs_msgs/OCSImageAdd.h>
#include <flor_ocs_msgs/OCSImageList.h>

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
        void processImageListRequest( const std_msgs::Bool::ConstPtr& msg );
        void processImageSelected( const std_msgs::UInt64::ConstPtr& msg );

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
