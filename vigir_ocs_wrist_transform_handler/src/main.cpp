#include <vigir_ocs_wrist_transform_handler/wrist_transform_handler.h>
#include <ros/ros.h>

int main(int argc, char *argv[])
{
    if( !ros::isInitialized() )
    {
        ros::init( argc, argv, "wrist_transform_handler");
    }

    vigir_ocs_wrist_transform_handler::WristTransformHandler handler;

    ros::spin();

    return 0;
}
