#include <stdlib.h>
#include <stdio.h>
#include <dlfcn.h>
#include <unistd.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/date_time.hpp>
#include "ros/ros.h"
#include "ps_move_template_controller.h"
#include "moveclient.h"

vigir_ocs::PSMoveTemplateController* move_controller;
boost::mutex io_mutex;
bool ros_down = false;

// Prototypes
int updateSuccess(MoveServerPacket *);
int updateFailure(int);
int updateCameraSuccess(MoveServerCameraFrameSlicePacket *);
int updateCameraFailure(int);
void workerFunc();

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_controller_node");

    move_controller = new vigir_ocs::PSMoveTemplateController();

    boost::thread workerThread(workerFunc);

    while(ros::ok())
    {
        //hotkey_relay.onUpdate();
        ros::spinOnce();
        usleep(3000);
    }

    io_mutex.lock();
    ros_down = true;
    io_mutex.unlock();

    workerThread.join();

    delete move_controller;

    return 0;
}

void workerFunc()
{
    std::cout << "Worker: running" << std::endl;

    int res;

    MoveStateDeferred *move_state_deferred;
    MoveServerPacket *move_server_packet;
    MoveServerCameraFrameSlicePacket *move_server_camera_frame_slice_packet;

    move_state_deferred = (MoveStateDeferred *)malloc(sizeof(MoveStateDeferred));
    move_server_packet = (MoveServerPacket *)malloc(sizeof(MoveServerPacket));
    move_server_camera_frame_slice_packet = (MoveServerCameraFrameSlicePacket *)malloc(sizeof(MoveServerCameraFrameSlicePacket));

    move_state_deferred->update_success = updateSuccess;
    move_state_deferred->update_failure = updateFailure;
    move_state_deferred->update_camera_success = updateCameraSuccess;
    move_state_deferred->update_camera_failure = updateCameraFailure;
    move_state_deferred->move_server_packet = move_server_packet;
    move_state_deferred->move_server_camera_frame_slice_packet = move_server_camera_frame_slice_packet;

    // Connect
    res = serverConnect("192.168.1.14", "7899", move_state_deferred);
    ROS_ERROR("connect: %d", res);

    while(1)
    {
        io_mutex.lock();
        if(ros_down)
            break;
        io_mutex.unlock();

        boost::posix_time::millisec workTime(3);

        // Pretend to do something useful...
        boost::this_thread::sleep(workTime);
    }

    io_mutex.unlock();

    // Disconnect
    res = serverDisconnect();
    ROS_ERROR("disconnect: %d", res);

    // Free heap
    free(move_state_deferred);
    free(move_server_packet);
    free(move_server_camera_frame_slice_packet);

    std::cout << "Worker: finished" << std::endl;
}

int updateSuccess(MoveServerPacket *move_server_packet)
{
    //ROS_ERROR("update success");
    return move_controller->updateSuccess(move_server_packet);
}

int updateFailure(int error)
{
    //ROS_ERROR("update error: %d", error);
    return move_controller->updateFailure(error);
}

int updateCameraSuccess(MoveServerCameraFrameSlicePacket *move_server_camera_frame_slice_packet)
{
    //ROS_ERROR("camera success");
    return move_controller->updateCameraSuccess(move_server_camera_frame_slice_packet);
}

int updateCameraFailure(int error)
{
    //ROS_ERROR("camera failure: %d", error);
    return move_controller->updateCameraFailure(error);
}
