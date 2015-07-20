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
