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
@TODO_ADD_AUTHOR_INFO
#ifndef ROBOT_STATE_MANAGER_H
#define ROBOT_STATE_MANAGER_H

#include <vigir_ocs_robot_model/moveit_ocs_model.h>
//temp fix. need to fix so install will install the h files.?
//#include<flor_moveit_ocs_model/../../../src/flor_control/flor_moveit_ocs_model/include/flor_moveit_ocs_model/moveit_ocs_model.h>

//Singleton class to store state of robot to be conveniently referenced outside base3dview
class RobotStateManager{
   public:
       static RobotStateManager* Instance();
       MoveItOcsModel* getRobotStateSingleton();
       MoveItOcsModel* getGhostRobotStateSingleton();
 
   private:
       RobotStateManager();  // Private so that it can  not be called
       RobotStateManager(RobotStateManager const&){};             // copy constructor is private
       RobotStateManager& operator=(RobotStateManager const&){};  // assignment operator is private
       static RobotStateManager* instance;
       MoveItOcsModel * robot_state_;
       MoveItOcsModel * ghost_robot_state_;       
};

#endif
