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
#ifndef ROBOTERRORCODES_H
#define ROBOTERRORCODES_H

#include <string>
#include <stdint.h>
#include <vector>

class RobotStatusCodes
{
public:
    typedef enum StatusCodes
    {
        NO_ERROR                                    =  0,
        ROBOT_STABILTY                              =  1, // Numbers subject to change for now
        GRASP_L_REACH_TIMEOUT                       =  2,
        GRASP_R_REACH_TIMEOUT                       =  3,
        GRASP_R_CLOSURE_FAILURE                     =  4,
        GRASP_L_CLOSURE_FAILURE                     =  5,
        GRASP_L_NO_OBJECT                           =  6,
        GRASP_R_NO_OBJECT                           =  7,
        GRASP_L_OBJECT_IN_PALM                      =  8,
        GRASP_R_OBJECT_IN_PALM                      =  9,
        GRASP_R_IK_FAILURE                          = 10,
        GRASP_L_IK_FAILURE                          = 11,
        GRASP_NO_APPENDAGE_CONTROL                  = 12,
        GRASP_L_NO_CONTROLLER                       = 13,
        GRASP_R_NO_CONTROLLER                       = 14,
        GRASP_CONTROLLER_OK                         = 15,
        GRASP_L_OBJECT_IN_FINGERS                   = 16,
        GRASP_R_OBJECT_IN_FINGERS                   = 17,
        // Comms bridge
        COMMS_BRIDGE_CONNECTION                     = 19,
        // Behavior statuses
        BEHAVIOR_FAILURE                            = 20,  // skipped to verify behavior of MAX_ERROR_MESSAGES and default loading
        BEHAVIOR_EXECUTION_FAILURE                  = 21,
        BEHAVIOR_ROBOT_FALL                         = 22,
        BEHAVIOR_ACTIVE                             = 23,
        BEHAVIOR_DONE                               = 24,
        BEHAVIOR_ERROR                              = 29,  // Bound range for behavior errors
        // Miscelaneous statues
        CONTROLLER_STAND_PREP                       = 30,
        PLANNER_GOAL_OCCUPIED                       = 31,
        PLANNER_START_OCCUPIED                      = 32,
        PLANNER_COLLISION_ERROR                     = 33,
        FOOTSTEP_PLANNER_GOAL_OCCUPIED              = 34,
        FOOTSTEP_PLANNER_START_OCCUPIED             = 35,
        FOOTSTEP_PLANNER_COLLISION_ERROR            = 36,
        // Controller statuses
        CONTROLLER_CLEAR_FOOTSTEP_PLAN              = 37,  // Clear any existing footstep plan from the controller
        CONTROLLER_POSTURE_CHANGE                   = 38,
        CONTROLLER_STATUS                           = 39,  // status
        CONTROLLER_ERROR                            = 40,  // Generic controller error
        CONTROLLER_INVALID_TRAJECTORY_TIMES         = 41,
        CONTROLLER_TRANSITIONING_THROUGH_BDI_STAND  = 42,
        CONTROLLER_TRANSITIONING_K_EFFORTS          = 43,
        CONTROLLER_INVALID_TRAJECTORY               = 44,
        CONTROLLER_INVALID_CONTROL_MODE             = 45,
        CONTROLLER_MAX_TIME                         = 46,
        CONTROLLER_NO_FOOTSTEP_PLAN                 = 47,
        CONTROLLER_INVALID_FOOTSTEP_PLAN            = 48,
        CONTROLLER_STOP                             = 49,
        // Planner statuses
        PLANNER_ERROR                               = 50,   // IK Planner codes
        PLANNER_IK_FAILED                           = 51,
        PLANNER_FAILED                              = 52,
        PLANNER_EXCEPTION                           = 53,
        PLANNER_UNKNOWN_FAILURE                     = 54,
        PLANNER_OCTOMAP_WARNING                     = 55,
        PLANNER_MOVEIT_PLAN_ACTIVE                  = 56,
        PLANNING_INVALID_REQUEST                    = 57,
        PLANNING_INVALID_START                      = 58,
        PLANNING_INVALID_GOAL                       = 59,
        PLANNING_INVALID_PLAN                       = 60,
        PLANNER_MSG_61                              = 61,
        PLANNER_MSG_62                              = 62,
        PLANNER_MSG_63                              = 63,
        // Robot Interface Codes
        ROBOT_CONNECTION                            = 101,
        ROBOT_CONTROL_ACTIVE                        = 102,
        ROBOT_STARTING                              = 103,
        ROBOT_STOPPING                              = 104,
        ROBOT_FAULT                                 = 105,
        ROBOT_COMMAND_SEQUENCE                      = 106,
        ROBOT_DATA                                  = 107,
        ROBOT_INTERFACE                             = 108,
        ROBOT_BEHAVIOR                              = 109,
        ROBOT_CONTROL_LOOP                          = 110,
        ROBOT_IMU_ALIGNMENT_FAULT                   = 111,
        ROBOT_STATUS_SOFT_FREEZE                    = 112,
        ROBOT_CRITICAL_FAULT_INTAKE_PRESSURE_LOW    = 113,
        ROBOT_CRITICAL_FAULT_MOTOR_OVERTEMP         = 114,
        ROBOT_CRITICAL_FAULT_IMU_FAILED             = 115,
        ROBOT_CRITICAL_FAULT_COMMS_TIMEOUT          = 116,
        ROBOT_CRITICAL_FAULT_AIR_PSI_LOW            = 117,
        ROBOT_CRITICAL_FAULT_PUMP_RPM_NOT_TRACKING  = 118,
        ROBOT_IMU_DRIFT                             = 119,
        // Robot behavior status
        ROBOT_BEHAVIOR_TRANSITION_IN_PROGRESS        = 120,
        ROBOT_BEHAVIOR_TRANSITION_SUCCESS            = 121,
        ROBOT_BEHAVIOR_FAILED_TRANS_ILLEGAL_BEHAVIOR = 122,
        ROBOT_BEHAVIOR_FAILED_TRANS_COM_POS          = 123,
        ROBOT_BEHAVIOR_FAILED_TRANS_COM_VEL          = 124,
        ROBOT_BEHAVIOR_FAILED_TRANS_VEL              = 125,
        ROBOT_BEHAVIOR_WARNING_AUTO_TRANS            = 126,
        ROBOT_BEHAVIOR_ERROR_FALLING                 = 127,
        ROBOT_BEHAVIOR_STAND                         = 128,
        ROBOT_BEHAVIOR_USER                          = 129,
        ROBOT_BEHAVIOR_FREEZE                        = 130,
        ROBOT_BEHAVIOR_STAND_PREP                    = 131,
        ROBOT_BEHAVIOR_WALK                          = 132,
        ROBOT_BEHAVIOR_STEP                          = 133,
        ROBOT_BEHAVIOR_MANIPULATE                    = 134,
        ROBOT_BEHAVIOR_WALK_SUBSTATE_CATCHING        = 135,
        ROBOT_BEHAVIOR_WALK_ERROR_INCONSISTENT_STEPS = 136,
        ROBOT_BEHAVIOR_WALK_WARNING_INSUFFICIENT_STEP_DATA= 137,
        ROBOT_BEHAVIOR_WALK_SUBSTATE_SWAYING         = 138,
        ROBOT_BEHAVIOR_WALK_SUBSTATE_STEPPING        = 139,
        ROBOT_BEHAVIOR_STEP_SUBSTATE_SWAYING         = 140,
        ROBOT_BEHAVIOR_STEP_SUBSTATE_STEPPING        = 141,
        // iRobot hand status
        RIGHT_FINGER_1_TEMPERATURE                   = 152,
        RIGHT_FINGER_2_TEMPERATURE                   = 153,
        RIGHT_FINGER_3_TEMPERATURE                   = 154,
        LEFT_FINGER_1_TEMPERATURE                    = 155,
        LEFT_FINGER_2_TEMPERATURE                    = 156,
        LEFT_FINGER_3_TEMPERATURE                    = 157,
        RIGHT_FINGER_1_NO_DATA                       = 158,
        RIGHT_FINGER_2_NO_DATA                       = 159,
        RIGHT_FINGER_3_NO_DATA                       = 160,
        LEFT_FINGER_1_NO_DATA                        = 161,
        LEFT_FINGER_2_NO_DATA                        = 162,
        LEFT_FINGER_3_NO_DATA                        = 163,
        RIGHT_FINGER_1_OK                            = 164,
        RIGHT_FINGER_2_OK                            = 165,
        RIGHT_FINGER_3_OK                            = 166,
        LEFT_FINGER_1_OK                             = 167,
        LEFT_FINGER_2_OK                             = 168,
        LEFT_FINGER_3_OK                             = 169,

        // Walk monitor status
        WALK_MONITOR_STATUS                          = 170, // feedback for walk monitor node health
        WALK_MONITOR_WALK_HEALTH                     = 171, // feedback for current walk health
        WALK_MONITOR_LOGGER                          = 172, // just for data logging
        // Footstep planner statuses
        FOOTSTEP_PLANNER_ACTIVE                      = 180,
        FOOTSTEP_PLANNER_SUCCESS                     = 181,
        FOOTSTEP_PLANNER_FAILED                      = 182,
        FOOTSTEP_PLANNER_GOAL_POSE_INACCESSIBLE      = 183,
        FOOTSTEP_PLANNER_START_POSE_INACCESSIBLE     = 184,
        FOOTSTEP_PLANNER_BODY_LEVEL_GRID_MAP         = 185,
        FOOTSTEP_PLANNER_GROUND_LEVEL_GRID_MAP       = 186,
        FOOTSTEP_PLANNER_TERRAIN_MODEL               = 187,
        FOOTSTEP_PLANNER_PLAN_OPTIMALITY             = 188,
        FOOTSTEP_PLANNER_MISC_FOOBAR                 = 189,
        FOOTSTEP_PLANNER_LARGE_HEIGHT_DIFF           = 195,

        // Motion services
        MOTION_SERVICE_UNKNOWN_MOTION                = 190,   // Motion Service Codes
        MOTION_SERVICE_INITIATE_MOTION               = 191,
        MOTION_SERVICE_MOTION_ERROR                  = 192,
        MOTION_SERVICE_MOTION_COMPLETE               = 193,

        // Robot Controller
        COMMS_BRIDGE_QUALITY                         = 200,
        CONTROLLER_INVALID_BEHAVIOR_TRANSITION       = 201,
        CONTROLLER_NON_SMOOTH_TRAJECTORY             = 301,
        MAX_ERROR_MESSAGES                           = 1000    // For uint16, this should be <= 16384
    } StatusCode;

    typedef enum StatusLevels
    {
        OK      = 0,
        DEBUG   = 1,
        WARNING = 2,
        ERROR   = 3
    } StatusLevel;

    static inline uint16_t status(const StatusCode& code, const StatusLevel& level)
    {
        return (( level << 14) + code);
    }

    static inline void codes(const uint16_t& status, uint16_t& code, uint8_t& level)
    {
        code  = (status & 0x3FFF);
        level = (status & 0xC000)>>14;
        return;
    }

    RobotStatusCodes();
    ~RobotStatusCodes() {};

    const std::string& str(const uint16_t& code)
    {
        if (code < messages_.size()) return messages_[code];

        return messages_[MAX_ERROR_MESSAGES]; // return value of "Invalid Error Code" initialized in constructor (to get rid of warning)
    }

    /// File to read in error messages, and store in string
    void loadErrorMessages(const std::string& filename);

private:
    std::vector<std::string> messages_;


};
#endif // ROBOTERRORCODES_H
