
#ifndef __AtlasInterfaceTypes_H
#define __AtlasInterfaceTypes_H

#include "AtlasControlTypes.h"
#include "AtlasVectorTypes.h"

#include <stdio.h>
#include <string.h>

#if __GNUC__ >= 4
#pragma GCC visibility push(default)
#endif

#define ATLAS_SOFTWARE_VERSION_STRING "2.5.0"
#define ATLAS_SOFTWARE_VERSION_MAJOR 2
#define ATLAS_SOFTWARE_VERSION_MINOR 5
#define ATLAS_SOFTWARE_VERSION_POINT 0

#ifdef __GNUC__
#pragma pack(push,1)
#endif

using namespace Atlas;

#ifndef NOT_IN_DOXYGEN
namespace AtlasRobot {
#endif


//////////////////////////////////////////////////////////////////////////////
//!
//! \name Robot Behaviors
//!
///@{

typedef enum
{
	BEHAVIOR_NONE,        //!< Robot currently not in any behavior; software freeze, controls disabled.
	BEHAVIOR_FREEZE,      //!< All joints frozen in current position.
	BEHAVIOR_STAND_PREP,  //!< Robot assumes a balanced posture, preparing for stand.
	BEHAVIOR_STAND,       //!< Statically stable stand.
	BEHAVIOR_WALK,        //!< Dynamically stable walk.
	BEHAVIOR_STEP,        //!< Statically stable, single-step walk.
	BEHAVIOR_MANIPULATE,  //!< Statically stable stand, with upper body joints controlled by user.
	BEHAVIOR_USER,        //!< Setpoints and gains set by user.
	NUM_BEHAVIORS

} AtlasRobotBehavior;



///@}
//////////////////////////////////////////////////////////////////////////////
//!
//! \name To-robot types and enumerations.
//!
///@{

///////////////////////////////////////////////////////////
//!
//!  \struct  AtlasJointDesired
//!
//!  \brief   Structure for specifying joint setpoints
//!
struct AtlasJointDesired
{
	float q_d;            //!< Desired position of joint, in radians.
	float qd_d;           //!< Desired velocity of joint, in radians / s.
	float f_d;            //!< Desired torque of joint, in N*m.

	//! \brief  Default contructor.  All data members set to 0.
	AtlasJointDesired() :
		q_d(0.0f),
		qd_d(0.0f),
		f_d(0.0f)
	{}
};


///////////////////////////////////////////////////////////
//!
//!  \struct  AtlasJointControlParams
//!
//!  \brief   Structure for specifying controller parameters
//!
//!    This structure contains the gains to be applied to a joint.
//!
//!    q, qd, f are sensed position, velocity, torque, from AtlasJointState
//!
//!    q_d, qd_d, f_d are desired position, velocity, torque, from
//!    AtlasJointDesired
//!
//!    The final joint command will be:
//!
//!        k_q_p     * ( q_d - q )       +
//!        k_q_i     * 1/s * ( q_d - q ) +
//!        k_qd_p    * ( qd_d - qd )     +
//!        k_f_p     * ( f_d - f )       +
//!        ff_qd     * qd                +
//!        ff_qd_d   * qd_d              +
//!        ff_f_d    * f_d               +
//!        ff_const
//!
struct AtlasJointControlParams
{
	float k_q_p;      //!< Position error gain, in N*m/rad.
	float k_q_i;      //!< Integral of position error gain, in N*m/(rad*s).
	float k_qd_p;     //!< Derivative error gain, in N*m/(rad/s).

	float k_f_p;      //!< Proportional force feedback gain.
	float ff_qd;      //!< Feedforward velocity gain.
	float ff_qd_d;    //!< Feedforward desired velocity gain.
	float ff_f_d;     //!< Feedforward desired force gain.
	float ff_const;   //!< Constant force term.

	//! \brief  Default constructor.  All data members set to 0.
	AtlasJointControlParams() :
		k_q_p(0.0f),
		k_q_i(0.0f),
		k_qd_p(0.0f),
		k_f_p(0.0f),
		ff_qd(0.0f),
		ff_qd_d(0.0f),
		ff_f_d(0.0f),
		ff_const(0.0f)
	{}
};


///////////////////////////////////////////////////////////
//!
//!  \struct  AtlasControlDataToRobot
//!
//!  \brief   Structure for control output data to be streamed to robot sim.
//!
//!    This structure is for holding data that is to be streamed to the robot
//!    at a high rate.  Most of this data will be parameters for actuator
//!    control.
//!
struct AtlasControlDataToRobot
{
	// timestamp should remain the first data member
	uint64_t timestamp;                                   //!< Pass milliseconds since unix epoch.

	AtlasJointDesired       j[Atlas::NUM_JOINTS];         //!< Setpoints for robot joints.
	AtlasJointControlParams jparams[Atlas::NUM_JOINTS];   //!< Joint gains, etc. for robot joints.

	/*
	 *  Behavior params.
	 */
	AtlasBehaviorStandParams      stand_params;           //!< Input parameters for Stand behavior
	AtlasBehaviorStepParams       step_params;            //!< Input parameters for Step behavior
	AtlasBehaviorWalkParams       walk_params;            //!< Input parameters for Walk behavior
	AtlasBehaviorManipulateParams manipulate_params;      //!< Input parameters for Manipulate behavior

	//! \brief  Default contructor.  All data members set to 0.
	AtlasControlDataToRobot() :
		timestamp(0l)
	{}

	AtlasControlDataToRobot(const AtlasControlDataToRobot& rhs)
	{
		memcpy((uint8_t*) this,
			(uint8_t*) &rhs,
			sizeof(AtlasControlDataToRobot));
	}

	AtlasControlDataToRobot& operator= (const AtlasControlDataToRobot& rhs)
	{
		memcpy((uint8_t*) this,
			(uint8_t*) &rhs,
			sizeof(AtlasControlDataToRobot));
		return *this;
	}
};


///@}
//////////////////////////////////////////////////////////////////////////////
//!
//! \name From-robot types and enumerations.
//!
///@{


///////////////////////////////////////////////////////////
//!
//!  \struct  AtlasJointState
//!
//!  \brief   Structure for holding robot joint state.
//!
struct AtlasJointState
{
	float q;       //!< Measured orientation or position of joint, in radians.
	float qd;      //!< Measured velocity of joint, in radians/s.
	float f;       //!< Measured torque of joint in N*m.

	//! \brief  Default contructor.  All data members set to 0.
	AtlasJointState() :
		q(0.0f),
		qd(0.0f),
		f(0.0f)
	{}
};


///////////////////////////////////////////////////////////
//!
//!  \enum    AtlasRobotRunState
//!
//!  \brief   Enumerations identifying run states of the robot.
//!
//!    The robot will be in only one run state at any one time.  It can
//!    only enter states under certain conditions.
//!
typedef enum
{
	//!
	//!  Initial state.
	//!
	//!  Entries:
	//!
	//!    \li  from RUN_STATE_STOP
	//!    \li  from RUN_STATE_START
	//!
	//!  Exits:
	//!
	//!    \li  to RUN_STATE_START when function start() called
	//!
	RUN_STATE_IDLE = 0,

	//!
	//!  Entries:
	//!
	//!    \li  from RUN_STATE_IDLE
	//!
	//!  Exits:
	//!
	//!    \li  to RUN_STATE_CONTROL automatically if pump start completes or pump not used
	//!    \li  to RUN_STATE_STOP automatically if pump start fails
	//!
	RUN_STATE_START = 1,

	//!
	//!  Entries:
	//!
	//!    \li  from RUN_STATE_START
	//!
	//!  Exits:
	//!
	//!    \li  to RUN_STATE_STOP if stop() called
	//!    \li  to RUN_STATE_STOP on critical fault
	//!
	RUN_STATE_CONTROL = 3,

	//!
	//!  Entries:
	//!
	//!    \li  from RUN_STATE_CONTROL
	//!    \li  from critical fault
	//!
	//!  Exits:
	//!
	//!    \li  to RUN_STATE_IDLE automatically
	//!
	RUN_STATE_STOP = 5,

} AtlasRobotRunState;


///////////////////////////////////////////////////////////
//!
//!  \enum    AtlasRobotStatus
//!
//!  \brief   Enumerations identifying various status states of the robot.
//!
//!    The robot may have more than one status at any given time.  The values
//!    represent bitwise flags.
//!
//!    A flag is considered "high" if the bit-wise and operation against that
//!    flag is non-0.  e.g., the flag STATUS_PUMP_STARTING is high if:
//!
//!      current_status_flags & STATUS_PUMP_STARTING != 0
//!
//!    It is important that FAULT flags are checked often.  If a
//!    CRITICAL_FAULT flag is raised, the robot will shut down automatically.
//!
typedef enum
{
	//!
	//!  \brief  low -> robot idle; high -> robot active
	//!
	STATUS_RUNNING = 0,

	//!
	//!  \brief  high -> hydraulic pump is starting
	//!
	STATUS_PUMP_STARTING = 1 << 1,

	//!
	//!  \brief  high -> during stop state, robot using soft freeze settings
	//!
	//!    <em> Not implemented as of 2.0.3. </em>  Robot will freeze when
	//!    entering Stop state from all behaviors.
	//!
	STATUS_SOFT_FREEZE = 1 << 2,

	//!
	//!  \brief  high -> not enough pump intake pressure
	//!
	CRITICAL_FAULT_INTAKE_PRESSURE_LOW = 1 << 8,

	//!
	//!  \brief  high -> motor temp too high
	//!
	CRITICAL_FAULT_MOTOR_OVERTEMP = 1 << 9,

	//!
	//!  \brief  high -> failed communication with IMU
	//!
	CRITICAL_FAULT_IMU_FAILED = 1 << 10,

	//!
	//!  \brief  high -> too long without comms between robot and fc
	//!
	CRITICAL_FAULT_COMMS_TIMEOUT = 1 << 12,

	//!
	//!  \brief  high -> pump air pressure too low
	//!
	CRITICAL_FAULT_AIR_PSI_LOW = 1 << 13,

	//!
	//!  \brief  high -> pump rpm is not tracking desired rpm
	//!
	CRITICAL_FAULT_PUMP_RPM_NOT_TRACKING = 1 << 14,

	//!
	//!  \brief  ...
	//!
	ALL_CRITICAL_FAULT_FLAGS =
		CRITICAL_FAULT_INTAKE_PRESSURE_LOW |
		CRITICAL_FAULT_MOTOR_OVERTEMP      |
		CRITICAL_FAULT_IMU_FAILED          |
		CRITICAL_FAULT_COMMS_TIMEOUT       |
		CRITICAL_FAULT_AIR_PSI_LOW         |
		CRITICAL_FAULT_PUMP_RPM_NOT_TRACKING,


	/*
	 * Non-critical faults
	 */

	///  \brief  high -> IMU is not aligned
	FAULT_IMU_ALIGNMENT_BAD = 1 << 15

} AtlasRobotStatus;


typedef enum
{
	HYDRAULIC_PRESSURE_OFF  = 0,
	HYDRAULIC_PRESSURE_LOW  = 1,
	HYDRAULIC_PRESSURE_HIGH = 2,

} AtlasHydraulicPressureSetting;


enum
{
	FLAG_LOW = 0,
	FLAG_HIGH = 1
};


///////////////////////////////////////////////////////////
//!
//!  \struct  AtlasControlDataFromRobot
//!
//!  \brief   Structure for current robot state to be streamed from the robot.
//!
struct AtlasControlDataFromRobot
{
	// timestamp should remain the first data member

	//!
	//!  \brief  Timestamp from robot hardware, in microseconds.
	//!
	//!    The time frame of the timestamp is arbitrary between boots of the
	//!    robot, but deltas between timestamps will track realtime deltas.
	//!
	uint64_t timestamp;

	int64_t seq_id;                                           //!< Sequence id of data from robot
	int64_t processed_to_robot_packet_seq_id;                 //!< Sequence id of last to-robot packet received and processed

	//!
	//!  \brief  Whether version of control data matches with on-robot software.
	//!
	//!    Set to 1 if there's a mismatch of control data version with the
	//!    on-robot software.  This usually means that the software installed
	//!    on the robot is a different version than that on the field
	//!    computer.  There's some chance control will continue to work even
	//!    if there's a mismatch.
	//!
	int64_t control_data_version_mismatch;

	AtlasRobotRunState  run_state;                            //!< State of robot
	uint64_t            robot_status_flags;                   //!< Set of AtlasRobotStatus flags

	///AtlasHydraulicPressureSetting pump_pressure;           // Removed for version 2.0.3
	float pump_inlet_pressure;                                //!< Pressure at pump inlet, in PSI
	float pump_supply_pressure;                               //!< Pressure at pump supply, in PSI
	float pump_return_pressure;                               //!< Pump return pressure, in PSI
	float air_sump_pressure;                                  //!< Air pressure in accusump

	AtlasJointState  j[Atlas::NUM_JOINTS];

	AtlasIMUData     filtered_imu;                            //!< Packet of filtered IMU data
	AtlasRawIMUData  raw_imu[Atlas::NUM_RAW_IMU_PACKETS];     //!< Last n packets unfiltered IMU data
	AtlasFootSensor  foot_sensors[Atlas::NUM_FOOT_SENSORS];   //!< State of foot sensors
	AtlasWristSensor wrist_sensors[Atlas::NUM_WRIST_SENSORS]; //!< State of wrist sensors

	AtlasPositionData pos_est;                                //!< World position estimate for reference when giving desired step positions (not updated except in behavior modes where noted)
	AtlasVec3f        foot_pos_est[Atlas::NUM_FEET];          //!< World position estimate for feet (not updated except in behavior modes where noted)

	AtlasRobotBehavior              current_behavior;         //!< Current behavior of robot

	AtlasBehaviorFeedback           behavior_feedback;        //!< General behavior feedback
	AtlasBehaviorStandFeedback      stand_feedback;           //!< Feedback specific to the Stand behavior
	AtlasBehaviorStepFeedback       step_feedback;            //!< Feedback specific to the Step behavior
	AtlasBehaviorWalkFeedback       walk_feedback;            //!< Feedback specific to the Walk behavior
	AtlasBehaviorManipulateFeedback manipulate_feedback;      //!< Feedback specific to the Manipulate behavior

	//!
	//!  \brief  Timestamp of sensor head PPS signal.
	//!
	//!    Uses the same time frame and units as the standard timestamp.
	//!    May not always be valid, if a sensor head PPS signal was not detected.
	//!    In that case, this value will be 0.
	//!
	uint64_t sensor_head_pps_timestamp;

	AtlasControlDataFromRobot() :
		timestamp(0l),
		seq_id(0l),
		processed_to_robot_packet_seq_id(0l),
		control_data_version_mismatch(0l),
		run_state(RUN_STATE_IDLE),
		robot_status_flags(0l),
		pump_inlet_pressure(0.0f),
		pump_supply_pressure(0.0f),
		pump_return_pressure(0.0f),
		air_sump_pressure(0.0f),
		current_behavior(BEHAVIOR_FREEZE)
	{}

	AtlasControlDataFromRobot(const AtlasControlDataFromRobot& rhs)
	{
		memcpy((uint8_t*) this,
			(uint8_t*) &rhs,
			sizeof(AtlasControlDataFromRobot));
	}

	AtlasControlDataFromRobot& operator= (const AtlasControlDataFromRobot& rhs)
	{
		memcpy((uint8_t*) this,
			(uint8_t*) &rhs,
			sizeof(AtlasControlDataFromRobot));
		return *this;
	}
};


///@}
//////////////////////////////////////////////////////////////////////////////
//!
//! \name Misc enumerations.
//!
///@{


///////////////////////////////////////////////////////////
//!
//!  \enum    AtlasErrorCode
//!
//!  \brief   Enumeration identifying error codes that can be returned from function calls.
//!
typedef enum
{
	NO_ERRORS                        =  0,   //!< no error detected
	ERROR_UNSPECIFIED                = -1,   //!< unspecified error
	ERROR_OPERATION_TIMED_OUT        = -2,   //!< operation timed out
	ERROR_ROBOT_UNSPECIFIED          = -3,   //!< no robot IP address or hostname specified
	ERROR_ROBOT_NET_CONNECT_FAILED   = -4,   //!< failed to establish net connection to robot
	ERROR_NO_ROBOT_NET_CONNECTION    = -5,   //!< function requires active connection to robot
	ERROR_INVALID_DATA_SEQ_ID        = -6,   //!< invalid data sequence number
	ERROR_FAILED_TO_SEND_DATA        = -7,   //!< failed to send data over network to robot
	ERROR_FAILED_TO_RECV_DATA        = -8,   //!< failed to receive data over network from robot
	ERROR_BAD_RECV_DATA              = -9,   //!< data received over network from robot bad
	ERROR_NO_DATA_AVAILABLE          = -10,  //!< no data available to read
	ERROR_COMMAND_INVALID_FROM_STATE = -11,  //!< sent command is invalid from current state
	ERROR_PREV_COMMAND_IN_PROGRESS   = -12,  //!< robot hasn't yet finished a previous command
	ERROR_ROBOT_COMMS_TIMED_OUT      = -13,  //!< communications to robot timed out
	ERROR_ILLEGAL_ARGUMENT           = -14,  //!< argument passed to function illegal
	ERROR_ENV_VAR_NOT_SET            = -15,  //!< environment variable ATLAS_ROBOT_INTERFACE not set
	ERROR_GAIT_DISABLED              = -16,  //!< desired gait disabled or not implemented
	NUM_ERROR_CODES

} AtlasErrorCode;

#ifndef NOT_IN_DOXYGEN
} // end namespace AtlasRobot
#endif

#ifdef __GNUC__
#pragma pack(pop)
#endif

#if __GNUC__ >= 4
#pragma GCC visibility pop
#endif


#endif  // __AtlasInterfaceTypes_H

