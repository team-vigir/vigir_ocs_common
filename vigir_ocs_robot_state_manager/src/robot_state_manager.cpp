
#include "robot_state_manager.h"

// Allocating and initializing GlobalClass's
// static data member.  The pointer is being
// allocated - not the object inself
RobotStateManager* RobotStateManager::instance = 0;

RobotStateManager::RobotStateManager()
{
    robot_state_= new MoveItOcsModel();
    ghost_robot_state_= new MoveItOcsModel();    
}

/** This function is called to create an instance of the class.
    Calling the constructor publicly is not allowed. The constructor
    is private and is only called by this Instance function.
*/
RobotStateManager* RobotStateManager::Instance()
{
   if (!instance)   // Only allow one instance of class to be generated.
       instance = new RobotStateManager();

   return instance;
}

MoveItOcsModel* RobotStateManager::getRobotStateSingleton()
{
    return robot_state_;
}
MoveItOcsModel* RobotStateManager::getGhostRobotStateSingleton()
{
    return ghost_robot_state_;
}
