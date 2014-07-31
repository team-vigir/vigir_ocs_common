#ifndef ROBOT_STATE_MANAGER_H
#define ROBOT_STATE_MANAGER_H

#include <flor_moveit_ocs_model/moveit_ocs_model.h>

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
