
#include "behavior_manager.h"

// Allocating and initializing GlobalClass's
// static data member.  The pointer is being
// allocated - not the object inself
BehaviorManager* BehaviorManager::instance = 0;

BehaviorManager::BehaviorManager()
{    
//    behavior_sub_ = nh_.subscribe<flor_ocs_msgs::OCSSynchronize>( "/flor/ocs/synchronize", 5, &BehaviorManager::createNotification, this );

}

/** This function is called to create an instance of the class.
    Calling the constructor publicly is not allowed. The constructor
    is private and is only called by this Instance function.
*/
BehaviorManager* BehaviorManager::Instance()
{
   if (!instance)   // Only allow one instance of class to be generated.
       instance = new BehaviorManager();

   return instance;
}


//void BehaviorManager::createNotification(const flor_ocs_msgs::OCSSynchronize& msg)
//{

//}

void BehaviorManager::createNotification()
{

}
