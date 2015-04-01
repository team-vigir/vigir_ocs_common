/*********************************************************************
*
* Software License Agreement (BSD License)
*
* Copyright (c) 2008, Willow Garage, Inc.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above
* copyright notice, this list of conditions and the following
* disclaimer in the documentation and/or other materials provided
* with the distribution.
* * Neither the name of Willow Garage, Inc. nor the names of its
* contributors may be used to endorse or promote products derived
* from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*********************************************************************/
namespace actionlib
{
////////Constuctors  //////////////////////////////////////////
template <class ActionSpec>
ComplexActionServer<ActionSpec>::ComplexActionServer(std::string name, ExecuteCallback execute_callback, bool auto_start)
    : new_goal_(false), preempt_request_(false), new_goal_preempt_request_(false), execute_callback_(execute_callback), execute_thread_(NULL), need_to_terminate_(false)
{
    if (execute_callback_ != NULL)
    {
        execute_thread_ = new boost::thread(boost::bind(&ComplexActionServer::executeLoop, this));
    }
    //create the action server
    as_ = boost::shared_ptr<ActionServer<ActionSpec> >(new ActionServer<ActionSpec>(n_, name,
                                                                                    boost::bind(&ComplexActionServer::goalCallback, this, _1),
                                                                                    boost::bind(&ComplexActionServer::preemptCallback, this, _1),
                                                                                    auto_start));
}

template <class ActionSpec>
ComplexActionServer<ActionSpec>::ComplexActionServer(std::string name, bool auto_start)
    : new_goal_(false), preempt_request_(false), new_goal_preempt_request_(false), execute_callback_(NULL), need_to_terminate_(false)
{
    //create the action server
    as_ = boost::shared_ptr<ActionServer<ActionSpec> >(new ActionServer<ActionSpec>(n_, name,
                                                                                    boost::bind(&ComplexActionServer::goalCallback, this, _1),
                                                                                    boost::bind(&ComplexActionServer::preemptCallback, this, _1),
                                                                                    auto_start));
    if (execute_callback_ != NULL)
    {
        execute_thread_ = new boost::thread(boost::bind(&ComplexActionServer::executeLoop, this));
    }
}

template <class ActionSpec>
ComplexActionServer<ActionSpec>::ComplexActionServer(std::string name, ExecuteCallback execute_callback)
    : new_goal_(false), preempt_request_(false), new_goal_preempt_request_(false), execute_callback_(execute_callback), need_to_terminate_(false)
{
    //create the action server
    as_ = boost::shared_ptr<ActionServer<ActionSpec> >(new ActionServer<ActionSpec>(n_, name,
                                                                                    boost::bind(&ComplexActionServer::goalCallback, this, _1),
                                                                                    boost::bind(&ComplexActionServer::preemptCallback, this, _1),
                                                                                    true));
    if (execute_callback_ != NULL)
    {
        execute_thread_ = new boost::thread(boost::bind(&ComplexActionServer::executeLoop, this));
    }
}

template <class ActionSpec>
ComplexActionServer<ActionSpec>::ComplexActionServer(ros::NodeHandle n, std::string name, ExecuteCallback execute_callback, bool auto_start)
    : n_(n), new_goal_(false), preempt_request_(false), new_goal_preempt_request_(false), execute_callback_(execute_callback), need_to_terminate_(false)
{
    //create the action server
    as_ = boost::shared_ptr<ActionServer<ActionSpec> >(new ActionServer<ActionSpec>(n, name,
                                                                                    boost::bind(&ComplexActionServer::goalCallback, this, _1),
                                                                                    boost::bind(&ComplexActionServer::preemptCallback, this, _1),
                                                                                    auto_start));
    if (execute_callback_ != NULL)
    {
        execute_thread_ = new boost::thread(boost::bind(&ComplexActionServer::executeLoop, this));
    }
}

template <class ActionSpec>
ComplexActionServer<ActionSpec>::ComplexActionServer(ros::NodeHandle n, std::string name, bool auto_start)
    : n_(n), new_goal_(false), preempt_request_(false), new_goal_preempt_request_(false), execute_callback_(NULL), need_to_terminate_(false)
{
    //create the action server
    as_ = boost::shared_ptr<ActionServer<ActionSpec> >(new ActionServer<ActionSpec>(n, name,
                                                                                    boost::bind(&ComplexActionServer::goalCallback, this, _1),
                                                                                    boost::bind(&ComplexActionServer::preemptCallback, this, _1),
                                                                                    auto_start));
    if (execute_callback_ != NULL)
    {
        execute_thread_ = new boost::thread(boost::bind(&ComplexActionServer::executeLoop, this));
    }
}

template <class ActionSpec>
ComplexActionServer<ActionSpec>::ComplexActionServer(ros::NodeHandle n, std::string name, ExecuteCallback execute_callback)
    : n_(n), new_goal_(false), preempt_request_(false), new_goal_preempt_request_(false), execute_callback_(execute_callback), need_to_terminate_(false)
{
    //create the action server
    as_ = boost::shared_ptr<ActionServer<ActionSpec> >(new ActionServer<ActionSpec>(n, name,
                                                                                    boost::bind(&ComplexActionServer::goalCallback, this, _1),
                                                                                    boost::bind(&ComplexActionServer::preemptCallback, this, _1),
                                                                                    true));
    if (execute_callback_ != NULL)
    {
        execute_thread_ = new boost::thread(boost::bind(&ComplexActionServer::executeLoop, this));
    }
}
////////End Constuctors  //////////////////////////////////////////

template <class ActionSpec>
ComplexActionServer<ActionSpec>::~ComplexActionServer()
{
    if(execute_thread_)
        shutdown();
}


template <class ActionSpec>
void ComplexActionServer<ActionSpec>::shutdown()
{
    if (execute_callback_)
    {
        {
            boost::mutex::scoped_lock terminate_lock(terminate_mutex_);
            need_to_terminate_ = true;
        }
        assert(execute_thread_);
        execute_thread_->join();
        delete execute_thread_;
        execute_thread_ = NULL;
    }
}

//called after receiving a goal
template <class ActionSpec>
//boost::shared_ptr<const typename ComplexActionServer<ActionSpec>::Goal> ComplexActionServer<ActionSpec>::acceptNewGoal()
boost::shared_ptr< typename ActionServer<ActionSpec>::GoalHandle> ComplexActionServer<ActionSpec>::acceptNewGoal()
{    
    boost::recursive_mutex::scoped_lock lock(lock_);
    if(!new_goal_ || !next_goal_.getGoal()){
        ROS_ERROR_NAMED("actionlib", "Attempting to accept the next goal when a new goal is not available");
        return boost::shared_ptr<GoalHandle>();
    }
//    //check if we need to send a preempted message for the goal that we're currently pursuing
//    if(isActive()
//            && current_goal_.getGoal()
//            && current_goal_ != next_goal_){
//        current_goal_.setCanceled(Result(), "This goal was canceled because another goal was recieved by the Complex action server");
//    }
    ROS_DEBUG_NAMED("actionlib", "Accepting a new goal");
    //accept the next goal
    current_goal_ = next_goal_;
    //goals_received_.push_back(current_goal_.getGoal);
    new_goal_ = false;
//    //set preempt to request to equal the preempt state of the new goal
//    preempt_request_ = new_goal_preempt_request_;
//    new_goal_preempt_request_ = false;

    //set the status of the current goal to be active    
    current_goal_.setAccepted("This goal has been accepted by the Complex action server");
    boost::shared_ptr<GoalHandle> goal_ptr(&current_goal_);//.getGoal();
    return goal_ptr;
}


template <class ActionSpec>
bool ComplexActionServer<ActionSpec>::isNewGoalAvailable(){
    return new_goal_;
}


template <class ActionSpec>
bool ComplexActionServer<ActionSpec>::isPreemptRequested()
{
    return false;
    //return preempt_request_;
}


template <class ActionSpec>
bool ComplexActionServer<ActionSpec>::isActive()
{
    return goals_received_.size() > 0;

    //if(!current_goal_.getGoal())
     //   return false;

    //unsigned int status = current_goal_.getGoalStatus().status;
    //return status == actionlib_msgs::GoalStatus::ACTIVE || status == actionlib_msgs::GoalStatus::PREEMPTING;
}


template <class ActionSpec>
void ComplexActionServer<ActionSpec>::setSucceeded(const Result& result, const std::string& text, GoalHandlePtr  goal)
{
    boost::recursive_mutex::scoped_lock lock(lock_);
    //ROS_DEBUG_NAMED("actionlib", "Setting the current goal as succeeded");
    //current_goal_.setSucceeded(result, text);    
    goal->setSucceeded(result, text);

}


template <class ActionSpec>
void ComplexActionServer<ActionSpec>::setAborted(const Result& result, const std::string& text, GoalHandlePtr  goal)
{
    boost::recursive_mutex::scoped_lock lock(lock_);
    //ROS_DEBUG_NAMED("actionlib", "Setting the current goal as aborted");
    //current_goal_.setAborted(result, text);
    goal.get()->setAborted(result,text);
}

//template <class ActionSpec>
//void ComplexActionServer<ActionSpec>::findGoal(GoalConstPtr goal)
//{
//    for(int i=0;i<goals_received_.size();++i)
//    {
//        if(goal == goals_received_[i])
//        {

//            return goals_received_[i]; //
//        }
//    }
//}



template <class ActionSpec>
void ComplexActionServer<ActionSpec>::setPreempted(const Result& result, const std::string& text)
{
    return;
////    boost::recursive_mutex::scoped_lock lock(lock_);
////    ROS_DEBUG_NAMED("actionlib", "Setting the current goal as canceled");
////    current_goal_.setCanceled(result, text);
///
}


template <class ActionSpec>
void ComplexActionServer<ActionSpec>::registerGoalCallback(boost::function<void ()> cb)
{
    // Cannot register a goal callback if an execute callback exists
    if (execute_callback_)
        ROS_WARN_NAMED("actionlib", "Cannot call ComplexActionServer::registerGoalCallback() because an executeCallback exists. Not going to register it.");
    else
        goal_callback_ = cb;
}


template <class ActionSpec>
void ComplexActionServer<ActionSpec>::registerPreemptCallback(boost::function<void ()> cb)
{
    return;
    //preempt_callback_ = cb;
}


template <class ActionSpec>
void ComplexActionServer<ActionSpec>::publishFeedback(const FeedbackConstPtr& feedback)
{
    current_goal_.publishFeedback(*feedback);
}


template <class ActionSpec>
void ComplexActionServer<ActionSpec>::publishFeedback(const Feedback& feedback)
{
    current_goal_.publishFeedback(feedback);
}


//called whenever a goal is received
template <class ActionSpec>
void ComplexActionServer<ActionSpec>::goalCallback(GoalHandle goal)
{
    boost::recursive_mutex::scoped_lock lock(lock_);
    ROS_DEBUG_NAMED("actionlib", "A new goal has been received by the single goal action server");


    //check that the timestamp is past or equal to that of the current goal and the next goal
//    if((!current_goal_.getGoal() || goal.getGoalID().stamp >= current_goal_.getGoalID().stamp)
//            && (!next_goal_.getGoal() || goal.getGoalID().stamp >= next_goal_.getGoalID().stamp))
//    {

        //if next_goal has not been accepted already... its going to get bumped, but we need to let the client know we're preempting
//        if(next_goal_.getGoal() && (!current_goal_.getGoal() || next_goal_ != current_goal_)){
//            next_goal_.setCanceled(Result(), "This goal was canceled because another goal was recieved by the Complex action server");
//      }

        next_goal_ = goal;
        new_goal_ = true;

        ROS_ERROR("goal callback\n");
        //goals_received_.push_back(goal);

      //  new_goal_preempt_request_ = false;
        //if the server is active, we'll want to call the preempt callback for the current goal
//        if(isActive()){
//            preempt_request_ = true;
//            //if the user has registered a preempt callback, we'll call it now
//            if(preempt_callback_)
//                preempt_callback_();
//        }


        //if the user has defined a goal callback, we'll call it now
       if(goal_callback_)
       {
           lock.unlock();

           //boost::thread goal_thread(boost::bind(&goal_callback_,this));

           lock.lock();
       }

        //run execute to accept goal and run
        execute_condition_.notify_all();


//    }
//    else{
//        //the goal requested has already been preempted by a different goal, so we're not going to execute it
//        goal.setCanceled(Result(), "This goal was canceled because another goal was recieved by the Complex action server");
//    }
}


template <class ActionSpec>
void ComplexActionServer<ActionSpec>::preemptCallback(GoalHandle preempt)
{
     return;
//    boost::recursive_mutex::scoped_lock lock(lock_);
//    ROS_DEBUG_NAMED("actionlib", "A preempt has been received by the ComplexActionServer");
//    //if the preempt is for the current goal, then we'll set the preemptRequest flag and call the user's preempt callback
//    if(preempt == current_goal_){
//        ROS_DEBUG_NAMED("actionlib", "Setting preempt_request bit for the current goal to TRUE and invoking callback");
//        preempt_request_ = true;
//        //if the user has registered a preempt callback, we'll call it now
//        if(preempt_callback_)
//            preempt_callback_();
//    }
//    //if the preempt applies to the next goal, we'll set the preempt bit for that
//    else if(preempt == next_goal_){
//        ROS_DEBUG_NAMED("actionlib", "Setting preempt request bit for the next goal to TRUE");
//        new_goal_preempt_request_ = true;
//    }
}


template <class ActionSpec>
void ComplexActionServer<ActionSpec>::executeLoop()
{
    ros::Duration loop_duration = ros::Duration().fromSec(.1);
    while (n_.ok())
    {
        {
            boost::mutex::scoped_lock terminate_lock(terminate_mutex_);
            if (need_to_terminate_)
                break;
        }
        boost::recursive_mutex::scoped_lock lock(lock_);

        //if (isActive())
            //ROS_ERROR_NAMED("actionlib", "Should never reach this code with an active goal");

        //need to spin new thread for each goal callback to execute

        if (isNewGoalAvailable())
        {
            GoalHandlePtr goal_handle = acceptNewGoal();
            ROS_FATAL_COND(!execute_callback_, "execute_callback_ must exist. This is a bug in ComplexActionServer");

            // Make sure we're not locked when we call execute
            lock.unlock();

            //spin new thread with this goal
            boost::thread goal_thread(boost::bind(&ComplexActionServer::runGoal,this, goal_handle->getGoal(),goal_handle));

            lock.lock();
//            if (isActive())
//            {
//                ROS_WARN_NAMED("actionlib", "Your executeCallback did not set the goal to a terminal status.\n"
//                               "This is a bug in your ActionServer implementation. Fix your code!\n"
//                               "For now, the ActionServer will set this goal to aborted");
//                setAborted(Result(), "This goal was aborted by the Complex action server. The user should have set a terminal status on this goal and did not");
//            }
        }
        else
            execute_condition_.timed_wait(lock, boost::posix_time::milliseconds(loop_duration.toSec() * 1000.0f));
    }
}

template <class ActionSpec>
void ComplexActionServer<ActionSpec>::runGoal(GoalConstPtr goal,GoalHandlePtr goal_handle)
{
    execute_callback_(goal, goal_handle);
}


template <class ActionSpec>
void ComplexActionServer<ActionSpec>::start()
{
    as_->start();
}

};
