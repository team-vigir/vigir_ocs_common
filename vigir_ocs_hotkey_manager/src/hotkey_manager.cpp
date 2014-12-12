
#include "hotkey_manager.h"

HotkeyManager* HotkeyManager::instance_ = 0;

HotkeyManager::HotkeyManager()
{
    key_event_sub_ = nh_.subscribe<flor_ocs_msgs::OCSKeyEvent>( "/flor/ocs/key_event", 5, &HotkeyManager::processNewKeyEvent, this );
}

HotkeyManager * HotkeyManager::Instance()
{
    if (!instance_)   // Only allow one instance of class to be generated.
        instance_ = new HotkeyManager();

    return instance_;
}

void HotkeyManager::processNewKeyEvent(const flor_ocs_msgs::OCSKeyEvent::ConstPtr &key_event)
{
    // store key state
    if(key_event->state)
        keys_pressed_list_.push_back(key_event->keystr);
    else
        keys_pressed_list_.erase(std::remove(keys_pressed_list_.begin(), keys_pressed_list_.end(), key_event->keystr), keys_pressed_list_.end());

//    // process hotkeys
//    bool ctrl_is_pressed = (std::find(keys_pressed_list_.begin(), keys_pressed_list_.end(), "Control_") != keys_pressed_list_.end());
//    bool shift_is_pressed = (std::find(keys_pressed_list_.begin(), keys_pressed_list_.end(), "Shift_") != keys_pressed_list_.end());
//    bool alt_is_pressed = (std::find(keys_pressed_list_.begin(), keys_pressed_list_.end(), "Alt_") != keys_pressed_list_.end());

    //case for every hotkey in ocs

    ///BASEVIEW         ///////////////////////////////////////////////////////////////////
    if(key_event->keystr == "Escape" && key_event->state) // 'esc'
    {
        //callHotkeyFunction("esc");
        // reset everything
        //deselectAll();
       // manager_->getToolManager()->setCurrentTool( interactive_markers_tool_ );
    }
//    else if(key_event->keystr == "q" && key_event->state && ctrl_is_pressed) // ctrl+q
//    {
          //callHotkeyFunction("ctrl+q");
//        // robot model visibility
//        robotModelToggled(!robot_model_->isEnabled());
//    }
//    else if(key_event->keystr == "w" && key_event->state && ctrl_is_pressed) // ctrl+w
//    {
//        // ghost visibility
//        simulationRobotToggled(!ghost_robot_model_->isEnabled());
//    }
//    else if(key_event->keystr == "1" && key_event->state && ctrl_is_pressed) // ctrl+1
//    {
//        // reset point clouds
//        clearPointCloudRaycastRequests();
//        clearPointCloudRegionRequests();
//        clearPointCloudStereoRequests();
//    }
//    else if(key_event->keystr == "9" && key_event->state && ctrl_is_pressed) // ctrl+9
//    {
//        // rainbow color
//        region_point_cloud_viewer_->subProp( "Color Transformer" )->setValue( "AxisColor" );
//    }
//    else if(key_event->keystr == "0" && key_event->state && ctrl_is_pressed) // ctrl+0
//    {
//        // intensity
//        region_point_cloud_viewer_->subProp( "Color Transformer" )->setValue( "Intensity" );
//    }
//    else if(key_event->keystr == "g" && key_event->state && ctrl_is_pressed) // ctrl+g
//    {
//        // define a step goal
//        defineFootstepGoal();
//    }
//    else if(key_event->keystr == "h" && key_event->state && ctrl_is_pressed) // ctrl+h
//    {
//        // request plan
//        if(footstep_vis_manager_->hasGoal())
//            footstep_vis_manager_->requestStepPlan();
//    }
//    else if(key_event->keystr == "j" && key_event->state && ctrl_is_pressed) // ctrl+j
//    {
//        // request plan
//        if(footstep_vis_manager_->hasValidStepPlan())
//            footstep_vis_manager_->requestExecuteStepPlan();
//    }
//    else if(ctrl_is_pressed && alt_is_pressed) // ctrl+alt emergency stop
//    {
//        stop_button_->setVisible(true);
//        stop_button_->setGeometry(this->geometry().bottomRight().x()/2 - 200,this->geometry().bottomRight().y()/2 - 150,400,300);
//    }
//    else if(shift_is_pressed && !shift_pressed_)
//    {
//        //Lock translation during rotation
//        flor_ocs_msgs::OCSControlMode msgMode;
//        if(interactive_marker_mode_ < IM_MODE_OFFSET)
//            msgMode.manipulationMode = interactive_marker_mode_ + IM_MODE_OFFSET;
//        else
//            msgMode.manipulationMode = interactive_marker_mode_ - IM_MODE_OFFSET;
//        interactive_marker_server_mode_pub_.publish(msgMode);
//        shift_pressed_ = true;
//    }
//    else
//    {
//        stop_button_->setVisible(false);

//        //Unclock translation during rotation
//        if(shift_pressed_)
//        {
//            flor_ocs_msgs::OCSControlMode msgMode;
//            if(interactive_marker_mode_ < IM_MODE_OFFSET)//Check if mode is 0, 1 or 2
//                msgMode.manipulationMode = interactive_marker_mode_;
//            else//means that shift is pressed
//                msgMode.manipulationMode = interactive_marker_mode_ - IM_MODE_OFFSET;
//            interactive_marker_server_mode_pub_.publish(msgMode);
//            shift_pressed_ = false;
//        }

//    }
    ///END BASEVIEW             ///////////////////////////////////////////////////////////////////

}

void HotkeyManager::callHotkeyFunction(std::string keyCombo)
{
    //gets the boost function then call binded function
    hotkey_functions_[keyCombo]();
}

void HotkeyManager::addHotkeyFunction(std::string keyCombo, boost::function<void()> function)
{
    if(hotkey_functions_.find(keyCombo) == hotkey_functions_.end())
        hotkey_functions_[keyCombo] = function;
    else
        ROS_ERROR("tried to add duplicating hotkey functions");
}




