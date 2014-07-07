#include "space_mouse.h"
#include <sensor_msgs/Joy.h>

#include <flor_ocs_msgs/OCSTemplateList.h>
#include <flor_ocs_msgs/OCSObjectSelection.h>
#include<flor_ocs_msgs/OCSTemplateUpdate.h>
#include<geometry_msgs/Pose.h>
#include<geometry_msgs/Point.h>

#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

using namespace std;

namespace vigir_ocs
{
SpaceMouse::SpaceMouse()
{

    // create publishers for visualization

    // get list of all templates and their poses
    template_list_sub_ = nh_.subscribe<flor_ocs_msgs::OCSTemplateList>("/template/list", 5, &SpaceMouse::processTemplateList, this);

    // get object that is selected
    select_object_sub_ = nh_.subscribe<flor_ocs_msgs::OCSObjectSelection>( "/flor/ocs/object_selection", 5, &SpaceMouse::processObjectSelection, this );

    //Initialize the the subscriber for the joy topic
    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &SpaceMouse::joyCallback, this);

    // update template position
    template_update_pub_  = nh_.advertise<flor_ocs_msgs::OCSTemplateUpdate>( "/template/update", 1, false );

    recieved_pose = false;
    cout << "SpaceMouse created";
}

SpaceMouse::~SpaceMouse()
{
}

void SpaceMouse::processTemplateList(const flor_ocs_msgs::OCSTemplateList::ConstPtr &list)
{
    template_id_list = list->template_id_list;

    cout << "Templatelist recieved";
    cout << "Current id: " << id;
    //Find the pose within the list we just got
    for(int i = 0; i < template_id_list.size(); i++)
    {
        cout << "id at position " << i << ": " << template_id_list[i];
        if(template_id_list[i] == id)
        {
            //Get pose stamped from the array of poses given
            cout << "Correct pose found!";
            pose = list->pose[i];
            recieved_pose = true;
        }
    }
}

void SpaceMouse::processObjectSelection(const flor_ocs_msgs::OCSObjectSelection::ConstPtr &obj)
{
    //Get id of object that is selected
    cout << "id recieved";

    id = obj->id;

    cout << "id set to: " << id;

}

void SpaceMouse::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    cout << "joy update";
    cout<< "value of recieved _pose: " << recieved_pose;

    if(recieved_pose)//Check if a template is selected
    {
        flor_ocs_msgs::OCSTemplateUpdate template_update;

        //Update x, y, and z values
        geometry_msgs::PoseStamped p = pose;

        cout << "Current x value = " << p.pose.position.x;
        cout << "Adding " << joy->axes[1] << "to x value";

        p.pose.position.x += joy->axes[1];
        p.pose.position.y += joy->axes[0];
        p.pose.position.z += joy->axes[2];

        //Add to the template_update
        template_update.template_id = id;
        template_update.pose = p;

        cout << "New x value = " << template_update.pose.pose.position.x;


        //Publish the new pose
        template_update_pub_.publish(template_update);

        cout << "template updated and published";
    }
}

}
