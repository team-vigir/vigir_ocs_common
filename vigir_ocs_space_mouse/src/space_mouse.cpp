#include "space_mouse.h"

#define _USE_MATH_DEFINES
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
    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("/spacenav/joy", 10, &SpaceMouse::joyCallback, this);

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

    cout << "Templatelist recieved\n";
    cout << "Current id: " << id << "\n";
    //Find the pose within the list we just got
    for(int i = 0; i < template_id_list.size(); i++)
    {
        cout << "id at position " << i << ": " << template_id_list[i] << "\n";
        if(template_id_list[i] == id)
        {
            //Get pose stamped from the array of poses given
            cout << "Correct pose found!\n";
            pose = list->pose[i];
            cout << "X: " << pose.pose.position.x << "\n";
            cout << "Y: " << pose.pose.position.y << "\n";
            cout << "Z: " << pose.pose.position.z << "\n";
            recieved_pose = true;
        }
    }
}

void SpaceMouse::processObjectSelection(const flor_ocs_msgs::OCSObjectSelection::ConstPtr &obj)
{
    //Get id of object that is selected
    cout << "id recieved\n";

    id = obj->id;

    cout << "id set to: " << id << "\n";

}

void SpaceMouse::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    cout << "joy update\n";
    cout<< "value of recieved_pose: " << recieved_pose << "\n";

    if(recieved_pose)//Check if a template is selected
    {
        flor_ocs_msgs::OCSTemplateUpdate template_update;

        geometry_msgs::PoseStamped p = pose;

        cout << "Current x value = " << p.pose.position.x << "\n";
        cout << "Adding " << joy->axes[1] << "to x value\n";

        //Update x, y, and z values

        float scale = 0.3;

        p.pose.position.x += joy->axes[1] * scale;
        p.pose.position.y += joy->axes[0] * scale;
        p.pose.position.z += joy->axes[2] * scale;

        //Update the rotation
        Quaternion pre;
        pre.w = p.pose.orientation.w;
        pre.x = p.pose.orientation.x;
        pre.y = p.pose.orientation.y;
        pre.z = p.pose.orientation.z;

        Vector v = convertToEuler(pre);
        v.x += joy->axes[3];
        v.y += joy->axes[4];
        v.z += joy->axes[5];
        Quaternion q = convertToQuaternion(v.x, v.y, v.z);
        p.pose.orientation.w += q.w;
        p.pose.orientation.x += q.x;
        p.pose.orientation.y += q.y;
        p.pose.orientation.z += q.z;

        //Add to the template_update
        template_update.template_id = id;
        template_update.pose = p;

        cout << "New x value = " << template_update.pose.pose.position.x << "\n";


        //Publish the new pose
        template_update_pub_.publish(template_update);

        cout << "template updated and published" << "\n";
    }
}

SpaceMouse::Quaternion SpaceMouse::convertToQuaternion(double heading, double attitude, double bank)
{
    // Assuming the angles are in radians.
    double c1 = cos(heading/2);
    double s1 = sin(heading/2);
    double c2 = cos(attitude/2);
    double s2 = sin(attitude/2);
    double c3 = cos(bank/2);
    double s3 = sin(bank/2);
    double c1c2 = c1*c2;
    double s1s2 = s1*s2;

    Quaternion q;

    q.w =c1c2*c3 - s1s2*s3;
    q.x =c1c2*s3 + s1s2*c3;
    q.y =s1*c2*c3 + c1*s2*s3;
    q.z =c1*s2*c3 - s1*c2*s3;

    return q;
}


SpaceMouse::Vector SpaceMouse::convertToEuler(Quaternion q1)
{
    double heading, attitude, bank;

    double test = q1.x*q1.y + q1.z*q1.w;
        if (test > 0.499) { // singularity at north pole
            heading = 2 * atan2(q1.x,q1.w);
            attitude = M_PI/2;
            bank = 0;
            Vector v;
            v.x = heading;
            v.y = attitude;
            v.z = bank;
            return v;
        }
        else if (test < -0.499) { // singularity at south pole
            heading = -2 * atan2(q1.x,q1.w);
            attitude = - M_PI/2;
            bank = 0;
            Vector v;
            v.x = heading;
            v.y = attitude;
            v.z = bank;
            return v;
        }
        double sqx = q1.x*q1.x;
        double sqy = q1.y*q1.y;
        double sqz = q1.z*q1.z;
        heading = atan2(2*q1.y*q1.w-2*q1.x*q1.z , 1 - 2*sqy - 2*sqz);
        attitude = asin(2*test);
        bank = atan2(2*q1.x*q1.w-2*q1.y*q1.z , 1 - 2*sqx - 2*sqz);

        Vector v;
        v.x = heading;
        v.y = attitude;
        v.z = bank;
    return v;
}

}
