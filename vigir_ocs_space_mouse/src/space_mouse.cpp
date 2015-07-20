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
#include "space_mouse.h"

#define _USE_MATH_DEFINES
using namespace std;

namespace vigir_ocs
{


SpaceMouse::SpaceMouse()//QObject* parent = 0)
    //: QObject(parent)
{

    // create publishers for visualization

    // get list of all templates and their poses
    //template_list_sub_ = nh_.subscribe<vigir_ocs_msgs::OCSTemplateList>("/template/list", 5, &SpaceMouse::processTemplateList, this);

    // get object that is selected
    //select_object_sub_ = nh_.subscribe<vigir_ocs_msgs::OCSObjectSelection>( "/flor/ocs/object_selection", 5, &SpaceMouse::processObjectSelection, this );

    //Initialize the the subscriber for the joy topic
    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("/spacenav/joy", 10, &SpaceMouse::joyCallback, this);

    // update template position
    template_update_pub_  = nh_.advertise<vigir_ocs_msgs::OCSTemplateUpdate>( "/template/update", 1, false );

    //ONLY TEMPORARY
    recieved_pose = true;
    //cout << "SpaceMouse created";

    //camera_sub_ = nh_.subscribe<vigir_ocs_msgs::OCSCameraTransform>( "/flor/ocs/camera_transform",5,&SpaceMouse::cameraCb,this);

    camera_pub_ = nh_.advertise<geometry_msgs::Pose>("/flor/ocs/set_camera_transform", 1, false);
    //ROS_ERROR("PRINT TEST");

}

SpaceMouse::~SpaceMouse()
{
}

//copy camera information and store
void SpaceMouse::cameraCb(const vigir_ocs_msgs::OCSCameraTransform::ConstPtr& msg)
{
    //ROS_ERROR("In camera cb");
    if(msg->widget_name == "MainView" && msg->view_id == 0)
    {
        cameraUpdate = *msg;

        //update camera geometry
        cameraPosition.setX(cameraUpdate.pose.position.x);
        cameraPosition.setY(cameraUpdate.pose.position.y);
        cameraPosition.setZ(cameraUpdate.pose.position.z);

        pose_.pose = cameraUpdate.pose;

        cameraOrientation.setX(cameraUpdate.pose.orientation.x);
        cameraOrientation.setY(cameraUpdate.pose.orientation.y);
        cameraOrientation.setZ(cameraUpdate.pose.orientation.z);
        cameraOrientation.setScalar(cameraUpdate.pose.orientation.w);
        recieved_pose = true;
    }
}

void SpaceMouse::processTemplateList(const vigir_ocs_msgs::OCSTemplateList::ConstPtr &list)
{
    //template_id_list = list->template_id_list;

    //cout << "Current id: " << id << "\n";
    //Find the pose within the list we just got
    for(int i = 0; i < template_id_list.size(); i++)
    {
        //cout << "id at position " << i << ": " << (int)template_id_list[i] << "\n";
        if((int)template_id_list[i] == id)
        {
            //Get pose stamped from the array of poses given
            //pose = list->pose[i];
            //recieved_pose = true;
        }
    }
}

void SpaceMouse::processObjectSelection(const vigir_ocs_msgs::OCSObjectSelection::ConstPtr &obj)
{
    //Get id of object that is selected

    /*if(obj->id != id)
        recieved_pose = false;
    id = obj->id;*/

}

void SpaceMouse::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    if(recieved_pose)//Check if a template is selected
    {
        //vigir_ocs_msgs::OCSTemplateUpdate template_update;

        vigir_ocs_msgs::OCSCameraTransform update = cameraUpdate;

        //This is the pose that we will work with
        geometry_msgs::PoseStamped p = pose_;

        //p = updatePose(p, joy);

        float rotation_scale = 0.035; //* length;
        float scale = 0.2; //* length;

        //Add to the template_update
        if(recieved_pose)
        {
            //template_update.template_id = id;
            //template_update.pose = p;

            //Publish the new pose
            //template_update_pub_.publish(template_update);

//            update.pose.position.x = p.pose.position.x;
//            update.pose.position.y = p.pose.position.y;
//            update.pose.position.z = p.pose.position.z;

//            update.pose.orientation.w = p.pose.orientation.w;
//            update.pose.orientation.x = p.pose.orientation.x;
//            update.pose.orientation.y = p.pose.orientation.y;
//            update.pose.orientation.z = p.pose.orientation.z;

//            camera_pub_.publish(update);
            geometry_msgs::Pose relative_pose;
            relative_pose.position.x = -joy->axes[1]*scale;
            relative_pose.position.y = joy->axes[2]*scale;
            relative_pose.position.z = -joy->axes[0]*scale;

            relative_pose.orientation.x = joy->axes[4] * rotation_scale;
            relative_pose.orientation.z = joy->axes[5] * rotation_scale;
            camera_pub_.publish(relative_pose);

//            cout << "template updated and published" << "\n";
        }

    }
}

//This function does the transformation from current camera position to what is recieved from the spacemouse node
geometry_msgs::PoseStamped SpaceMouse::updatePose(geometry_msgs::PoseStamped p, const sensor_msgs::Joy::ConstPtr& joy)
{

    //Calculate scale
    //QVector3D cam = QVector3D(cameraPosition.x(), cameraPosition.y(), cameraPosition.z());
    //QVector3D obj = QVector3D(p.pose.position.x, p.pose.position.y, p.pose.position.z);

    //double length = (cam - obj).length();
    //cout << "Length: " << length << "\n";

    float rotation_scale = 0.035; //* length;
    float scale = 0.2; //* length;
    //Translation- move relative to camera for direction but still translate in world space
    //create a vector representing the offset to be applied to position.
    QVector3D* offset = new QVector3D(joy->axes[1] * scale *-1,joy->axes[2] * scale, -joy->axes[0] * scale);
    //rotate the offset to be relative to camera orientation, can now be applied to position to move relative to camera
    *offset = cameraOrientation.rotatedVector(*offset);


    //Update x, y, and z values



    p.pose.position.x += offset->x();
    p.pose.position.y += offset->y();
    p.pose.position.z += offset->z();

    //Update the rotation


//    cout << "Original W:" << p.pose.orientation.w << "\n";
//    cout << "Original X:" << p.pose.orientation.x << "\n";
//    cout << "Original Y:" << p.pose.orientation.y << "\n";
//    cout << "Original Z:" << p.pose.orientation.z << "\n";

    QQuaternion pre;
    pre.setScalar(p.pose.orientation.w);
    pre.setX(p.pose.orientation.x);
    pre.setY(p.pose.orientation.y);
    pre.setZ(p.pose.orientation.z);
    //SpaceMouse::Vector v = convertToEuler(pre);

    QQuaternion difference;
    //camera relative rotation
    QQuaternion* rot = new QQuaternion();                  //convert from radians to degrees * 180/pi
    if(std::abs(joy->axes[5]) > 0.3)
        *rot *= QQuaternion::fromAxisAndAngle(0,1,0,(joy->axes[5]*rotation_scale)* 57.2957795131);
    else
        *rot *= QQuaternion::fromAxisAndAngle(0,1,0,(0)* 57.2957795131);
    if(std::abs(joy->axes[4]) > 0.3)
        *rot *= QQuaternion::fromAxisAndAngle(1,0,0,-(joy->axes[4]*rotation_scale)* 57.2957795131); //more intuitive for camera if reversed
    else
        *rot *= QQuaternion::fromAxisAndAngle(1,0,0,-(0)* 57.2957795131); //more intuitive for camera if reversed
    if(std::abs(joy->axes[3]) > 0.3)
        *rot *= QQuaternion::fromAxisAndAngle(0,0,1,(-joy->axes[3]*rotation_scale)* 57.2957795131);
    else
       *rot *= QQuaternion::fromAxisAndAngle(0,0,1,(0)* 57.2957795131);
    //*rot *= QQuaternion::fromAxisAndAngle(1,0,0,-(joy->axes[4]*rotation_scale)* 57.2957795131); //more intuitive for camera if reversed
    //*rot *= QQuaternion::freomAxisAndAngle(0,0,1,(joy->axes[5]*rotation_scale)* 57.2957795131);

    //cout << "rot x:" << rot->x() << "\n";
    //cout << "rot y:" << rot->y() << "\n";
    //cout << "rot z:" << rot->z() << "\n";

    //calculate difference between camera orientation and original rotation of object
    //difference of q1 and q2 is  q` = q^-1 * q2
    difference = cameraOrientation.conjugate() * (pre);
    //set object orientation to camera
    pre = cameraOrientation;
    //apply desired rotation
    pre *= *rot;
    //revert back change of camera rotation to leave object in newly rotated state
    pre *= difference;
    delete(rot);


//        v.x += joy->axes[3]*sceale;
//        v.y += joy->axes[4]*scale;
//        v.z += joy->axes[5]*scale;
    //Quaternion q = convertToQuaternion(v.x, v.y, v.z);
    p.pose.orientation.w = pre.scalar();
    p.pose.orientation.x = pre.x();
    p.pose.orientation.y = pre.y();
    p.pose.orientation.z = pre.z();

    return p;
}

}
