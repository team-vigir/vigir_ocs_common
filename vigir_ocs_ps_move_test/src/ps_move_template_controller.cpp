#include "ps_move_template_controller.h"

#define _USE_MATH_DEFINES
using namespace std;

namespace vigir_ocs
{

PSMoveTemplateController::PSMoveTemplateController()
{
    // create publishers for visualization

    // get list of all templates and their poses
    template_list_sub_ = nh_.subscribe<flor_ocs_msgs::OCSTemplateList>("/template/list", 5, &PSMoveTemplateController::processTemplateList, this);

    // get object that is selected
    select_object_sub_ = nh_.subscribe<flor_ocs_msgs::OCSObjectSelection>( "/flor/ocs/object_selection", 5, &PSMoveTemplateController::processObjectSelection, this );

    // update template position
    template_update_pub_  = nh_.advertise<flor_ocs_msgs::OCSTemplateUpdate>( "/template/update", 1, false );

    received_pose = false;
    ROS_ERROR("PSMoveTemplateController created");

    camera_sub = nh_.subscribe<flor_ocs_msgs::OCSCameraTransform>( "/flor/ocs/camera_transform",5,&PSMoveTemplateController::cameraCb,this);

    camera_pub = nh_.advertise<flor_ocs_msgs::OCSCameraTransform>("/flor/ocs/set_camera_transform", 1, false);
}

PSMoveTemplateController::~PSMoveTemplateController()
{
}

//copy camera information and store
void PSMoveTemplateController::cameraCb(const flor_ocs_msgs::OCSCameraTransform::ConstPtr& msg)
{
    if(msg->widget_name == "MainView" && msg->view_id == 0)
    {
        cameraUpdate = *msg;

        //update camera geometry
        cameraPosition.setX(cameraUpdate.pose.position.x);
        cameraPosition.setY(cameraUpdate.pose.position.y);
        cameraPosition.setZ(cameraUpdate.pose.position.z);

        cameraOrientation.setX(cameraUpdate.pose.orientation.x);
        cameraOrientation.setY(cameraUpdate.pose.orientation.y);
        cameraOrientation.setZ(cameraUpdate.pose.orientation.z);
        cameraOrientation.setScalar(cameraUpdate.pose.orientation.w);
    }
}

void PSMoveTemplateController::processTemplateList(const flor_ocs_msgs::OCSTemplateList::ConstPtr &list)
{
    template_id_list = list->template_id_list;

    //Find the pose within the list we just got
    for(int i = 0; i < template_id_list.size(); i++)
    {
        if((int)template_id_list[i] == id)
        {
            //Get pose stamped from the array of poses given
            pose = list->pose[i];
            received_pose = true;
        }
    }
}

void PSMoveTemplateController::processObjectSelection(const flor_ocs_msgs::OCSObjectSelection::ConstPtr &obj)
{
    //Get id of object that is selected
    if(obj->id != id)
        received_pose = false;
    id = obj->id;
}

geometry_msgs::PoseStamped PSMoveTemplateController::updatePose(geometry_msgs::PoseStamped p, MoveServerPacket *move_server_packet, float normalized_scale)
{
    //Calculate scale
    //double length = (cam - obj).length();

    ROS_ERROR("%f",cameraUpdate.vfov);

    float rotation_scale = normalized_scale; //* length;
    float position_scale = 0.01 * normalized_scale; //* length; // move position reported in mm

    //Translation- move relative to camera for direction but still translate in world space
    //create a vector representing the offset to be applied to position.
    QVector3D* offset = new QVector3D((move_server_packet->state[0].pos[0] - oldMovePosition.x()) * position_scale,
                                      (move_server_packet->state[0].pos[1] - oldMovePosition.y()) * position_scale,
                                      (move_server_packet->state[0].pos[2] - oldMovePosition.z()) * position_scale);
    //rotate the offset to be relative to camera orientation, can now be applied to position to move relative to camera
    *offset = cameraOrientation.rotatedVector(*offset);

    //Update x, y, and z values
    p.pose.position.x += offset->x();
    p.pose.position.y += offset->y();
    p.pose.position.z += offset->z();

    //Update the rotation
    QQuaternion pre;
    pre.setScalar(p.pose.orientation.w);
    pre.setX(p.pose.orientation.x);
    pre.setY(p.pose.orientation.y);
    pre.setZ(p.pose.orientation.z);

    QQuaternion difference;
    //camera relative rotation
    QQuaternion* rot = new QQuaternion(move_server_packet->state[0].quat[3],move_server_packet->state[0].quat[0],move_server_packet->state[0].quat[1],move_server_packet->state[0].quat[2]);
    //calculate the angle offset
    *rot = rot->conjugate() * oldMoveOrientation;

    //calculate difference between camera orientation and original rotation of object
    //difference of q1 and q2 is  q` = q1^-1 * q2
    difference = cameraOrientation.conjugate() * (pre);
    //set object orientation to camera
    pre = cameraOrientation;
    //apply desired rotation
    pre *= *rot;
    //revert back change of camera rotation to leave object in newly rotated state
    pre *= difference;
    delete(rot);

    p.pose.orientation.w = pre.scalar();
    p.pose.orientation.x = pre.x();
    p.pose.orientation.y = pre.y();
    p.pose.orientation.z = pre.z();

    return p;
}

int PSMoveTemplateController::updateSuccess(MoveServerPacket *move_server_packet)
{
    ROS_ERROR("%f, %f, %f, %d", move_server_packet->state[0].pos[0], move_server_packet->state[0].pos[1], move_server_packet->state[0].pos[2], move_server_packet->state[0].pad.analog_trigger);

    if(received_pose)//Check if a template is selected
    {
        //Only update if trigger is pressed
        if(move_server_packet->state[0].pad.analog_trigger > 0)
        {
            flor_ocs_msgs::OCSTemplateUpdate template_update;

            //Update current pose
            geometry_msgs::PoseStamped p = pose;
            p = updatePose(p, move_server_packet, (float)move_server_packet->state[0].pad.analog_trigger/255.0f);

            //Add to the template_update
            template_update.template_id = id;
            template_update.pose = p;

            //Publish the new pose
            template_update_pub_.publish(template_update);

            cout << "template updated and published" << "\n";
        }

        //Save position and orientation
        oldMovePosition.setX(move_server_packet->state[0].pos[0]);
        oldMovePosition.setY(move_server_packet->state[0].pos[1]);
        oldMovePosition.setZ(move_server_packet->state[0].pos[2]);

        oldMoveOrientation.setX(move_server_packet->state[0].quat[0]);
        oldMoveOrientation.setY(move_server_packet->state[0].quat[1]);
        oldMoveOrientation.setZ(move_server_packet->state[0].quat[2]);
        oldMoveOrientation.setScalar(move_server_packet->state[0].quat[3]);

    }

    return 0;
}


int PSMoveTemplateController::updateFailure(int error)
{
    ROS_ERROR("Error: %d", error);

    return 0;
}


int PSMoveTemplateController::updateCameraSuccess(MoveServerCameraFrameSlicePacket *move_server_camera_frame_slice_packet)
{
    ROS_ERROR("Camera Frame");

    return 0;
}


int PSMoveTemplateController::updateCameraFailure(int error)
{
    ROS_ERROR("Camera Error: %d", error);

    return 0;
}

}

