#include "ps_move_template_controller.h"

#define _USE_MATH_DEFINES
using namespace std;

namespace vigir_ocs
{

void quatToEuler(QQuaternion q1, float& heading, float& attitude, float& bank)
{
    double test = q1.x()*q1.y() + q1.z()*q1.scalar();
    if (test > 0.499) // singularity at north pole
    {
        heading = 2 * atan2(q1.x(),q1.scalar());
        attitude = M_PI/2;
        bank = 0;
        return;
    }
    if (test < -0.499) // singularity at south pole
    {
        heading = -2 * atan2(q1.x(),q1.scalar());
        attitude = - M_PI/2;
        bank = 0;
        return;
    }
    double sqx = q1.x()*q1.x();
    double sqy = q1.y()*q1.y();
    double sqz = q1.z()*q1.z();
    heading = 57.2957795*(atan2(2*q1.y()*q1.scalar()-2*q1.x()*q1.z() , 1 - 2*sqy - 2*sqz));
    attitude = 57.2957795*(asin(2*test));
    bank = 57.2957795*(atan2(2*q1.x()*q1.scalar()-2*q1.y()*q1.z() , 1 - 2*sqx - 2*sqz));
}

PSMoveTemplateController::PSMoveTemplateController()
{

    // get the camera pose and properties
    camera_sub_ = nh_.subscribe<flor_ocs_msgs::OCSCameraTransform>( "/flor/ocs/camera_transform",5,&PSMoveTemplateController::processCameraTransform,this);

    // update the position of its interactive marker
    interactive_marker_update_pub_ = nh_.advertise<flor_ocs_msgs::OCSInteractiveMarkerUpdate>( "/flor/ocs/interactive_marker_server/update", 100, false );
    selected_object_update_sub_ = nh_.subscribe<flor_ocs_msgs::OCSSelectedObjectUpdate>( "/flor/ocs/interactive_marker_server/selected_object_update", 100, &PSMoveTemplateController::processSelectedObjectUpdate,this);
}

PSMoveTemplateController::~PSMoveTemplateController()
{
}

//store information about the selected object
void PSMoveTemplateController::processSelectedObjectUpdate(const flor_ocs_msgs::OCSSelectedObjectUpdate::ConstPtr& msg)
{
    selected_object_topic_ = msg->topic;
    object_pose_ = msg->pose;
}

//copy camera information and store
void PSMoveTemplateController::processCameraTransform(const flor_ocs_msgs::OCSCameraTransform::ConstPtr& msg)
{
    if(msg->widget_name == "MainView" && msg->view_id == 0)
    {
        camera_update_ = *msg;

        //update camera geometry
        camera_position_.setX(camera_update_.pose.position.x);
        camera_position_.setY(camera_update_.pose.position.y);
        camera_position_.setZ(camera_update_.pose.position.z);

        camera_orientation_.setX(camera_update_.pose.orientation.x);
        camera_orientation_.setY(camera_update_.pose.orientation.y);
        camera_orientation_.setZ(camera_update_.pose.orientation.z);
        camera_orientation_.setScalar(camera_update_.pose.orientation.w);
    }
}

geometry_msgs::PoseStamped PSMoveTemplateController::updatePose(geometry_msgs::PoseStamped object_pose, MoveServerPacket *move_server_packet, float normalized_scale)
{
    geometry_msgs::PoseStamped transformed_pose;

    /////////////////////////
    //Calculate scale
    QVector3D cam(camera_position_);
    QVector3D obj(object_pose.pose.position.x,object_pose.pose.position.y,object_pose.pose.position.z);
    double distance = (cam - obj).length();
    //ROS_ERROR("distance: %f   vfov: %f    center distance: %f", distance, camera_update_.vfov, cos(camera_update_.vfov) * distance);
    double scale = (cos(camera_update_.vfov) * distance) * 1.0 / 5.0; // distance=5m is scale 1.0

    float rotation_scale = normalized_scale;// * scale;
    float position_scale = 0.01 * normalized_scale * scale; // move position reported in mm, so scale it down (right now it's 10x real-world scale)

    /////////////////////////
    //Translation- move relative to camera for direction but still translate in world space
    // create a vector representing the offset to be applied to position.
    QVector3D position_offset((move_server_packet->state[0].handle_pos[0] - old_move_position_.x()) * position_scale,
                              (move_server_packet->state[0].handle_pos[1] - old_move_position_.y()) * position_scale,
                              (move_server_packet->state[0].handle_pos[2] - old_move_position_.z()) * position_scale);
    //rotate the offset to be relative to camera orientation, can now be applied to position to move relative to camera
    QVector3D rotated_position_offset = camera_orientation_.rotatedVector(position_offset);

    //Update x, y, and z values
    // NEED TO RESTORE THESE FOR POSITION CONTROL
    if(first_time_)
    {
        transformed_pose.pose.position.x = object_pose.pose.position.x;
        transformed_pose.pose.position.y = object_pose.pose.position.y;
        transformed_pose.pose.position.z = object_pose.pose.position.z;
    }
    else
    {
        transformed_pose.pose.position.x = object_pose.pose.position.x;// + rotated_position_offset.x();
        transformed_pose.pose.position.y = object_pose.pose.position.y;// + rotated_position_offset.y();
        transformed_pose.pose.position.z = object_pose.pose.position.z;// + rotated_position_offset.z();
    }

    /////////////////////////
    //Rotation
    // calculate the relative rotation from the old move orientation, so we only have the difference
    QQuaternion move_orientation(move_server_packet->state[0].quat[3],move_server_packet->state[0].quat[0],move_server_packet->state[0].quat[1],move_server_packet->state[0].quat[2]);
    QQuaternion move_orientation_offset = old_move_orientation_.conjugate() * move_orientation;

    // object orientation
    QQuaternion object_orientation(object_pose.pose.orientation.w,object_pose.pose.orientation.x,object_pose.pose.orientation.y,object_pose.pose.orientation.z);

    camera_orientation_.normalize();
    move_orientation_offset.normalize();
    object_orientation.normalize();

    printf("DEBUG\n");
    //printf(" cam:   %.3f, %.3f, %.3f, %.3f\n",camera_orientation_.scalar(),camera_orientation_.x(),camera_orientation_.y(),camera_orientation_.z());
    //printf(" move1: %.3f, %.3f, %.3f, %.3f\n",move_orientation_offset.scalar(),move_orientation_offset.x(),move_orientation_offset.y(),move_orientation_offset.z());
    //printf(" obj:   %.3f, %.3f, %.3f, %.3f\n",object_orientation.scalar(),object_orientation.x(),object_orientation.y(),object_orientation.z());

    float x,y,z;
    quatToEuler(camera_orientation_,y,z,x);
    printf(" cam:   %.3f, %.3f, %.3f\n",x,y,z);
    quatToEuler(move_orientation_offset,y,z,x);
    printf(" move1: %.3f, %.3f, %.3f\n",x,y,z);
    quatToEuler(object_orientation,y,z,x);
    printf(" obj:   %.3f, %.3f, %.3f\n",x,y,z);

    if(first_time_)
    {
        // now we need to apply the orientation offset relative to the camera view
//        QQuaternion camera_T_move = camera_orientation_ * move_orientation;
//        QQuaternion object_T_camera = last_object_T_move_.conjugate() * camera_orientation_;
//        QQuaternion object_T_move = object_T_camera * camera_T_move;

//        camera_T_object = camera_orientation_.conjugate() * object_orientation;
        degrees_ = 0;
    }
//    QQuaternion rotated_orientation = move_orientation * last_object_T_move_.conjugate();

    ///////
    // old stuff - not quite right, doesn't take starting pose of move into consideration
    //based on http://www.arcsynthesis.org/gltut/Positioning/Tut08%20Camera%20Relative%20Orientation.html
    //new_rotation_offset = camera_orientation^-1 * (rotation_offset * camera_orientation)
    //QQuaternion rotated_orientation_offset = camera_orientation_ * move_orientation_offset * camera_orientation_.conjugate() * object_orientation;
    QQuaternion rotated_orientation_offset = camera_orientation_ * QQuaternion::fromAxisAndAngle(QVector3D(1,0,0),1) * camera_orientation_.conjugate() * object_orientation; // making transform constant

    rotated_orientation_offset = camera_orientation_ * move_orientation * camera_orientation_.conjugate();
    quatToEuler(rotated_orientation_offset,y,z,x);
    printf(" off:   %.3f, %.3f, %.3f\n",x,y,z);

    ///////
    // based on matrices
    // calculate camera x axis
//    QVector3D camera_x_axis = camera_orientation_.rotatedVector(QVector3D(1,0,0));
//    move_orientation_offset = QQuaternion::fromAxisAndAngle(camera_x_axis,degrees_++);//camera_orientation_ * move_orientation_offset;

//    QQuaternion camera_T_new_object = move_orientation_offset * camera_T_object;
//    QQuaternion rotated_orientation_offset = camera_orientation_ * camera_T_new_object;

    ///////
    // based on vectors
    //Gets the world vector space for cameras vectors
    QVector3D camera_x = camera_orientation_.rotatedVector(QVector3D(1,0,0));
    QVector3D camera_y = camera_orientation_.rotatedVector(QVector3D(0,1,0));
    QVector3D camera_z = camera_orientation_.rotatedVector(QVector3D(0,0,1));
    printf(" camera X:   %.3f, %.3f, %.3f\n",camera_x.x(),camera_x.y(),camera_x.z());
    printf(" camera Y:   %.3f, %.3f, %.3f\n",camera_y.x(),camera_y.y(),camera_y.z());
    printf(" camera Z:   %.3f, %.3f, %.3f\n",camera_z.x(),camera_z.y(),camera_z.z());

    QVector3D move_x = move_orientation.rotatedVector(QVector3D(1,0,0));
    QVector3D move_y = move_orientation.rotatedVector(QVector3D(0,1,0));
    QVector3D move_z = move_orientation.rotatedVector(QVector3D(0,0,1));
    printf(" move X:   %.3f, %.3f, %.3f\n",move_x.x(),move_x.y(),move_x.z());
    printf(" move Y:   %.3f, %.3f, %.3f\n",move_y.x(),move_y.y(),move_y.z());
    printf(" move Z:   %.3f, %.3f, %.3f\n",move_z.x(),move_z.y(),move_z.z());


//    //Turns relative vectors from world to objects local space
//    QVector3D object_relative_y = object_orientation.conjugate().rotatedVector(camera_y);
//    QVector3D object_relative_x = object_orientation.conjugate().rotatedVector(camera_x);
//    QVector3D object_relative_z = object_orientation.conjugate().rotatedVector(camera_z);

//    QQuaternion rotated_orientation_offset = QQuaternion::fromAxisAndAngle(object_relative_z,1)
//                                           * QQuaternion::fromAxisAndAngle(object_relative_y,0)
//                                           * QQuaternion::fromAxisAndAngle(object_relative_x,0);

    rotated_orientation_offset.normalize();

    // DEBUG
//    ROS_ERROR("DEBUG");
//    float x,y,z;
//    quatToEuler(object_orientation,y,x,z);
//    ROS_ERROR(" obj: %.3f, %.3f, %.3f",x,y,z);
//    quatToEuler(move_orientation_offset,y,x,z);
//    ROS_ERROR(" off: %.3f, %.3f, %.3f",x,y,z);
//    quatToEuler(rotated_orientation_offset,y,x,z);
//    ROS_ERROR(" rot: %.3f, %.3f, %.3f",x,y,z);
    // DEBUG

    QQuaternion transformed_orientation;
    //if(first_time_)
    //    transformed_orientation = object_orientation;
    //else
        transformed_orientation = rotated_orientation_offset;

    transformed_pose.pose.orientation.w = transformed_orientation.scalar();
    transformed_pose.pose.orientation.x = transformed_orientation.x();
    transformed_pose.pose.orientation.y = transformed_orientation.y();
    transformed_pose.pose.orientation.z = transformed_orientation.z();

    object_pose_.pose.orientation.w = transformed_orientation.scalar();
    object_pose_.pose.orientation.x = transformed_orientation.x();
    object_pose_.pose.orientation.y = transformed_orientation.y();
    object_pose_.pose.orientation.z = transformed_orientation.z();

    /////////////////////////
    //Header
    transformed_pose.header = object_pose.header;
    transformed_pose.header.stamp = ros::Time::now();

    first_time_ = false;

    return transformed_pose;
}

int PSMoveTemplateController::updateSuccess(MoveServerPacket *move_server_packet)
{
    //ROS_ERROR("updateSuccess: %f, %f, %f, %d", move_server_packet->state[0].pos[0], move_server_packet->state[0].pos[1], move_server_packet->state[0].pos[2], move_server_packet->state[0].pad.analog_trigger);

    //Only update if trigger is pressed and there is an object selected
    if(move_server_packet->state[0].pad.analog_trigger > 0 && selected_object_topic_ != "")
    {
        if(last_analog_trigger_ == 0)
            first_time_ = true;

        // create an interactive marker update message using that string
        flor_ocs_msgs::OCSInteractiveMarkerUpdate cmd;

        //Update current pose
        geometry_msgs::PoseStamped p;
        p = updatePose(object_pose_, move_server_packet, /*(float)move_server_packet->state[0].pad.analog_trigger/255.0f*/1.0f);

        //Add to the template_update
        cmd.topic = selected_object_topic_;
        cmd.pose = p;
        cmd.pose.header.frame_id = "/world";
        cmd.pose.header.stamp = ros::Time::now();
        cmd.pose_mode = flor_ocs_msgs::OCSInteractiveMarkerUpdate::ABSOLUTE;
        cmd.update_mode = flor_ocs_msgs::OCSInteractiveMarkerUpdate::SET_POSE;

        //Publish the new pose
        interactive_marker_update_pub_.publish(cmd);

        cout << "object pose updated and published" << "\n";
    }

    // save last trigger value
    last_analog_trigger_ = move_server_packet->state[0].pad.analog_trigger;

    //Save position and orientation
    old_move_position_.setX(move_server_packet->state[0].handle_pos[0]);
    old_move_position_.setY(move_server_packet->state[0].handle_pos[1]);
    old_move_position_.setZ(move_server_packet->state[0].handle_pos[2]);

    old_move_orientation_.setX(move_server_packet->state[0].quat[0]);
    old_move_orientation_.setY(move_server_packet->state[0].quat[1]);
    old_move_orientation_.setZ(move_server_packet->state[0].quat[2]);
    old_move_orientation_.setScalar(move_server_packet->state[0].quat[3]);

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

