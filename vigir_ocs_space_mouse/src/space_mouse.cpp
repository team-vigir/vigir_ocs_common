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
    template_list_sub_ = nh_.subscribe<flor_ocs_msgs::OCSTemplateList>("/template/list", 5, &SpaceMouse::processTemplateList, this);

    // get object that is selected
    select_object_sub_ = nh_.subscribe<flor_ocs_msgs::OCSObjectSelection>( "/flor/ocs/object_selection", 5, &SpaceMouse::processObjectSelection, this );

    //Initialize the the subscriber for the joy topic
    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("/spacenav/joy", 10, &SpaceMouse::joyCallback, this);

    // update template position
    template_update_pub_  = nh_.advertise<flor_ocs_msgs::OCSTemplateUpdate>( "/template/update", 1, false );

    recieved_pose = false;
    cout << "SpaceMouse created";

    camera_sub = nh_.subscribe<flor_ocs_msgs::OCSCameraTransform>( "/flor/ocs/camera_transform",5,&SpaceMouse::cameraCb,this);

    camera_pub = nh_.advertise<flor_ocs_msgs::OCSCameraTransform>("/flor/ocs/set_camera_transform", 1, false);
    //ROS_ERROR("PRINT TEST");


}

SpaceMouse::~SpaceMouse()
{
}

//copy camera information and store
void SpaceMouse::cameraCb(const flor_ocs_msgs::OCSCameraTransform::ConstPtr& msg)
{
    cameraUpdate = *msg;

    if(cameraUpdate.widget_name == "MainView" && cameraUpdate.view_id == 0)
    {
        //ROS_ERROR("CAMERA CALLBACK");
        cout << "CAMERA CALLBACK";

        //update camera geometry
        cameraPosition.setX(cameraUpdate.pose.position.x);
        cameraPosition.setY(cameraUpdate.pose.position.y);
        cameraPosition.setZ(cameraUpdate.pose.position.z);

        pose.pose.position.x = cameraUpdate.pose.position.x;
        pose.pose.position.y = cameraUpdate.pose.position.y;
        pose.pose.position.z = cameraUpdate.pose.position.z;

        recieved_pose = true;

        cameraOrientation.setX(cameraUpdate.pose.orientation.x);
        cameraOrientation.setY(cameraUpdate.pose.orientation.y);
        cameraOrientation.setZ(cameraUpdate.pose.orientation.z);
        cameraOrientation.setScalar(cameraUpdate.pose.orientation.w);
    }
}

void SpaceMouse::processTemplateList(const flor_ocs_msgs::OCSTemplateList::ConstPtr &list)
{
    template_id_list = list->template_id_list;

    //cout << "Templatelist recieved HEHEHHEHEHEHEHEH\n";
    //cout << "Current id: " << id << "\n";
    //Find the pose within the list we just got
    for(int i = 0; i < template_id_list.size(); i++)
    {
        //cout << "id at position " << i << ": " << (int)template_id_list[i] << "\n";
        if((int)template_id_list[i] == id)
        {
            //Get pose stamped from the array of poses given
            //cout << "Correct pose found!\n";
            //pose = list->pose[i];
            cout << "X: " << pose.pose.position.x << "\n";
            cout << "Y: " << pose.pose.position.y << "\n";
            cout << "Z: " << pose.pose.position.z << "\n";
            //recieved_pose = true;
        }
    }
}

void SpaceMouse::processObjectSelection(const flor_ocs_msgs::OCSObjectSelection::ConstPtr &obj)
{
    //Get id of object that is selected
    //cout << "id recieved\n";

    if(obj->id != id)
        recieved_pose = false;
    id = obj->id;

    //cout << "id set to: " << id << "\n";

}

void SpaceMouse::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    //cout << "joy update\n";
    //cout<< "value of recieved_pose: " << recieved_pose << "\n";

    if(recieved_pose)//Check if a template is selected
    {
        flor_ocs_msgs::OCSTemplateUpdate template_update;

        flor_ocs_msgs::OCSCameraTransform update = cameraUpdate;

        geometry_msgs::PoseStamped p = pose;

        //cout << "Current x value = " << p.pose.position.x << "\n";
        //cout << "Adding " << joy->axes[1] << "to x value\n";

        //Calculate scale
        QVector3D cam = QVector3D(cameraPosition.x(), cameraPosition.y(), cameraPosition.z());
        QVector3D obj = QVector3D(p.pose.position.x, p.pose.position.y, p.pose.position.z);
        /*cout << "Object:" << "\n";
        cout << "X: " << p.pose.position.x << "\n";
        cout << "Y: " << p.pose.position.y << "\n";
        cout << "Z: " << p.pose.position.z << "\n";
        cout << "Camera:" << "\n";
        cout << "X: " << p.pose.position.x << "\n";
        cout << "Y: " << p.pose.position.y << "\n";
        cout << "Z: " << p.pose.position.z << "\n";*/

        double length = (cam - obj).length();
        cout << "Length: " << length << "\n";

        float rotation_scale = 0.05 * length;
        float scale = 0.005 * length;
        //Translation- move relative to camera for direction but still translate in world space
        //create a vector representing the offset to be applied to position.
        QVector3D* offset = new QVector3D(joy->axes[1] * scale *-1,joy->axes[0] * scale,joy->axes[2] * scale);
        //rotate the offset to be relative to camera orientation, can now be applied to position to move relative to camera
        *offset = cameraOrientation.rotatedVector(*offset);


        //Update x, y, and z values



        p.pose.position.x += offset->x();
        p.pose.position.y += offset->y();
        p.pose.position.z += offset->z();

        //Update the rotation


        cout << "Original W:" << p.pose.orientation.w << "\n";

        QQuaternion pre;
        pre.setScalar(p.pose.orientation.w);
        pre.setX(p.pose.orientation.x);
        pre.setY(p.pose.orientation.y);
        pre.setZ(p.pose.orientation.z);
        //SpaceMouse::Vector v = convertToEuler(pre);

        QQuaternion difference;
        //camera relative rotation
        QQuaternion* rot = new QQuaternion();                  //convert from radians to degrees * 180/pi
        if(std::abs(joy->axes[3]) > 0.1)
            *rot *= QQuaternion::fromAxisAndAngle(0,1,0,(joy->axes[3]*rotation_scale)* 57.2957795131);
        else
            *rot *= QQuaternion::fromAxisAndAngle(0,1,0,(0)* 57.2957795131);
        if(std::abs(joy->axes[4]) > 0.1)
            *rot *= QQuaternion::fromAxisAndAngle(1,0,0,-(joy->axes[4]*rotation_scale)* 57.2957795131); //more intuitive for camera if reversed
        else
            *rot *= QQuaternion::fromAxisAndAngle(1,0,0,-(0)* 57.2957795131); //more intuitive for camera if reversed
        if(std::abs(joy->axes[5]) > 0.1)
            *rot *= QQuaternion::fromAxisAndAngle(0,0,1,(joy->axes[5]*rotation_scale)* 57.2957795131);
        else
           *rot *= QQuaternion::fromAxisAndAngle(0,0,1,(0)* 57.2957795131);
        //*rot *= QQuaternion::fromAxisAndAngle(1,0,0,-(joy->axes[4]*rotation_scale)* 57.2957795131); //more intuitive for camera if reversed
        //*rot *= QQuaternion::fromAxisAndAngle(0,0,1,(joy->axes[5]*rotation_scale)* 57.2957795131);

        cout << "rot x:" << rot->x() << "\n";
        cout << "rot y:" << rot->y() << "\n";
        cout << "rot z:" << rot->z() << "\n";

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


//        v.x += joy->axes[3]*scale;
//        v.y += joy->axes[4]*scale;
//        v.z += joy->axes[5]*scale;
        //Quaternion q = convertToQuaternion(v.x, v.y, v.z);
        p.pose.orientation.w = pre.scalar();
        p.pose.orientation.x = pre.x();
        p.pose.orientation.y = pre.y();
        p.pose.orientation.z = pre.z();

        //Add to the template_update
        if(recieved_pose)
        {
            template_update.template_id = id;
            template_update.pose = p;
            cout << "New x rotation" << template_update.pose.pose.orientation.x << "\n";
            cout << "New x value = " << template_update.pose.pose.position.x << "\n";
            cout << "New W: " << template_update.pose.pose.orientation.w << "\n";

            //Publish the new pose
            //template_update_pub_.publish(template_update);

            update.pose.position.x = p.pose.position.x;
            update.pose.position.y = p.pose.position.y;
            update.pose.position.z = p.pose.position.z;

            camera_pub.publish(update);

            cout << "template updated and published" << "\n";
        }

    }
}
QQuaternion SpaceMouse::convertToQuaternion(double heading, double attitude, double bank)
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

    double w =c1c2*c3 - s1s2*s3;
    double x =c1c2*s3 + s1s2*c3;
    double y =s1*c2*c3 + c1*s2*s3;
    double z =c1*s2*c3 - s1*c2*s3;

    QQuaternion q = QQuaternion(w, x, y , z);

    return q;
}


SpaceMouse::Vector SpaceMouse::convertToEuler(QQuaternion q1)
{
    double heading, attitude, bank;

    double test = q1.x()*q1.y() + q1.z()*q1.scalar();
        if (test > 0.499) { // singularity at north pole
            heading = 2 * atan2(q1.x(),q1.scalar());
            attitude = M_PI/2;
            bank = 0;
            Vector v;
            v.x = heading;
            v.y = attitude;
            v.z = bank;
            return v;
        }
        else if (test < -0.499) { // singularity at south pole
            heading = -2 * atan2(q1.x(),q1.scalar());
            attitude = - M_PI/2;
            bank = 0;
            Vector v;
            v.x = heading;
            v.y = attitude;
            v.z = bank;
            return v;
        }
        double sqx = q1.x()*q1.x()    ;
        double sqy = q1.y()*q1.y();
        double sqz = q1.z()*q1.z();
        heading = atan2(2*q1.y()*q1.scalar()-2*q1.x()*q1.z() , 1 - 2*sqy - 2*sqz);
        attitude = asin(2*test);
        bank = atan2(2*q1.x()*q1.scalar()-2*q1.y()*q1.z() , 1 - 2*sqx - 2*sqz);

        Vector v;
        v.x = heading;
        v.y = attitude;
        v.z = bank;
    return v;
}

}
