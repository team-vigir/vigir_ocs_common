/*
 * Main3DView class implementation.
 *
 * Author: Brian Wright
 *
 * Based on librviz_tutorials.
 *
 */




#include "Controller.h"

#define PS3_BUTTON_SELECT            0
#define PS3_BUTTON_STICK_LEFT        1
#define PS3_BUTTON_STICK_RIGHT       2
#define PS3_BUTTON_START             3
#define PS3_BUTTON_CROSS_UP          4
#define PS3_BUTTON_CROSS_RIGHT       5
#define PS3_BUTTON_CROSS_DOWN        6
#define PS3_BUTTON_CROSS_LEFT        7
#define PS3_BUTTON_REAR_LEFT_2       8
#define PS3_BUTTON_REAR_RIGHT_2      9
#define PS3_BUTTON_REAR_LEFT_1       10
#define PS3_BUTTON_REAR_RIGHT_1      11
#define PS3_BUTTON_ACTION_TRIANGLE   12
#define PS3_BUTTON_ACTION_CIRCLE     13
#define PS3_BUTTON_ACTION_CROSS      14
#define PS3_BUTTON_ACTION_SQUARE     15
#define PS3_BUTTON_PAIRING           16

#define PS3_AXIS_STICK_LEFT_LEFTWARDS    0
#define PS3_AXIS_STICK_LEFT_UPWARDS      1
#define PS3_AXIS_STICK_RIGHT_LEFTWARDS   2
#define PS3_AXIS_STICK_RIGHT_UPWARDS     3
#define PS3_AXIS_BUTTON_CROSS_UP         4
#define PS3_AXIS_BUTTON_CROSS_RIGHT      5
#define PS3_AXIS_BUTTON_CROSS_DOWN       6
#define PS3_AXIS_BUTTON_CROSS_LEFT       7
#define PS3_AXIS_BUTTON_REAR_LEFT_2      8
#define PS3_AXIS_BUTTON_REAR_RIGHT_2     9
#define PS3_AXIS_BUTTON_REAR_LEFT_1      10
#define PS3_AXIS_BUTTON_REAR_RIGHT_1     11
#define PS3_AXIS_BUTTON_ACTION_TRIANGLE  12
#define PS3_AXIS_BUTTON_ACTION_CIRCLE    13
#define PS3_AXIS_BUTTON_ACTION_CROSS     14
#define PS3_AXIS_BUTTON_ACTION_SQUARE    15
#define PS3_AXIS_ACCELEROMETER_LEFT      16
#define PS3_AXIS_ACCELEROMETER_FORWARD   17
#define PS3_AXIS_ACCELEROMETER_UP        18
#define PS3_AXIS_GYRO_YAW                19

namespace vigir_ocs
{
    // Constructor for Controller.  This does most of the work of the class.
    Controller::Controller( QWidget* parent )
        : QWidget(parent)
    {
        ros::NodeHandle nh_out(nh, "/template");        

        //subscribe to list to grab movement data
        template_list_sub = nh_out.subscribe<flor_ocs_msgs::OCSTemplateList>( "list", 1, &Controller::templateListCb ,this );

        //create publisher to update movement data
        template_update_pub = nh_out.advertise<flor_ocs_msgs::OCSTemplateUpdate>( "update", 1, false );

        //subscribe to joystick
        joy_sub = nh.subscribe<sensor_msgs::Joy>( "/joy", 1, &Controller::joyCB ,this );
        //create publisher to joystick
        joy_pub = nh.advertise<sensor_msgs::JoyFeedbackArray>( "/joy/set_feedback", 1, false );

        //subscribe to robot hands
        left_sub = nh.subscribe<geometry_msgs::PoseStamped>("/flor/ghost/pose/left_hand",1,&Controller::leftCB,this );
        right_sub = nh.subscribe<geometry_msgs::PoseStamped>("/flor/ghost/pose/right_hand",1,&Controller::rightCB,this );

        ghost_hand_pub = nh.advertise<flor_ocs_msgs::OCSInteractiveMarkerUpdate>("/flor/ocs/interactive_marker_server/update", 1, false);
       // pose_pub = nh.advertise<>


        timer.start(33, this);

        templateIndex = 0;
        initialPublish = true;
        leftMode = false;
        rightMode = false;
    }

    // Destructor
    Controller::~Controller()
    {

    }



    //sends data to list and updates it
    void Controller::templateListCb(const flor_ocs_msgs::OCSTemplateList::ConstPtr& msg)
    {
        //grab current template List
        temList = *msg;
        //update combo Box initially
        if(initialPublish)
        {
            Q_EMIT updateTemplateComboBox(templateIndex);
            initialPublish = false;
        }
    }

    void Controller::setCameraTransform(int viewId, float x, float y, float z, float rx, float ry, float rz, float w)
    {
       // ROS_ERROR("camera data: %f %f %f %f %f %f %f",x,y,z,rx,ry,rz,w );
        cameraPosition.setX(x);
        cameraPosition.setX(y);
        cameraPosition.setX(z);
        cameraOrientation.setX(rx);
        cameraOrientation.setY(ry);
        cameraOrientation.setZ(rz);
        cameraOrientation.setScalar(w);
       // fromQuaternionToEuler(cameraOrientation);
    }

    void Controller::fromQuaternionToEuler(QQuaternion q1)
    {


//        double test = q1.x()*q1.y() + q1.z()*q1.scalar();
//        if (test > 0.499) { // singularity at north pole
//            heading = 2 * atan2(q1.x(),q1.scalar());
//            attitude = Math.PI/2;
//            bank = 0;
//            return;
//        }
//        if (test < -0.499) { // singularity at south pole
//            heading = -2 * atan2(q1.x(),q1.scalar());
//            attitude = - Math.PI/2;
//            bank = 0;
//            return;
//        }
//        double sqx = q1.x()*q1.x();
//        double sqy = q1.y()*q1.y();
//        double sqz = q1.z()*q1.z();
//        heading = atan2(2*q1.y()*q1.scalar()-2*q1.x()*q1.z() , 1 - 2*sqy - 2*sqz);
//        attitude = asin(2*test);
//        bank = atan2(2*q1.x()*q1.scalar()-2*q1.y()*q1.z() , 1 - 2*sqx - 2*sqz);
    }

    void Controller::joyCB(const sensor_msgs::Joy::ConstPtr& oldJoyData)
    {
        //grab current joystick data
        joy = *oldJoyData;
    }

    void Controller::leftCB(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        leftHand = *msg;
    }
    void Controller::rightCB(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        rightHand = *msg;
    }

    void Controller::handleArms()
    {

    }

    void Controller::leftModeOn()
    {
        leftMode = true;
        rightMode = false;        
    }
    void Controller::rightModeOn()
    {
        rightMode = true;
        leftMode = false;        
    }
    //template mode is default if no arms
    void Controller::templateModeOn()
    {
        leftMode = false;
        rightMode = false;       
    }

    void Controller::handleJoystick()
    {
        float x = 0;
        float y = 0;
        float z = 0;
        float rotX = 0;
        float rotY = 0;
        float rotZ = 0;
        bool tiltLeft = false;
        bool tiltRight = false;

        //read controller values
        //left stick translation
        if(joy.axes[PS3_AXIS_STICK_LEFT_LEFTWARDS] > .5)
            x = -.1;
        if(joy.axes[PS3_AXIS_STICK_LEFT_UPWARDS] > .5)
            y = .1;
        if(joy.axes[PS3_AXIS_STICK_LEFT_LEFTWARDS] < -.5)
            x = .1;
        if(joy.axes[PS3_AXIS_STICK_LEFT_UPWARDS] < -.5)
            y = -.1;
        //dpad
        if(joy.buttons[PS3_BUTTON_CROSS_UP] == 1)
            y = .1;
        if(joy.buttons[PS3_BUTTON_CROSS_LEFT] == 1)
            x = -.1;
        if(joy.buttons[PS3_BUTTON_CROSS_RIGHT] == 1)
            x = .1;
        if(joy.buttons[PS3_BUTTON_CROSS_DOWN] == 1)
            y = -.1;
        //rotation right stick
        if(joy.axes[PS3_AXIS_STICK_RIGHT_UPWARDS] > .5)
            rotY = 5;
        if(joy.axes[PS3_AXIS_STICK_RIGHT_UPWARDS] < -.5)
            rotY = -5;
        if(joy.axes[PS3_AXIS_STICK_RIGHT_LEFTWARDS] > .5)
            rotX = -5;
        if(joy.axes[PS3_AXIS_STICK_RIGHT_LEFTWARDS] < -.5)
            rotX = 5;
        if(joy.buttons[PS3_AXIS_BUTTON_REAR_RIGHT_1])
            z = .1;
        if(joy.buttons[PS3_AXIS_BUTTON_REAR_LEFT_1])
            z = -.1;

        if(joy.axes[PS3_AXIS_ACCELEROMETER_LEFT] < -.1)
            tiltLeft = true;
        else if (joy.axes[PS3_AXIS_ACCELEROMETER_LEFT] > .1)
            tiltRight = true;
        //switch axes on tilt
        if(tiltLeft)
        {
            float tmp = z;
            z = x;
            x = -tmp;

            rotZ = rotX;
            rotX = 0;
        }
        else if(tiltRight)
        {
            float tmp = z;
            z = -x;
            x = tmp;

            rotZ = -rotX;
            rotX = 0;
        }

        QQuaternion * rotation = new QQuaternion(temList.pose[templateIndex].pose.orientation.w,temList.pose[templateIndex].pose.orientation.x,
                                                 temList.pose[templateIndex].pose.orientation.y,temList.pose[templateIndex].pose.orientation.z);
        QVector3D * position = new QVector3D(temList.pose[templateIndex].pose.position.x,temList.pose[templateIndex].pose.position.y,temList.pose[templateIndex].pose.position.z);
        buildTransformation(x,y,z,rotX,rotY,rotZ,rotation,position);

        //create msg to publish
        flor_ocs_msgs::OCSTemplateUpdate msg;
        msg.template_id = temList.template_id_list[templateIndex];

        //copy rotation first
        msg.pose.pose.orientation.x = rotation->x();
        msg.pose.pose.orientation.y = rotation->y();
        msg.pose.pose.orientation.z = rotation->z();
        msg.pose.pose.orientation.w = rotation->scalar();

        //update coordinates based on latest data from list
        msg.pose.pose.position.x = position->x();
        msg.pose.pose.position.y = position->y();
        msg.pose.pose.position.z = position->z();

        //don't publish if sending same data
        if(!compareJoyData())
        {
            //ROS_INFO(" PUBLISHING");
            template_update_pub.publish(msg);
        }
        else
        {
           // ROS_INFO("NOT PUBLISHING");
        }
        //store last joystick state for comparisons
        oldJoy = joy;

        delete(rotation);
        delete(position);
    }

    //compares previous joystick state and current joystick state
    // false if different or should publish
    // true if duplicate or should not publish
    bool Controller::compareJoyData()
    {
        //null check
        if(oldJoy.axes.size() ==0)
            return false; // need to publish once to have old data

        bool axesDup = true;
        bool buttonDup = true;
        //ignore gyro and accel (16-19)
        for(int i=0;i<16;i++)
        {
            if(oldJoy.axes[i] != joy.axes[i] || joy.axes[i]!=0)
                axesDup = false;
        }
        for(int i=0;i<oldJoy.buttons.size();i++)
        {
            if(oldJoy.buttons[i] != joy.buttons[i])
                buttonDup = false;
        }
        //handle buttons on difference only
        if(!buttonDup)
        {
            handleButtons();
        }
        //publish if anything different
        if(!axesDup||!buttonDup)
            return false;
        else
            return true;
    }

    //handles buttons that only actuate once(selecting or changing modes)
    void Controller::handleButtons()
    {
        //only want to send different values
        //change template on X
        if(oldJoy.buttons[PS3_BUTTON_ACTION_CROSS] != joy.buttons[PS3_BUTTON_ACTION_CROSS] && joy.buttons[PS3_BUTTON_ACTION_CROSS]==1)
            changeTemplate();
        if(oldJoy.buttons[PS3_AXIS_BUTTON_ACTION_SQUARE] != joy.buttons[PS3_AXIS_BUTTON_ACTION_SQUARE])
            leftMode = !leftMode;

    }
    //handles rumble
    void Controller::buildJoy()
    {

    }

    //update position and rotation
    void Controller::buildmsg(float posX, float posY , float rotY, float rotX)
    {       
        QQuaternion * rotation;
        QVector3D* position;
        if(leftMode)
        {
            rotation = new QQuaternion(leftHand.pose.orientation.w,leftHand.pose.orientation.x,
                                       leftHand.pose.orientation.y,leftHand.pose.orientation.z);
            position = new QVector3D(leftHand.pose.position.x,leftHand.pose.position.y,leftHand.pose.position.z);
        }
        else
        {
            //build quaternion based on current rotation
            rotation = new QQuaternion(temList.pose[templateIndex].pose.orientation.w,temList.pose[templateIndex].pose.orientation.x,
                                                 temList.pose[templateIndex].pose.orientation.y,temList.pose[templateIndex].pose.orientation.z);
            position = new QVector3D(temList.pose[templateIndex].pose.position.x,temList.pose[templateIndex].pose.position.y,temList.pose[templateIndex].pose.position.z);
        }

        buildTransformation(posX,posY,0,rotX,rotY,0,rotation,position);

        if(leftMode)
        {
            //ROS_INFO("sending robot arm data");
            flor_ocs_msgs::OCSInteractiveMarkerUpdate msg;
            msg.topic = "/l_arm_pose_marker";
            msg.pose.header.frame_id = "/world";
            msg.pose.pose.orientation.x = rotation->x();
            msg.pose.pose.orientation.y = rotation->y();
            msg.pose.pose.orientation.z = rotation->z();
            msg.pose.pose.orientation.w = rotation->scalar();

            msg.pose.pose.position.x = position->x();
            msg.pose.pose.position.y = position->y();
            msg.pose.pose.position.z = position->z();

            ghost_hand_pub.publish(msg);

        }
        else
        {
            //create msg to publish
            flor_ocs_msgs::OCSTemplateUpdate msg;
            msg.template_id = temList.template_id_list[templateIndex];

            //copy rotation first
            msg.pose.pose.orientation.x = rotation->x();
            msg.pose.pose.orientation.y = rotation->y();
            msg.pose.pose.orientation.z = rotation->z();
            msg.pose.pose.orientation.w = rotation->scalar();

            //update coordinates based on latest data from list
            msg.pose.pose.position.x = position->x();
            msg.pose.pose.position.y = position->y();
            msg.pose.pose.position.z = position->z();

            //publish
            template_update_pub.publish(msg);
        }
        //delete(v);
        delete(rotation);
        delete(position);
    }

    //apply rotation and transform to Quaternion and position
    void Controller::buildTransformation(float posX, float posY ,float posZ, float rotX, float rotY,float rotZ, QQuaternion* rotation, QVector3D* position)
    {
        //Translation- move relative to camera for direction but still translate in world space
        //create a vector representing the offset to be applied to position.
        QVector3D* offset = new QVector3D(posX,posY,posZ);
        //rotate the offset to be relative to camera orientation, can now be applied to position to move relative to camera
        *offset = cameraOrientation.rotatedVector(*offset);

       ROS_ERROR("rotation before: %f %f %f %f",rotation->x(),rotation->y(), rotation->z(),rotation->scalar());
        //Camera relative rotation
        rotation->normalize();      
        //build quaternion that stores desired rotation offset
        QQuaternion* r = new QQuaternion();
        *r *= QQuaternion::fromAxisAndAngle(1,0,0,rotX);
        *r *= QQuaternion::fromAxisAndAngle(0,1,0,rotY);
        *r *= QQuaternion::fromAxisAndAngle(0,0,1,rotZ);

        //calculate difference between camera orientation and original rotation of object
        //difference of q1 and q2 is  q` = q^-1 * q2
        QQuaternion difference = cameraOrientation.conjugate() * *rotation;
        //set object orientation to camera
        *rotation = cameraOrientation;
        rotation->normalize();
        ROS_ERROR("rotation camera: %f %f %f %f",rotation->x(),rotation->y(), rotation->z(),rotation->scalar());

        //apply desired rotation
        *rotation *= *r;
        rotation->normalize();
        ROS_ERROR("rotation r: %f %f %f %f",rotation->x(),rotation->y(), rotation->z(),rotation->scalar());
        //revert back change of camera rotation to leave object in newly rotated state
        *rotation *= difference;
        rotation->normalize();
        ROS_ERROR("rotation difference: %f %f %f %f",rotation->x(),rotation->y(), rotation->z(),rotation->scalar());
       // QQuaternion n = cameraOrientation.conjugate()*( *r * cameraOrientation);
       // n *= *rotation;
       // *rotation = n;


           //keep values within reasonable limits to secure additional calculations
       //  rotation->normalize();

         //update coordinates based on latest data from list
         position->setX(position->x()+offset->x());
         position->setY(position->y()+offset->y());
         position->setZ(position->z()+offset->z());
         delete(offset);
       //  delete(r);
}

    void Controller::timerEvent(QTimerEvent *event)
    {
        //joy must be initialized to update joystick data
        if(joy.axes.size() >0)
            handleJoystick();
        // check if ros is still running; if not, just kill the application
        if(!ros::ok())
            qApp->quit();

        //Spin at beginning of Qt timer callback, so current ROS time is retrieved
        ros::spinOnce();
    }

    void Controller::changeTemplate()
    {
        //circular
        if(templateIndex == temList.template_id_list.size() -1)
            templateIndex = 0;
        else
            templateIndex++;        
        //update selected item in comboBox (will update contents unnecessarily and call changeTemplateID redundantly)
        Q_EMIT updateTemplateComboBox(templateIndex);
    }

    void Controller::changeTemplateID(int newIndex)
    {
        //invalid index check
        if(newIndex >= temList.template_id_list.size() || newIndex < 0)
            return;
        templateIndex = newIndex;
    }

    std::vector<std::string> Controller::getTemplateNames()
    {        
        return temList.template_list;
    }

    QQuaternion Controller::rotate(float rotateLeftRight, float rotateUpDown, QQuaternion* rotation)
    {

    //Get Main camera in Use.

    //Gets the world vector space for cameras up vector
    //Vector3 relativeUp = cam.transform.TransformDirection(Vector3.up);
    QVector3D relativeUp(0,1,0);
    relativeUp = cameraOrientation.rotatedVector(relativeUp);
    //Gets world vector for space cameras right vector
    //Vector3 relativeRight = cam.transform.TransformDirection(Vector3.right);
    QVector3D relativeRight(1,0,0);
    relativeRight = cameraOrientation.rotatedVector(relativeRight);
    //Turns relativeUp vector from world to objects local space
    //Vector3 objectRelativeUp = transform.InverseTransformDirection(relativeUp);
    QVector3D objectRelativeUp = -rotation->rotatedVector(relativeUp);
    //Turns relativeRight vector from world to object local space
    //Vector3 objectRelaviveRight = transform.InverseTransformDirection(relativeRight);
    QVector3D objectRelativeRight = -rotation->rotatedVector(relativeRight);


//    rotateBy = Quaternion.AngleAxis(rotateLeftRight / gameObject.transform.localScale.x * sensitivity, objectRelativeUp)
  //  * Quaternion.AngleAxis(-rotateUpDown / gameObject.transform.localScale.x * sensitivity, objectRelaviveRight);


    return QQuaternion::fromAxisAndAngle(objectRelativeUp,rotateLeftRight) * QQuaternion::fromAxisAndAngle(objectRelativeRight,-rotateUpDown);


    }


}
