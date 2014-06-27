/*
 * Main3DView class implementation.
 *
 * Author: Brian Wright
 *
 * Based on librviz_tutorials.
 *
 */




#include "Controller.h"

//buttons
#define XBOX_A                          0
#define XBOX_B                          1
#define XBOX_X                          2
#define XBOX_Y                          3
#define XBOX_LB                         4
#define XBOX_RB                         5
#define XBOX_BACK                       6
#define XBOX_START                      7
#define XBOX_POWER                      8
#define XBOX_BUTTON_STICK_LEFT          9
#define XBOX_BUTTON_STICK_RIGHT         10

//axis
#define XBOX_AXIS_STICK_LEFT_LEFTWARDS   0
#define XBOX_AXIS_STICK_LEFT_UPWARDS     1
#define XBOX_LT                          2
#define XBOX_AXIS_STICK_RIGHT_LEFTWARDS  3
#define XBOX_AXIS_STICK_RIGHT_UPWARDS    4
#define XBOX_RT                          5
#define XBOX_CROSS_LEFT                  6
#define XBOX_CROSS_DOWN                  7



namespace vigir_ocs
{
    // Constructor for Controller.  This does most of the work of the class.
    Controller::Controller( QWidget* parent )
        : QWidget(parent)
    {
        ros::NodeHandle nh_out(nh, "/template");

        joystick_modes_sub = nh.subscribe<flor_ocs_msgs::OCSJoystick>("/flor/ocs/joystick",1,&Controller::modeCb,this);

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

        camera_sub = nh.subscribe<flor_ocs_msgs::OCSCameraTransform>( "flor/ocs/camera_transform",1,&Controller::cameraCb,this);

        timer.start(33, this);

        templateIndex = 0;
        initialPublish = true;

        //camera mode and template mode by default
        leftMode = false;
        rightMode = false;
        worldMode = false;
        objectMode = false;

        //allows joystick to keep track of mode
        currentManipulationController = 0;
    }

    // Destructor
    Controller::~Controller()
    {

    }

    void Controller::timerEvent(QTimerEvent *event)
    {
        // check if ros is still running; if not, just kill the application
        if(!ros::ok())
            qApp->quit();

        //Spin at beginning of Qt timer callback, so current ROS time is retrieved
        ros::spinOnce();

        //joy must be initialized to update joystick data
        if(joy.axes.size() >0)
            handleJoystick();
    }

    void Controller::modeCb(const flor_ocs_msgs::OCSJoystick::ConstPtr& msg)
    {
        joyModes = *msg;
        //set current modes based on subscribed data
        setObjectMode(joyModes.objectMode);
        setManipulation(joyModes.manipulationMode);
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

    //copy camera information and store
    void Controller::cameraCb(const flor_ocs_msgs::OCSCameraTransform::ConstPtr& msg)
    {
        cameraUpdate = *msg;
        if(cameraUpdate.widget_name == "MainView" && cameraUpdate.view_id == 0)
        {
            //update camera geometry
            cameraPosition.setX(cameraUpdate.pose.position.x);
            cameraPosition.setX(cameraUpdate.pose.position.y);
            cameraPosition.setX(cameraUpdate.pose.position.z);
            cameraOrientation.setX(cameraUpdate.pose.orientation.x);
            cameraOrientation.setY(cameraUpdate.pose.orientation.y);
            cameraOrientation.setZ(cameraUpdate.pose.orientation.z);
            cameraOrientation.setScalar(cameraUpdate.pose.orientation.w);
        }
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

    //set which object to be manipulated
    //for changing on subscribed data
    void Controller::setObjectMode(int mode)
    {
        //ROS_ERROR("object mode %d",mode);
        switch (mode)
        {
        case 0:
            //template mode is default if no arms
            leftMode = false;
            rightMode = false;
            break;
        case 1:
            leftMode = true;
            rightMode = false;
            break;
        case 2:
            rightMode = true;
            leftMode = false;
            break;
        }
    }

    void Controller::setManipulation(int mode)
    {
        ROS_ERROR("manipulation mode %d",mode);        
        switch(mode)
        {
        case 2://camera
            worldMode = false;
            objectMode = false;
            break;
        case 1: // world
            objectMode = false;
            worldMode = true;
            break;
        case 0: // object
            worldMode = false;
            objectMode = true;
            break;
        }
    }

    void Controller::changeManipulationController()
    {
        //change modes in circular fashion
        if(currentManipulationController == 2)
            currentManipulationController = 0;
        else
            currentManipulationController++;

        setManipulation(currentManipulationController);
    }


    void Controller::handleJoystick()
    {
        //null check for templates
        if(temList.template_id_list.size() == 0 && !leftMode && !rightMode)
            return;        
        float x = 0;
        float y = 0;
        float z = 0;
        float rotX = 0;
        float rotY = 0;
        float rotZ = 0;

        //read controller values
        //left stick translation
        if(joy.axes[XBOX_AXIS_STICK_LEFT_LEFTWARDS] > .5)
            x = -.1;
        if(joy.axes[XBOX_AXIS_STICK_LEFT_UPWARDS] > .5)
            y = .1;
        if(joy.axes[XBOX_AXIS_STICK_LEFT_LEFTWARDS] < -.5)
            x = .1;
        if(joy.axes[XBOX_AXIS_STICK_LEFT_UPWARDS]  <-.5)
            y = -.1;
        //dpad
        if(joy.axes[XBOX_CROSS_DOWN] > 0)
            y = .1;
        if(joy.axes[XBOX_CROSS_LEFT] < 0)
            x = -.1;
        if(joy.axes[XBOX_CROSS_LEFT] > 0)
            x = .1;
        if(joy.axes[XBOX_CROSS_DOWN] < 0)
            y = -.1;
        //rotation right stick
        if(joy.axes[XBOX_AXIS_STICK_RIGHT_UPWARDS] > .5)
            rotY = 5;
        if(joy.axes[XBOX_AXIS_STICK_RIGHT_UPWARDS] < -.5)
            rotY = -5;
        if(joy.axes[XBOX_AXIS_STICK_RIGHT_LEFTWARDS] > .5)
            rotX = -5;
        if(joy.axes[XBOX_AXIS_STICK_RIGHT_LEFTWARDS] < -.5)
            rotX = 5;
        if(joy.buttons[XBOX_LB] == 1)
            z = -.1;
        if(joy.buttons[XBOX_RB] == 1)
            z = .1;
        if(joy.buttons[XBOX_A] == 1)
        {
            //set to template mode if either arm mode is true, otherwise switch current template
            if(!leftMode && !rightMode)
                changeTemplate();
            else
                setObjectMode(0);
        }
        if(joy.buttons[XBOX_X] == 1) //left mode on
            setObjectMode(1);
        if(joy.buttons[XBOX_B] == 1)//right mode on
            setObjectMode(2);
        if(joy.buttons[XBOX_Y] == 1)
            changeManipulationController();

        QQuaternion * rotation;
        QVector3D * position;

        if(leftMode)
        {
            rotation = new QQuaternion(leftHand.pose.orientation.w,leftHand.pose.orientation.x,
                                       leftHand.pose.orientation.y,leftHand.pose.orientation.z);
            position = new QVector3D(leftHand.pose.position.x,leftHand.pose.position.y,leftHand.pose.position.z);
        }
        else if(rightMode)
        {
            ///placeholder
            rotation = new QQuaternion(leftHand.pose.orientation.w,leftHand.pose.orientation.x,
                                       leftHand.pose.orientation.y,leftHand.pose.orientation.z);
            position = new QVector3D(leftHand.pose.position.x,leftHand.pose.position.y,leftHand.pose.position.z);
        }
        else //template manipulation
        {
            rotation = new QQuaternion(temList.pose[templateIndex].pose.orientation.w,temList.pose[templateIndex].pose.orientation.x,
                                                 temList.pose[templateIndex].pose.orientation.y,temList.pose[templateIndex].pose.orientation.z);
            position = new QVector3D(temList.pose[templateIndex].pose.position.x,temList.pose[templateIndex].pose.position.y,temList.pose[templateIndex].pose.position.z);
        }
        buildTransformation(x,y,z,rotX,rotY,rotZ,rotation,position);


        flor_ocs_msgs::OCSTemplateUpdate msg;

        if(leftMode)
        {

        }
        else if(rightMode)
        {

        }
        else
        {
            //create msg to publish

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
        }
        //don't publish if sending same data
        if(!compareJoyData())
        {            
            ROS_ERROR(" PUBLISHING ");
            if(leftMode)
            {
            }
            else if(rightMode)
            {
            }
            else
            {
                template_update_pub.publish(msg);
            }
        }
        else
        {
            ROS_ERROR("NOT PUBLISHING");
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

        for(int i=0;i<oldJoy.axes.size();i++)
        {
            if(oldJoy.axes[i] != joy.axes[i] || joy.axes[i] > 0.5 || joy.axes[i] < -0.5 )
            {
                //bumpers are 1 by default
                if(joy.axes[XBOX_LT] != 1 || joy.axes[XBOX_RT] != 1)
                    axesDup = false;
            }
        }
        for(int i=0;i<oldJoy.buttons.size();i++)
        {
            if(oldJoy.buttons[i] != joy.buttons[i])
            {
                //ignore rb and lb as they should always write if 1
                if(joy.buttons[XBOX_LB] == 1 || joy.buttons[XBOX_RB] == 1)
                {
                }
                else
                {
                    buttonDup = false;
                }
            }
        }
        //ROS_ERROR("dups axes: %d button %d",axesDup,buttonDup);
        //handle buttons on difference only
        if(!buttonDup)
        {
            handleButtons();
        }
        //publish if anything different or needs to be written(like a constant direction)
        if(!axesDup||!buttonDup)
            return false;
        else
            return true;


    }

    //handles buttons that only actuate once(selecting or changing modes)
    void Controller::handleButtons()
    {
        //only want to send different values
        //change template on A
        //oldJoy.buttons[XBOX_A] != joy.buttons[XBOX_A] &&

        //oldJoy.buttons[XBOX_X] != joy.buttons[XBOX_X] &&
//                if(joy.buttons[XBOX_A] == 1)
//                {
//                    //set to template mode if either arm mode is true, otherwise switch current template
//                    if(!leftMode && !rightMode)
//                        changeTemplate();
//                    else
//                        setObjectMode(0);
//                }
//                if(joy.buttons[XBOX_X] == 1); //left mode on
//                    setObjectMode(1);
//                if(joy.buttons[XBOX_B] == 1);//right mode on
//                    setObjectMode(2);

    }
    //handles rumble
    void Controller::buildJoy()
    {

    }

    //update position and rotation and publish
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
        else if (rightMode)
        {
            //NOT VALID .. TO BE IMPLEMENTED
            // to avoid seg fault on deleting rotation and position
            rotation = new QQuaternion(leftHand.pose.orientation.w,leftHand.pose.orientation.x,
                                       leftHand.pose.orientation.y,leftHand.pose.orientation.z);
            position = new QVector3D(leftHand.pose.position.x,leftHand.pose.position.y,leftHand.pose.position.z);
           ///////

        }
        else //moving templates
        {
            if(temList.template_id_list.size()==0)
                return;
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
        else if (rightMode)
        {
            //TO BE IMPLEMENTED
        }
        else //template mode
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

        rotation->normalize();
        //build quaternion that stores desired rotation offset
        QQuaternion* r = new QQuaternion();
        *r *= QQuaternion::fromAxisAndAngle(0,1,0,rotY);
        *r *= QQuaternion::fromAxisAndAngle(1,0,0,rotX);
        *r *= QQuaternion::fromAxisAndAngle(0,0,1,rotZ);
       // r->normalize();
        QQuaternion difference;

        if(worldMode) //world relative rotation
        {
            QQuaternion* identity = new QQuaternion();
            difference = identity->conjugate() * *rotation;
            *rotation = *identity;
            *rotation *= *r;
            *rotation *= difference;
            delete(identity);
        }
        else if(objectMode) //object relative rotation
        {
            *rotation *= *r;
        }
        else //Camera relative rotation
        {
            QQuaternion* rot = new QQuaternion();
            *rot *= QQuaternion::fromAxisAndAngle(0,1,0,rotY); //more intuitive for camera if reversed
            *rot *= QQuaternion::fromAxisAndAngle(1,0,0,-rotX);
            *rot *= QQuaternion::fromAxisAndAngle(0,0,1,rotZ);
            //calculate difference between camera orientation and original rotation of object
            //difference of q1 and q2 is  q` = q^-1 * q2
            difference = cameraOrientation.conjugate() * *rotation;
            //set object orientation to camera
            *rotation = cameraOrientation;
            //apply desired rotation
            *rotation *= *rot;
            //revert back change of camera rotation to leave object in newly rotated state
            *rotation *= difference;
            delete(rot);
        }
        //rotation->normalize();

        if(worldMode)
        {
            position->setX(position->x()+posX);
            position->setY(position->y()+posY);
            position->setZ(position->z()+posZ);
        }
        else if(objectMode)//object relative translation
        {
            QVector3D* objectOffset = new QVector3D(posX,posY,posZ);
            *objectOffset = rotation->rotatedVector(*objectOffset);

            position->setX(position->x()+objectOffset->x());
            position->setY(position->y()+objectOffset->y());
            position->setZ(position->z()+objectOffset->z());
            delete(objectOffset);
        }
        else // camera relative
        {
            //update coordinates based on latest data from list
            position->setX(position->x()+offset->x());
            position->setY(position->y()+offset->y());
            position->setZ(position->z()+offset->z());

        }
         delete(offset);
         delete(r);
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

}
