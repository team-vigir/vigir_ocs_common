#include "jointList.h"
#include <QVBoxLayout>
#include <QRegExp>
#include <ros/ros.h>
#include <urdf/model.h>
#include <QDebug>

jointList::jointList(QWidget *parent) :
    QWidget(parent)
{
    this->setWindowTitle("Joint_Lists");
    this->setMinimumSize(425,120);
    jointTable = new QTreeWidget();
    jointTable->setColumnCount(4);
    QVBoxLayout* main_layout = new QVBoxLayout;

    main_layout->addWidget(jointTable);
    std::cout << "Adding layout..." << std::endl;
    setLayout(main_layout);

    QStringList columns;
    columns.push_back("Joint");
    columns.push_back("Position");
    columns.push_back("Velocity");
    columns.push_back("Effort");

    jointTable->setHeaderLabels(columns);
    jointTable->setColumnWidth(0,150);
    QTreeWidgetItem *torso = new QTreeWidgetItem(jointTable);
    torso->setText(0,"Torso");
    QTreeWidgetItem *left_leg = new QTreeWidgetItem(jointTable);
    left_leg->setText(0,"Left Leg");
    QTreeWidgetItem *right_leg = new QTreeWidgetItem(jointTable);
    right_leg->setText(0,"Right Leg");
    QTreeWidgetItem *left_arm = new QTreeWidgetItem(jointTable);
    left_arm->setText(0,tr("Left Arm"));
    QTreeWidgetItem *right_arm = new QTreeWidgetItem(jointTable);
    right_arm->setText(0,tr("Right Arm"));

    //Torso Joints
    QTreeWidgetItem *joint = new QTreeWidgetItem(torso);
    joint->setText(0,"back_bkz");
    joints.push_back(joint);
    joint= new QTreeWidgetItem(torso);
    joint->setText(0,"back_bky");
    joints.push_back(joint);
    joint = new QTreeWidgetItem(torso);
    joint->setText(0,"back_bkx");
    joints.push_back(joint);
    joint = new QTreeWidgetItem(torso);
    joint->setText(0,"neck_ry");
    joints.push_back(joint);

    //Left Leg Joints
    joint = new QTreeWidgetItem(left_leg);
    joint->setText(0,"l_leg_hpz");
    joints.push_back(joint);
    joint = new QTreeWidgetItem(left_leg);
    joint->setText(0,"l_leg_hpx");
    joints.push_back(joint);
    joint = new QTreeWidgetItem(left_leg);
    joint->setText(0,"l_leg_hpy");
    joints.push_back(joint);
    joint = new QTreeWidgetItem(left_leg);
    joint->setText(0,"l_leg_kny");
    joints.push_back(joint);
    joint = new QTreeWidgetItem(left_leg);
    joint->setText(0,"l_leg_aky");
    joints.push_back(joint);
    joint = new QTreeWidgetItem(left_leg);
    joint->setText(0,"l_leg_akx");
    joints.push_back(joint);

    //Right Leg Joints
    joint= new QTreeWidgetItem(right_leg);
    joint->setText(0,"r_leg_hpz");
    joints.push_back(joint);
    joint = new QTreeWidgetItem(right_leg);
    joint->setText(0,"r_leg_hpx");
    joints.push_back(joint);
    joint = new QTreeWidgetItem(right_leg);
    joint->setText(0,"r_leg_hpy");
    joints.push_back(joint);
    joint = new QTreeWidgetItem(right_leg);
    joint->setText(0,"r_leg_kny");
    joints.push_back(joint);
    joint = new QTreeWidgetItem(right_leg);
    joint->setText(0,"r_leg_aky");
    joints.push_back(joint);
    joint = new QTreeWidgetItem(right_leg);
    joint->setText(0,"r_leg_akx");
    joints.push_back(joint);

    //Left Arm Joints
    joint = new QTreeWidgetItem(left_arm);
    joint->setText(0,"l_arm_shy");
    joints.push_back(joint);
    joint= new QTreeWidgetItem(left_arm);
    joint->setText(0,"l_arm_shx");
    joints.push_back(joint);
    joint = new QTreeWidgetItem(left_arm);
    joint->setText(0,"l_arm_ely");
    joints.push_back(joint);
    joint = new QTreeWidgetItem(left_arm);
    joint->setText(0,"l_arm_elx");
    joints.push_back(joint);
    joint = new QTreeWidgetItem(left_arm);
    joint->setText(0,"l_arm_wry");
    joints.push_back(joint);
    joint = new QTreeWidgetItem(left_arm);
    joint->setText(0,"l_arm_wrx");
    joints.push_back(joint);
    //Right Arm Joints
    joint= new QTreeWidgetItem(right_arm);
    joint->setText(0,"r_arm_shy");
    joints.push_back(joint);
    joint = new QTreeWidgetItem(right_arm);
    joint->setText(0,"r_arm_shx");
    joints.push_back(joint);
    joint= new QTreeWidgetItem(right_arm);
    joint->setText(0,"r_arm_ely");
    joints.push_back(joint);
    joint = new QTreeWidgetItem(right_arm);
    joint->setText(0,"r_arm_elx");
    joints.push_back(joint);
    joint = new QTreeWidgetItem(right_arm);
    joint->setText(0,"r_arm_wry");
    joints.push_back(joint);
    joint = new QTreeWidgetItem(right_arm);
    joint->setText(0,"r_arm_wrx");
    joints.push_back(joint);

    warnMin=.75;
    errorMin=.90;

    std::cout << "JointList Widget setup now starting subscribing to Ros topic." << std::endl;
    std::string robotInfo;

    ros::NodeHandle nh;
    nh.getParam("/robot_description",robotInfo);
    processRobotInfo(robotInfo);

    joint_states = nh_.subscribe<sensor_msgs::JointState>( "/atlas/joint_states", 2, &jointList::updateList, this );

    key_event_sub_ = nh_.subscribe<flor_ocs_msgs::OCSKeyEvent>( "/flor/ocs/key_event", 5, &jointList::processNewKeyEvent, this );

    //ros::spinOnce();

    timer.start(33, this);
}

void jointList::timerEvent(QTimerEvent *event)
{
    // check if ros is still running; if not, just kill the application
    if(!ros::ok())
        qApp->quit();
    
    //Spin at beginning of Qt timer callback, so current ROS time is retrieved
    ros::spinOnce();
}

void jointList::processRobotInfo(std::string robotInfo)
{
    std::cout << "Setting up limits from ros param..." << std::endl;



    QRegExp effort("<limit effort=\\\"[0-9]*.[0-9]*\\\"");
    QRegExp lowLimit("soft_lower_limit=\\\"(-)*[0-9]*.[0-9]*\\\"");
    QRegExp upLimit("soft_upper_limit=\\\"(-)*[0-9]*.[0-9]*\\\"");
    QRegExp names("name=\\\"[a-z_]*\\\"");
    QString robotString = QString::fromStdString(robotInfo);

    for(int i = 0;i < joints.size(); i++)
    {
        int pos = 0;
        while((pos = names.indexIn(robotString,pos)) != -1)
        {
            if(names.cap(0).remove(0,6).toStdString() == joints[i]->text(0).toStdString().append("\""))
            {
                int tempPos = pos;
                QString tempString;
                tempPos = effort.indexIn(robotString,tempPos);
                if(tempPos != -1)
                {
                    tempString = effort.cap(0).remove(0,15);
                    tempString.resize(tempString.size()-1);
                    effortLimits.push_back(tempString.toDouble());
                }
                tempPos = pos;
                tempPos = lowLimit.indexIn(robotString,tempPos);
                if(tempPos != -1)
                {
                    tempString = lowLimit.cap(0).remove(0,18);
                    tempString.resize(tempString.size()-1);
                    downPoseLimit.push_back(tempString.toDouble());
                }
                tempPos = pos;
                pos = robotInfo.size()-100;
                tempPos = upLimit.indexIn(robotString,tempPos);
                if(tempPos != -1)
                {
                    tempString = upLimit.cap(0).remove(0,18);
                    tempString.resize(tempString.size()-1);
                    upPoseLimit.push_back(tempString.toDouble());
                }
            }
            pos += names.matchedLength();
        }


    }
    for(int i=0;i<joints.size();i++)
    {
        std::cout << " Joint["<< i <<"]  limits pos(" << downPoseLimit[i] << ", " << upPoseLimit[i] << ") effort=" << effortLimits[i] << std::endl;
    }
    std::cout << "Finished setting up limits. Now subscribing to joint information...." << std::endl;
}

jointList::~jointList()
{
    //delete ui;
}
int jointList::getNumError()
{
    return err;
}
int jointList::getNumWarn()
{
    return warn;
}

void jointList::updateList( const sensor_msgs::JointState::ConstPtr& joint_states )
{
    // clear joint status messages
    Q_EMIT sendJointData(0,"");

    jointsOkay = true;

    warn = 0;
    err = 0;
    for(int i=0;i<joint_states->name.size(); i++)
        joints[i]->parent()->setBackground(0,Qt::white);
    for(int i=0;i<joint_states->name.size(); i++)
    {
        joints[i]->setText(1,QString::number(joint_states->position[i]));
        joints[i]->setText(2,QString::number(joint_states->velocity[i]));
        joints[i]->setText(3,QString::number(joint_states->effort[i]));
        joints[i]->setBackgroundColor(0,Qt::white);
        joints[i]->setBackgroundColor(1,Qt::white);
        joints[i]->setBackgroundColor(3,Qt::white);

        //std::cout << "p=" << joint_states->position[i] << " v=" << joint_states->velocity[i];
        //std::cout << " e=" << joint_states->effort[i] << " dpl=" << downPoseLimit[i];
        //std::cout << " upl=" << upPoseLimit[i] << " el=" << effortLimits[i] << std::endl;
        if(joint_states->position[i] <= warnMin*downPoseLimit[i])
        {           
            warn++;
            joints[i]->setBackgroundColor(0,Qt::yellow);
            joints[i]->setBackgroundColor(1,Qt::yellow);
            if(joints[i]->parent()->background(0) != Qt::red)
            {
                joints[i]->parent()->setBackgroundColor(0,Qt::yellow);
            }

            jointsOkay = false;
            //notify popup miniJoint widget
            Q_EMIT sendJointData(1,joints[i]->text(0));
        }
        if(joint_states->position[i] <= errorMin*downPoseLimit[i])
        {
            warn--;
            err++;
            joints[i]->setBackgroundColor(0,Qt::red);
            joints[i]->setBackgroundColor(1,Qt::red);
            joints[i]->parent()->setBackgroundColor(0,Qt::red);

            jointsOkay = false;
            Q_EMIT sendJointData(2,joints[i]->text(0));
        }

        if(joint_states->position[i] >= warnMin*upPoseLimit[i])
        {
            warn++;
            joints[i]->setBackgroundColor(0,Qt::yellow);
            joints[i]->setBackgroundColor(1,Qt::yellow);
            if(joints[i]->parent()->background(0) != Qt::red)
            {
                joints[i]->parent()->setBackgroundColor(0,Qt::yellow);
            }

            jointsOkay = false;
            Q_EMIT sendJointData(1,joints[i]->text(0));

        }
        if(joint_states->position[i] >= errorMin*upPoseLimit[i])
        {
            warn--;
            err++;
            joints[i]->setBackgroundColor(0,Qt::red);
            joints[i]->setBackgroundColor(1,Qt::red);
            joints[i]->parent()->setBackgroundColor(0,Qt::red);

            jointsOkay = false;
            Q_EMIT sendJointData(2,joints[i]->text(0));
        }

        if(joint_states->effort[i] >= warnMin*effortLimits[i])
        {
            warn++;
            joints[i]->setBackgroundColor(0,Qt::yellow);
            joints[i]->setBackgroundColor(3,Qt::yellow);
            if(joints[i]->parent()->background(0) != Qt::red)
            {
                joints[i]->parent()->setBackgroundColor(0,Qt::yellow);
            }

            jointsOkay = false;
            Q_EMIT sendJointData(1,joints[i]->text(0));
        }
        if(joint_states->effort[i] >= errorMin*effortLimits[i])
        {
            warn--;
            err++;
            joints[i]->setBackgroundColor(0,Qt::red);
            joints[i]->setBackgroundColor(3,Qt::red);
            joints[i]->parent()->setBackgroundColor(0,Qt::red);

            jointsOkay = false;
            Q_EMIT sendJointData(2,joints[i]->text(0));
        }

        if(joint_states->effort[i] <= -(warnMin*effortLimits[i]))
        {
            warn++;
            joints[i]->setBackgroundColor(0,Qt::yellow);
            joints[i]->setBackgroundColor(3,Qt::yellow);
            if(joints[i]->parent()->background(0) != Qt::red)
            {
                joints[i]->parent()->setBackgroundColor(0,Qt::yellow);
            }

            jointsOkay = false;
            Q_EMIT sendJointData(1,joints[i]->text(0));
        }
        if(joint_states->effort[i] <= -(errorMin*effortLimits[i]))
        {
            warn--;
            err++;
            joints[i]->setBackgroundColor(0,Qt::red);
            joints[i]->setBackgroundColor(3,Qt::red);
            joints[i]->parent()->setBackgroundColor(0,Qt::red);

            jointsOkay = false;
            Q_EMIT sendJointData(2,joints[i]->text(0));
        }

    }
    //notify main view status bar that all joints are fine
    //if(jointsOkay)
    //    Q_EMIT sendJointData(0,"No bad Joints");
}

void jointList::processNewKeyEvent(const flor_ocs_msgs::OCSKeyEvent::ConstPtr &key_event)
{
    // store key state
    if(key_event->state)
        keys_pressed_list_.push_back(key_event->key);
    else
        keys_pressed_list_.erase(std::remove(keys_pressed_list_.begin(), keys_pressed_list_.end(), key_event->key), keys_pressed_list_.end());

    // process hotkeys
    std::vector<int>::iterator key_is_pressed;

    key_is_pressed = std::find(keys_pressed_list_.begin(), keys_pressed_list_.end(), 37);
    /*if(key_event->key == 17 && key_event->state && key_is_pressed != keys_pressed_list_.end()) // ctrl+8
    {
        if(this->isVisible())
        {
            this->hide();
        }
        else
        {
            //this->move(QPoint(key_event->cursor_x+5, key_event->cursor_y+5));
            this->show();
        }
    }*/
}

