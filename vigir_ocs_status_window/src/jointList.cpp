#include "jointList.h"
#include <QVBoxLayout>
#include <QRegExp>
#include <ros/ros.h>
#include <QDebug>

jointList::jointList(QWidget *parent) :
    QWidget(parent)
{
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
    joint->setText(0,"back_lbz");
    joints.push_back(joint);
    joint= new QTreeWidgetItem(torso);
    joint->setText(0,"back_mby");
    joints.push_back(joint);
    joint = new QTreeWidgetItem(torso);
    joint->setText(0,"back_ubx");
    joints.push_back(joint);
    joint = new QTreeWidgetItem(torso);
    joint->setText(0,"neck_ay");
    joints.push_back(joint);

    //Left Leg Joints
    joint = new QTreeWidgetItem(left_leg);
    joint->setText(0,"l_leg_uhz");
    joints.push_back(joint);
    joint = new QTreeWidgetItem(left_leg);
    joint->setText(0,"l_leg_mhx");
    joints.push_back(joint);
    joint = new QTreeWidgetItem(left_leg);
    joint->setText(0,"l_leg_lhy");
    joints.push_back(joint);
    joint = new QTreeWidgetItem(left_leg);
    joint->setText(0,"l_leg_kny");
    joints.push_back(joint);
    joint = new QTreeWidgetItem(left_leg);
    joint->setText(0,"l_leg_uay");
    joints.push_back(joint);
    joint = new QTreeWidgetItem(left_leg);
    joint->setText(0,"l_leg_lax");
    joints.push_back(joint);

    //Right Leg Joints
    joint= new QTreeWidgetItem(right_leg);
    joint->setText(0,"r_leg_uhz");
    joints.push_back(joint);
    joint = new QTreeWidgetItem(right_leg);
    joint->setText(0,"r_leg_mhx");
    joints.push_back(joint);
    joint = new QTreeWidgetItem(right_leg);
    joint->setText(0,"r_leg_lhy");
    joints.push_back(joint);
    joint = new QTreeWidgetItem(right_leg);
    joint->setText(0,"r_leg_kny");
    joints.push_back(joint);
    joint = new QTreeWidgetItem(right_leg);
    joint->setText(0,"r_leg_uay");
    joints.push_back(joint);
    joint = new QTreeWidgetItem(right_leg);
    joint->setText(0,"r_leg_lax");
    joints.push_back(joint);

    //Left Arm Joints
    joint = new QTreeWidgetItem(left_arm);
    joint->setText(0,"l_arm_usy");
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
    joint->setText(0,"l_arm_uwy");
    joints.push_back(joint);
    joint = new QTreeWidgetItem(left_arm);
    joint->setText(0,"l_arm_mwx");
    joints.push_back(joint);
    //Right Arm Joints
    joint= new QTreeWidgetItem(right_arm);
    joint->setText(0,"r_arm_usy");
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
    joint->setText(0,"r_arm_uwy");
    joints.push_back(joint);
    joint = new QTreeWidgetItem(right_arm);
    joint->setText(0,"r_arm_mwx");
    joints.push_back(joint);

    warnMin=.75;
    errorMin=.90;

    std::cout << "JointList Widget setup now starting subscribing to Ros topic." << std::endl;
    ros::NodeHandle nh;
    std::string robotInfo;
    nh.getParam("/robot_description",robotInfo);
    processRobotInfo(robotInfo);
    joint_states = nh.subscribe<sensor_msgs::JointState>( "/atlas/joint_states", 2, &jointList::updateList, this );

    //ros::spinOnce();

    timer.start(33, this);
}

void jointList::timerEvent(QTimerEvent *event)
{
    //Spin at beginning of Qt timer callback, so current ROS time is retrieved
    ros::spinOnce();
}

void jointList::processRobotInfo(std::string robotInfo)
{
    std::cout << "Setting up limits from ros param..." << std::endl;

    QRegExp effort("<limit effort=\\\"\\d+.\\d+");
    QRegExp lowLimit("soft_lower_limit=\\\"(\\)*-)\\d+.\\d+");
    QRegExp upLimit("soft_upper_limit=\\\"(\\)*)\\d+.\\d+");
    QRegExp names("<joint name=\\\"([a-z]|_){7,9}");
    QString foo = QString::fromStdString(robotInfo);
    int pos = 0;
    while((pos = effort.indexIn(foo,pos)) != -1)
    {
        effortLimits.push_back((effort.cap(0).remove(0,15)).toDouble());
        pos += effort.matchedLength();
    }
    pos = 0;
    while((pos = lowLimit.indexIn(foo,pos)) != -1)
    {
        downPoseLimit.push_back((lowLimit.cap(0).remove(0,18)).toDouble());
        pos += lowLimit.matchedLength();
    }
    pos = 0;
    while((pos = upLimit.indexIn(foo,pos)) != -1)
    {
        upPoseLimit.push_back((upLimit.cap(0).remove(0,18)).toDouble());
        pos += upLimit.matchedLength();
    }
    pos = 0;
    int cur=0;
    int order[joints.size()];
    while((pos = names.indexIn(foo,pos)) != -1 && cur < 28)
    {

        for(int i=0;i<joints.size();i++)
        {
            if(joints[i]->text(0) == names.cap(0).remove(0,13))
            {
                order[cur] = i;

                i = 99;
            }
        }
        //std::cout << (names.cap(0).remove(0,13)).toStdString() << std::endl;
        pos += names.matchedLength();

        cur++;
    }
    for(int i=0;i<joints.size();i++)
    {
        for(int j =i;j<joints.size();j++)
        {
            if(order[j] == i)
            {
                float temp;
                temp = downPoseLimit[i];
                downPoseLimit[i] = downPoseLimit[j];
                downPoseLimit[j] =temp;

                temp = upPoseLimit[i];
                upPoseLimit[i] = upPoseLimit[j];
                upPoseLimit[j] = temp;

                temp = effortLimits[i];
                effortLimits[i] = effortLimits[j];
                effortLimits[j] = temp;

                temp = order[i];
                order[i] = order[j];
                order[j]=temp;
                j=99;
            }
        }
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
    warn = 0;
    err = 0;
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
        }
        if(joint_states->position[i] <= errorMin*downPoseLimit[i])
        {
            warn--;
            err++;
            joints[i]->setBackgroundColor(0,Qt::red);
            joints[i]->setBackgroundColor(1,Qt::red);
        }

        if(joint_states->position[i] >= warnMin*upPoseLimit[i])
        {
            warn++;
            joints[i]->setBackgroundColor(0,Qt::yellow);
            joints[i]->setBackgroundColor(1,Qt::yellow);
        }
        if(joint_states->position[i] >= errorMin*upPoseLimit[i])
        {
            warn--;
            err++;
            joints[i]->setBackgroundColor(0,Qt::red);
            joints[i]->setBackgroundColor(1,Qt::red);
        }

        if(joint_states->effort[i] >= warnMin*effortLimits[i])
        {
            warn++;
            joints[i]->setBackgroundColor(0,Qt::yellow);
            joints[i]->setBackgroundColor(3,Qt::yellow);
        }
        if(joint_states->effort[i] >= errorMin*effortLimits[i])
        {
            warn--;
            err++;
            joints[i]->setBackgroundColor(0,Qt::red);
            joints[i]->setBackgroundColor(3,Qt::red);
        }

        if(joint_states->effort[i] <= -(warnMin*effortLimits[i]))
        {
            warn++;
            joints[i]->setBackgroundColor(0,Qt::yellow);
            joints[i]->setBackgroundColor(3,Qt::yellow);
        }
        if(joint_states->effort[i] <= -(errorMin*effortLimits[i]))
        {
            warn--;
            err++;
            joints[i]->setBackgroundColor(0,Qt::red);
            joints[i]->setBackgroundColor(3,Qt::red);
        }
    }
}
