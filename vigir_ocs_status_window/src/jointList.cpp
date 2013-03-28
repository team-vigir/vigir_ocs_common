#include "jointList.h"
#include <QVBoxLayout>
#include <ros/ros.h>
#include <QDebug>
#include "rviz/visualization_manager.h"
#include "rviz/render_panel.h"
#include "rviz/display.h"
#include "rviz/frame_manager.h"

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

    render_panel_ = new rviz::RenderPanel();
    manager_ = new rviz::VisualizationManager( render_panel_ );
    render_panel_->initialize( manager_->getSceneManager(), manager_ );
    manager_->initialize();
    manager_->startUpdate();

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

    warnMin=.8;
    errorMin=.95;

    std::cout << "JointList Widget setup now starting subscribing to Ros topic." << std::endl;
    ros::NodeHandle nh;
    std::string robotInfo;
    nh.getParam("/robot_description",robotInfo);
    processRobotInfo(robotInfo);
    joint_states = nh.subscribe<sensor_msgs::JointState>( "/atlas/joint_states", 2, &jointList::updateList, this );

    //ros::spinOnce();
}

void jointList::processRobotInfo(std::string robotInfo)
{




}

jointList::~jointList()
{
    //delete ui;
}
int jointList::getNumError()
{
    return error;
}
int jointList::getNumWarn()
{
    return warn;
}

//void jointList::getLimitsMessage ( const FOO& lmt_msg)
//{
//    limitsMessage = lmt_msg;
//}

void jointList::updateList( const sensor_msgs::JointState::ConstPtr& joint_states )
{
    warn = 0;
    error = 0;
    for(int i=0;i<joint_states->name.size(); i++)
    {
        joints[i]->setText(1,QString::number(joint_states->position[i]));
        joints[i]->setText(2,QString::number(joint_states->velocity[i]));
        joints[i]->setText(3,QString::number(joint_states->effort[i]));
//        if(joint_states->position[i] >= warnMin*limitsMessage->position[i])
//        {
//            warn++;
//            joints[i].setBackgroundColor(0,Qt::yellow);
//            joints[i].setBackgroundColor(1,Qt::yellow);
//        }
//        if(joint_states->position[i] >= errorMin*limitsMessage->position[i])
//        {
//            warn--;
//            error++;
//            joints[i].setBackgroundColor(0,Qt::red);
//            joints[i].setBackgroundColor(1,Qt::red);
//        }

//        if(joint_states->position[i] <= warnMin*limitsMessage->position[i])
//        {
//            warn++;
//            joints[i].setBackgroundColor(0,Qt::yellow);
//            joints[i].setBackgroundColor(1,Qt::yellow);
//        }
//        if(joint_states->position[i] <= errorMin*limitsMessage->position[i])
//        {
//            warn--;
//            error++;
//            joints[i].setBackgroundColor(0,Qt::red);
//            joints[i].setBackgroundColor(1,Qt::red);
//        }

//        if(joint_states->effort[i] >= warnMin*limitsMessage->effort[i])
//        {
//            warn++;
//            joints[i].setBackgroundColor(0,Qt::yellow);
//            joints[i].setBackgroundColor(3,Qt::yellow);
//        }
//        if(joint_states->effort[i] >= errorMin*limitsMessage->effort[i])
//        {
//            warn--;
//            error++;
//            joints[i].setBackgroundColor(0,Qt::red);
//            joints[i].setBackgroundColor(3,Qt::red);
//        }
    }
}
