#include "jointList.h"
#include <QVBoxLayout>
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
    QTreeWidgetItem *joints[28];
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
    joints[0] = new QTreeWidgetItem(torso);
    joints[0]->setText(0,"back_lbz");
    joints[1]= new QTreeWidgetItem(torso);
    joints[1]->setText(0,"back_mby");
    joints[2] = new QTreeWidgetItem(torso);
    joints[2]->setText(0,"back_ubx");

    joints[3] = new QTreeWidgetItem(torso);
    joints[3]->setText(0,"neck_ay");

	//Left Leg Joints
    joints[4] = new QTreeWidgetItem(left_leg);
    joints[4]->setText(0,"l_leg_uhz");
    joints[5] = new QTreeWidgetItem(left_leg);
    joints[5]->setText(0,"l_leg_mhx");
    joints[6] = new QTreeWidgetItem(left_leg);
    joints[6]->setText(0,"l_leg_lhy");
    joints[7] = new QTreeWidgetItem(left_leg);
    joints[7]->setText(0,"l_leg_kny");
    joints[8] = new QTreeWidgetItem(left_leg);
    joints[8]->setText(0,"l_leg_uay");
    joints[9] = new QTreeWidgetItem(left_leg);
    joints[9]->setText(0,"l_leg_lax");

	//Right Leg Joints
    joints[10]= new QTreeWidgetItem(right_leg);
    joints[10]->setText(0,"r_leg_uhz");
    joints[11] = new QTreeWidgetItem(right_leg);
    joints[11]->setText(0,"r_leg_mhx");
    joints[12] = new QTreeWidgetItem(right_leg);
    joints[12]->setText(0,"r_leg_lhy");
    joints[13] = new QTreeWidgetItem(right_leg);
    joints[13]->setText(0,"r_leg_kny");
    joints[14] = new QTreeWidgetItem(right_leg);
    joints[14]->setText(0,"r_leg_uay");
    joints[15] = new QTreeWidgetItem(right_leg);
    joints[15]->setText(0,"r_leg_lax");

	//Left Arm Joints
    joints[16] = new QTreeWidgetItem(left_arm);
    joints[16]->setText(0,"l_arm_usy");
    joints[17]= new QTreeWidgetItem(left_arm);
    joints[17]->setText(0,"l_arm_shx");
    joints[18] = new QTreeWidgetItem(left_arm);
    joints[18]->setText(0,"l_arm_ely");
    joints[19] = new QTreeWidgetItem(left_arm);
    joints[19]->setText(0,"l_arm_elx");
    joints[20] = new QTreeWidgetItem(left_arm);
    joints[20]->setText(0,"l_arm_uwy");
    joints[21] = new QTreeWidgetItem(left_arm);
    joints[21]->setText(0,"l_arm_mwx");

	//Right Arm Joints
    joints[22]= new QTreeWidgetItem(right_arm);
    joints[22]->setText(0,"r_arm_usy");
    joints[23] = new QTreeWidgetItem(right_arm);
    joints[23]->setText(0,"r_arm_shx");
    joints[24]= new QTreeWidgetItem(right_arm);
    joints[24]->setText(0,"r_arm_ely");
    joints[25] = new QTreeWidgetItem(right_arm);
    joints[25]->setText(0,"r_arm_elx");
    joints[26] = new QTreeWidgetItem(right_arm);
    joints[26]->setText(0,"r_arm_uwy");
    joints[27] = new QTreeWidgetItem(right_arm);
    joints[27]->setText(0,"r_arm_mwx");

    warnMin=.8;
    errorMin=.95;

     std::cout << "JointList Widget setup now starting subscribing to Ros topic." << std::endl;
    ros::NodeHandle nh;
    joint_states = nh.subscribe<sensor_msgs::JointState>( "/atlas/joint_states", 2, &jointList::updateList, this );

    //ros::spinOnce();
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

void jointList::updateList( const sensor_msgs::JointStateConstPtr& joint_states )
{
    warn = 0;
    error = 0;
     std::cout << "Recieved Ros Message." << std::endl;
    for(int i=0;i<joint_states->name.size(); i++)
    {
        joints[i].setText(1,QString::number(joint_states->position[i]));
        joints[i].setText(2,QString::number(joint_states->velocity[i]));
        joints[i].setText(3,QString::number(joint_states->effort[i]));
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

//        if(joint_states->velocity[i] >= warnMin*limitsMessage->velocity[i])
//        {
//            warn++;
//            joints[i].setBackgroundColor(0,Qt::yellow);
//            joints[i].setBackgroundColor(2,Qt::yellow);
//        }
//        if(joint_states->velocity[i] >= errorMin*limitsMessage->velocity[i])
//        {
//            warn--;
//            error++;
//            joints[i].setBackgroundColor(0,Qt::red);
//            joints[i].setBackgroundColor(2,Qt::red);
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
