#include "glancehubsbar.h"
#include <ros/ros.h>
#include "ui_glancehubsbar.h"
#include "flor_ocs_msgs/RobotStatusCodes.h"
#include <QRegExp>
#include <urdf/model.h>


glancehubsbar::glancehubsbar(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::glancehubsbar)
{
    ui->setupUi(this);
    ros::NodeHandle nh;
    controlMode_sub = nh.subscribe<flor_control_msgs::FlorControlModeCommand>("/flor/controller/mode_command",5,&glancehubsbar::controlModeMsgRcv, this);
    robotStatusMoveit_sub = nh.subscribe<flor_ocs_msgs::OCSRobotStatus>("/flor/planning/upper_body/status",2,&glancehubsbar::robotStatusMoveit,this);
    robotStatusFootstep_sub = nh.subscribe<flor_ocs_msgs::OCSRobotStatus>("/flor/footstep_planner/status",2,&glancehubsbar::robotStatusFootstep,this);
    ui->errorbutton->setStyleSheet(" background-color: red");
    ui->warnbutton->setStyleSheet(" background-color: yellow");
    warnwidget = new QWidget();
    errorwidget = new QWidget();
    warntable = new QTableWidget();
    errortable = new QTableWidget();
    warnwidget->hide();
    errorwidget->hide();
    QHBoxLayout *hlay1 = new QHBoxLayout();
    hlay1->addWidget(warntable);
    warnwidget->setLayout(hlay1);
    QHBoxLayout *hlay2 = new QHBoxLayout();
    hlay2->addWidget(errortable);
    errorwidget->setLayout(hlay2);
    std::cout << "Adding layout..." << std::endl;
    warntable->setColumnCount(1);
    warntable->setColumnWidth(0,200);
    QStringList labels1;
    labels1.push_back("Warning Joints:");
    warntable->setHorizontalHeaderLabels(labels1);
    errortable->setColumnCount(1);
    errortable->setColumnWidth(0,200);
    QStringList labels2;
    labels2.push_back("Faulty Joints:");
    errortable->setHorizontalHeaderLabels(labels2);
    item = new QTableWidgetItem();
    //setLayout(main_layout);

    QStringList columns;


    //Torso Joints


    joints.push_back("back_bkz");
    joints.push_back("back_bky");
    joints.push_back("back_bkx");
    joints.push_back("neck_ry");

    //Left Leg Joints
    joints.push_back("l_leg_hpz");
    joints.push_back("l_leg_hpx");
    joints.push_back("l_leg_hpy");
    joints.push_back("l_leg_kny");
    joints.push_back("l_leg_aky");
    joints.push_back("l_leg_akx");

    //Right Leg Joints

    joints.push_back("r_leg_hpz");
    joints.push_back("r_leg_hpx");
    joints.push_back("r_leg_hpy");
    joints.push_back("r_leg_kny");
    joints.push_back("r_leg_aky");
    joints.push_back("r_leg_akx");

    //Left Arm Joints

    joints.push_back("l_arm_shy");
    joints.push_back("l_arm_shx");
    joints.push_back("l_arm_ely");
    joints.push_back("l_arm_elx");
    joints.push_back("l_arm_wry");
    joints.push_back("l_arm_wrx");
    //Right Arm Joints

    joints.push_back("r_arm_shy");
    joints.push_back("r_arm_shx");
    joints.push_back("r_arm_ely");
    joints.push_back("r_arm_elx");
    joints.push_back("r_arm_wry");
    joints.push_back("r_arm_wrx");

    warnMin=.75;
    errorMin=.90;

    warn=0;
    err=0;
    std::cout << "JointList Widget setup now starting subscribing to Ros topic." << std::endl;
    std::string robotInfo;

    nh.getParam("/robot_description",robotInfo);
    processRobotInfo(robotInfo);
    joint_states = nh.subscribe<sensor_msgs::JointState>( "/atlas/joint_states", 2, &glancehubsbar::updateList, this );

    timer.start(33, this);
}

void glancehubsbar::timerEvent(QTimerEvent *event)
{
    //Spin at beginning of Qt timer callback, so current ROS time is retrieved
    if(event->timerId() == timer.timerId())
        ros::spinOnce();
}
void glancehubsbar::processRobotInfo(std::string robotInfo)
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
                if(names.cap(0).remove(0,6).toStdString() == joints[i].toStdString().append("\""))
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
void glancehubsbar::updateList( const sensor_msgs::JointState::ConstPtr& joint_states )
{
    warn = 0;
    err = 0;
    //for(int i=0;i<joint_states->name.size(); i++)
    //joints[i].setStylesheet->parent()->setBackground(0,Qt::white);
    for(int i=0;i<joint_states->name.size(); i++)
    {
       /* joints[i]->setText(1,QString::number(joint_states->position[i]));
        joints[i]->setText(2,QString::number(joint_states->velocity[i]));
        joints[i]->setText(3,QString::number(joint_states->effort[i]));
        joints[i]->setBackgroundColor(0,Qt::white);
        joints[i]->setBackgroundColor(1,Qt::white);
        joints[i]->setBackgroundColor(3,Qt::white);*/

    //std::cout << "p=" << joint_states->position[i] << " v=" << joint_states->velocity[i];
        //std::cout << " e=" << joint_states->effort[i] << " dpl=" << downPoseLimit[i];
        //std::cout << " upl=" << upPoseLimit[i] << " el=" << effortLimits[i] << std::endl;
        if(joint_states->position[i] <= warnMin*downPoseLimit[i])
        {
            warn++;

            ui->warnbutton->setText("Joint warning:"+QString::number(warn));
            item->setText(joints[i]);
            warntable->insertRow(0);
            warntable->setItem(0,0,item);
            /*joints[i]->setBackgroundColor(0,Qt::yellow);
            joints[i]->setBackgroundColor(1,Qt::yellow);
            if(joints[i]->parent()->background(0) != Qt::red)
            {
                joints[i]->parent()->setBackgroundColor(0,Qt::yellow);
            }*/
        }
        if(joint_states->position[i] <= errorMin*downPoseLimit[i])
        {
            warn--;
            err++;
            ui->errorbutton->setText("Joint Warning"+QString::number(err));
            ui->warnbutton->setText("Joint Faults:"+QString::number(warn));
            item->setText(joints[i]);
            errortable->insertRow(0);
            errortable->setItem(0,0,item);
           /* joints[i]->setBackgroundColor(0,Qt::red);
            joints[i]->setBackgroundColor(1,Qt::red);
            joints[i]->parent()->setBackgroundColor(0,Qt::red);*/
        }

        if(joint_states->position[i] >= warnMin*upPoseLimit[i])
        {
            warn++;
            ui->warnbutton->setText("Joint Faults:"+QString::number(warn));
            item->setText(joints[i]);
            warntable->insertRow(0);
            warntable->setItem(0,0,item);

            /*joints[i]->setBackgroundColor(0,Qt::yellow);
            joints[i]->setBackgroundColor(1,Qt::yellow);
            if(joints[i]->parent()->background(0) != Qt::red)
            {
                joints[i]->parent()->setBackgroundColor(0,Qt::yellow);
            }*/

        }
        if(joint_states->position[i] >= errorMin*upPoseLimit[i])
        {
            warn--;
            err++;


            ui->errorbutton->setText("Joint Warning"+QString::number(err));
            ui->warnbutton->setText("Joint Faults:"+QString::number(warn));
            item->setText(joints[i]);
            errortable->insertRow(0);
            errortable->setItem(0,0,item);
            /*joints[i]->setBackgroundColor(0,Qt::red);
            joints[i]->setBackgroundColor(1,Qt::red);
            joints[i]->parent()->setBackgroundColor(0,Qt::red);*/
        }

        if(joint_states->effort[i] >= warnMin*effortLimits[i])
        {
            warn++;
            ui->warnbutton->setText("Joint Faults:"+QString::number(warn));
            item->setText(joints[i]);
            warntable->insertRow(0);
            warntable->setItem(0,0,item);

            /*joints[i]->setBackgroundColor(0,Qt::yellow);
            joints[i]->setBackgroundColor(3,Qt::yellow);
            if(joints[i]->parent()->background(0) != Qt::red)
            {
                joints[i]->parent()->setBackgroundColor(0,Qt::yellow);
            }*/
        }
        if(joint_states->effort[i] >= errorMin*effortLimits[i])
        {
            warn--;
            err++;
            ui->errorbutton->setText("Joint Warning"+QString::number(err));
            ui->warnbutton->setText("Joint Faults:"+QString::number(warn));
            item->setText(joints[i]);
            errortable->insertRow(0);
            errortable->setItem(0,0,item);
            /*
            joints[i]->setBackgroundColor(0,Qt::red);
            joints[i]->setBackgroundColor(3,Qt::red);
            joints[i]->parent()->setBackgroundColor(0,Qt::red);
            */
        }

        if(joint_states->effort[i] <= -(warnMin*effortLimits[i]))
        {
            warn++;
            ui->warnbutton->setText("Joint Faults:"+QString::number(warn));
            item->setText(joints[i]);
            warntable->insertRow(0);
            warntable->setItem(0,0,item);
            /*joints[i]->setBackgroundColor(0,Qt::yellow);
            joints[i]->setBackgroundColor(3,Qt::yellow);
            if(joints[i]->parent()->background(0) != Qt::red)
            {
                joints[i]->parent()->setBackgroundColor(0,Qt::yellow);
            }*/
        }
        if(joint_states->effort[i] <= -(errorMin*effortLimits[i]))
        {
            warn--;
            err++;
            ui->errorbutton->setText("Joint Warning"+QString::number(err));
            ui->warnbutton->setText("Joint Faults:"+QString::number(warn));
            item->setText(joints[i]);
            errortable->insertRow(0);
            errortable->setItem(0,0,item);

            /*joints[i]->setBackgroundColor(0,Qt::red);
            joints[i]->setBackgroundColor(3,Qt::red);
            joints[i]->parent()->setBackgroundColor(0,Qt::red);*/
        }
    }
}
void glancehubsbar::on_warnbutton_clicked()
{
    warnwidget->show();

}

void glancehubsbar::on_errorbutton_clicked()
{
    errorwidget->show();
}

void glancehubsbar::robotStatusMoveit(const flor_ocs_msgs::OCSRobotStatus::ConstPtr &msg)
{
    if(msg->status != RobotStatusCodes::PLANNER_MOVEIT_PLAN_ACTIVE)
        ui->plannerLight->setStyleSheet("QLabel { background-color: red; }");
    else
        ui->plannerLight->setStyleSheet("QLabel { background-color: green; }");
}

void glancehubsbar::robotStatusFootstep(const flor_ocs_msgs::OCSRobotStatus::ConstPtr &msg)
{
    switch(msg->status)
    {
    case RobotStatusCodes::FOOTSTEP_PLANNER_ACTIVE:
        ui->footLight->setStyleSheet("QLabel { background-color: yellow; }");
        break;
    case RobotStatusCodes::FOOTSTEP_PLANNER_FAILED:
        ui->footLight->setStyleSheet("QLabel { background-color: red; }");
        break;
    case RobotStatusCodes::FOOTSTEP_PLANNER_SUCCESS:
        ui->footLight->setStyleSheet("QLabel { background-color: green; }");
        break;
    }

}

void glancehubsbar::controlModeMsgRcv(const flor_control_msgs::FlorControlModeCommand::ConstPtr& msg)
{
    QString newText;
    switch(msg->behavior)
    {
    case flor_control_msgs::FlorControlModeCommand::FLOR_DANCE:
        newText = QString::fromStdString("Flor Dance");
        break;
    case flor_control_msgs::FlorControlModeCommand::FLOR_MANIPULATE:
        newText = QString::fromStdString("Flor Manipulate");
        break;
    case flor_control_msgs::FlorControlModeCommand::FLOR_OFF:
        newText = QString::fromStdString("Flor Off");
        break;
    case flor_control_msgs::FlorControlModeCommand::FLOR_STAND:
        newText = QString::fromStdString("Flor Stand");
        break;
    case flor_control_msgs::FlorControlModeCommand::FLOR_STEP:
        newText = QString::fromStdString("Flor Step");
        break;
    case flor_control_msgs::FlorControlModeCommand::FLOR_STEP_MANI:
        newText = QString::fromStdString("Flor Step Mani");
        break;
    case flor_control_msgs::FlorControlModeCommand::FLOR_STOP:
        newText = QString::fromStdString("Flor Stop");
        break;
    case flor_control_msgs::FlorControlModeCommand::FLOR_WALK:
        newText = QString::fromStdString("Flor Walk");
        break;
    case flor_control_msgs::FlorControlModeCommand::FLOR_WALK_MANI:
        newText = QString::fromStdString("Flor Walk Mani");
        break;
    case flor_control_msgs::FlorControlModeCommand::FLOR_WBC:
        newText = QString::fromStdString("Flor WBC");
        break;
    case flor_control_msgs::FlorControlModeCommand::FREEZE:
        newText = QString::fromStdString("Flor Freeze");
        break;
    case flor_control_msgs::FlorControlModeCommand::MANIPULATE:
        newText = QString::fromStdString("Flor Manipulate");
        break;
    case flor_control_msgs::FlorControlModeCommand::STAND:
        newText = QString::fromStdString("Flor Stand");
        break;
    case flor_control_msgs::FlorControlModeCommand::STAND_PREP:
        newText = QString::fromStdString("Flor Stand Prep");
        break;
    case flor_control_msgs::FlorControlModeCommand::STEP:
        newText = QString::fromStdString("Flor Step");
        break;
    case flor_control_msgs::FlorControlModeCommand::USER:
        newText = QString::fromStdString("Flor User");
        break;
    case flor_control_msgs::FlorControlModeCommand::WALK:
        newText = QString::fromStdString("Flor Walk");
        break;
    }
    ui->controlModeLabel->setText(newText);
    std::cout << "Changing to "<< newText.toStdString() << " Mode" << std::endl;
}

glancehubsbar::~glancehubsbar()
{
    delete ui;
}
