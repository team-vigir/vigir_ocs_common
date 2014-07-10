#include "glancehubSbar.h"
#include <ros/ros.h>
#include "ui_glancehubSbar.h"
#include <ros/package.h>
#include <QStandardItemModel>

glancehubSbar::glancehubSbar(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::glancehubSbar)
{
    ui->setupUi(this);
    ghub = new glancehub(this);
    //remove window border and set on top
    Qt::WindowFlags flags = ghub->windowFlags();
    flags |= Qt::WindowStaysOnTopHint;
    flags |= Qt::FramelessWindowHint;
    flags |= Qt::Dialog; // //ensure ghub as a dialog box, not a seperate window/tab
    ghub->setWindowFlags(flags);

    ghub->setWindowOpacity(0);

    connect(ghub,SIGNAL(sendMoveitStatus(bool)),this,SLOT(receiveMoveitStatus(bool)));
    connect(ghub,SIGNAL(sendFootstepStatus(int)),this,SLOT(receiveFootstepStatus(int)));
    connect(ghub,SIGNAL(sendFlorStatus(int)),this,SLOT(receiveFlorStatus(int)));
    connect(ui->modeBox,SIGNAL(currentIndexChanged(int)),this,SLOT(receiveModeChange(int)));

    ui->modelabel->setText("");// default setting is off on start
    previousSelection = "Flor_Off";

    //sets first item to unselectable
    QStandardItemModel* model =
            qobject_cast<QStandardItemModel*>(ui->modeBox->model());
    QModelIndex firstIndex = model->index(0, ui->modeBox->modelColumn(),
            ui->modeBox->rootModelIndex());
    QStandardItem* firstItem = model->itemFromIndex(firstIndex);
    firstItem->setSelectable(false);

    //set popup width larger
    ui->modeBox->view()->setFixedWidth(130);

    //setup publisher to change modes
    mode_pub = nh.advertise<flor_control_msgs::FlorControlModeCommand>("/flor/controller/mode_command", 5, false);

    ui->plannerLight->setStyleSheet("QLabel { background-color: white; border:2px solid grey; }");
    ui->footstepLight->setStyleSheet("QLabel { background-color: white; border:2px solid grey; }");
    ui->modeBox->setStyleSheet("QComboBox {selection-color: grey;}");

    ui->plannerLight->setToolTip("waiting for status update");
    ui->moveitLabel->setToolTip("waiting for status update");

    ui->footstepLight->setToolTip("waiting for status update");
    ui->footstepLabel->setToolTip("waiting for status update");

    colorTimer.start(300,this);
    maxFlashes = 6;
    flashingMoveIt = false;
    flashingFootstep = false;
    coloredMoveIt = false;
    coloredFootstep = false;
    flashFootstepCounter = 0;
    flashMoveItCounter = 0;
    white = "QLabel {background-color: white; border:2px solid grey;}";
}

void glancehubSbar::timerEvent(QTimerEvent *event)
{
    if(flashingMoveIt)
    {
        if(flashMoveItCounter < maxFlashes)
        {
            //flash moveit light
            if(coloredMoveIt)
            {
                ui->plannerLight->setStyleSheet(white);
                coloredMoveIt = !coloredMoveIt;
            }
            else
            {
                ui->plannerLight->setStyleSheet(flashColorMoveIt);
                coloredMoveIt = !coloredMoveIt;
                flashMoveItCounter++;
            }
        }
        else
        {
            //flashing done. reset and wait for next call
            //counter is reset by functions that accepts new states
            flashingMoveIt = false;
        }
    }

    if(flashingFootstep)
    {
        if(flashFootstepCounter < maxFlashes)
        {
            //flash footstep light
            if(coloredFootstep)
            {
                ui->footstepLight->setStyleSheet(white);
                coloredFootstep = !coloredFootstep;
            }
            else
            {
                ui->footstepLight->setStyleSheet(flashColorFootstep);
                coloredFootstep = !coloredFootstep;
                flashFootstepCounter++;
            }
        }
        else
        {
            flashingFootstep = false;
        }
    }
}

void glancehubSbar::receiveModeChange(int mode)
{
    QString modeBefore = previousSelection;

    flor_control_msgs::FlorControlModeCommand msg;
    msg.header.stamp = ros::Time::now();
    ui->modelabel->setStyleSheet("QLabel{color:red; }");
    switch(mode)
    {
    case 0:
        return; //first is not selectable
        break;
    case 1:
        ui->modelabel->setText(modeBefore+" -> Stand");
        msg.behavior = flor_control_msgs::FlorControlModeCommand::STAND;
        previousSelection = "Stand";
        break;
    case 2:
        ui->modelabel->setText(modeBefore+" -> Walk");
        msg.behavior = flor_control_msgs::FlorControlModeCommand::WALK;
        previousSelection = "Walk";
        break;
    case 3:
        ui->modelabel->setText(modeBefore+" -> Step");
        msg.behavior = flor_control_msgs::FlorControlModeCommand::STEP;
        previousSelection = "Step";
        break;
    case 4:
        ui->modelabel->setText(modeBefore+" -> Manipulate");
        msg.behavior = flor_control_msgs::FlorControlModeCommand::MANIPULATE;
        previousSelection = "Manipulate";
        break;
    case 5:
        ui->modelabel->setText(modeBefore+" -> Flor_Stand");
        msg.behavior = flor_control_msgs::FlorControlModeCommand::FLOR_STAND;
        previousSelection = "Flor_Stand";
        break;
    case 6:
        ui->modelabel->setText(modeBefore+" -> Flor_Walk_Mani");
        msg.behavior = flor_control_msgs::FlorControlModeCommand::FLOR_WALK_MANI;
        previousSelection = "Flor_Walk_Mani";
        break;
    case 7:
        ui->modelabel->setText(modeBefore+" -> Flor_Step_Mani");
        msg.behavior = flor_control_msgs::FlorControlModeCommand::FLOR_STEP_MANI;
        previousSelection = "Flor_Step_Mani";
        break;
    case 8:
        ui->modelabel->setText(modeBefore+" -> Flor_Dance");
        msg.behavior = flor_control_msgs::FlorControlModeCommand::FLOR_DANCE;
        previousSelection = "Flor_Dance";
        break;
    case 9:
        ui->modelabel->setText(modeBefore+" -> Flor_WBC");
        msg.behavior = flor_control_msgs::FlorControlModeCommand::FLOR_WBC;
        previousSelection = "Flor_WBC";
        break;    
    case flor_control_msgs::FlorControlModeCommand::FLOR_STOP:
        ui->modelabel->setText(modeBefore+" -> Flor_Stop");
        msg.behavior = flor_control_msgs::FlorControlModeCommand::FLOR_STOP;
        previousSelection = "Flor_Stop";
        break;
    }
    mode_pub.publish(msg);
}

void glancehubSbar::receiveMoveitStatus(bool status)
{
    if(status)
        flashColorMoveIt = "QLabel { background-color: red;border:2px solid grey; }";
    else
       flashColorMoveIt = "QLabel { background-color: green; border:2px solid grey; }";
    ui->plannerLight->setToolTip(ghub->getMoveitStat());
    ui->moveitLabel->setToolTip(ghub->getMoveitStat());
    flashingMoveIt = true;
    flashMoveItCounter = 0; // reset counter here because we want to flash latest color 10 times (this function may be called multiple times in short span)
}

void glancehubSbar::receiveFootstepStatus(int status)
{
        switch(status)
        {
        case RobotStatusCodes::FOOTSTEP_PLANNER_ACTIVE:
            flashColorFootstep = "QLabel { background-color: yellow; border:2px solid grey;}";
            break;
        case RobotStatusCodes::FOOTSTEP_PLANNER_FAILED:
            flashColorFootstep = "QLabel { background-color: red; border:2px solid grey; }";
            break;
        case RobotStatusCodes::FOOTSTEP_PLANNER_SUCCESS:
            flashColorFootstep = "QLabel { background-color: green; border:2px solid grey;}";
            break;
        }
    ui->footstepLight->setToolTip(ghub->getFootstepStat());
    ui->footstepLabel->setToolTip(ghub->getFootstepStat());
    flashingFootstep = true;
    flashFootstepCounter = 0;
}

void glancehubSbar::receiveFlorStatus(int status)
{
    ROS_ERROR("RECEIVEING MODES");
    QString newText;
    switch(status)
    {
    case flor_control_msgs::FlorControlModeCommand::FLOR_DANCE:
        newText = QString::fromStdString("Flor_Dance");
        break;
    case flor_control_msgs::FlorControlModeCommand::FLOR_MANIPULATE:
        newText = QString::fromStdString("Flor_Manipulate");
        break;
    case flor_control_msgs::FlorControlModeCommand::FLOR_OFF:
        newText = QString::fromStdString("Flor_Off");
        break;
    case flor_control_msgs::FlorControlModeCommand::FLOR_STAND:
        newText = QString::fromStdString("Flor_Stand");
        break;
    case flor_control_msgs::FlorControlModeCommand::FLOR_STEP:
        newText = QString::fromStdString("Flor_Step");
        break;
    case flor_control_msgs::FlorControlModeCommand::FLOR_STEP_MANI:
        newText = QString::fromStdString("Flor_Step_Mani");
        break;
    case flor_control_msgs::FlorControlModeCommand::FLOR_STOP:
        newText = QString::fromStdString("Flor_Stop");
        break;
    case flor_control_msgs::FlorControlModeCommand::FLOR_WALK:
        newText = QString::fromStdString("Flor_Walk");
        break;
    case flor_control_msgs::FlorControlModeCommand::FLOR_WALK_MANI:
        newText = QString::fromStdString("Flor_Walk_Mani");
        break;
    case flor_control_msgs::FlorControlModeCommand::FLOR_WBC:
        newText = QString::fromStdString("Flor_WBC");
        break;
    case flor_control_msgs::FlorControlModeCommand::FREEZE:
        newText = QString::fromStdString("Freeze");
        break;
    case flor_control_msgs::FlorControlModeCommand::MANIPULATE:
        newText = QString::fromStdString("Manipulate");
        break;
    case flor_control_msgs::FlorControlModeCommand::STAND:
        newText = QString::fromStdString("Stand");
        break;
    case flor_control_msgs::FlorControlModeCommand::STAND_PREP:
        newText = QString::fromStdString("Stand_Prep");
        break;
    case flor_control_msgs::FlorControlModeCommand::STEP:
        newText = QString::fromStdString("Step");
        break;
    case flor_control_msgs::FlorControlModeCommand::USER:
        newText = QString::fromStdString("User");
        break;
    case flor_control_msgs::FlorControlModeCommand::WALK:
        newText = QString::fromStdString("Walk");
        break;
    }   
    updateBoxSelection(newText);
    ui->modelabel->setText(""); // only want to display transitions
}

void glancehubSbar::updateBoxSelection(QString mode)
{
    if(mode == "Flor_Stop")
    {
        ui->modeBox->setCurrentIndex(0);
        return;
    }

    for(int i=0;i<ui->modeBox->count();i++)
    {
        if(ui->modeBox->itemText(i) == mode)
        {
            ui->modeBox->setCurrentIndex(i);
            return;
        }
    }
}

glancehubSbar::~glancehubSbar()
{
    delete(ghub);
    delete ui;
}
