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
    ghub_ = new glancehub(this);
    //remove window border and set on top
    Qt::WindowFlags flags = ghub_->windowFlags();
    flags |= Qt::WindowStaysOnTopHint;
    flags |= Qt::FramelessWindowHint;
    flags |= Qt::Dialog; // //ensure ghub_ as a dialog box, not a seperate window/tab
    ghub_->setWindowFlags(flags);

    ghub_->setWindowOpacity(0);

    connect(ghub_,SIGNAL(sendMoveitStatus(bool)),this,SLOT(receiveMoveitStatus(bool)));
    connect(ghub_,SIGNAL(sendFootstepStatus(int)),this,SLOT(receiveFootstepStatus(int)));
    connect(ghub_,SIGNAL(sendFlorStatus(int)),this,SLOT(receiveFlorStatus(int)));
    connect(ui->modeBox,SIGNAL(currentIndexChanged(int)),this,SLOT(receiveModeChange(int)));

    ui->modelabel->setText(""); // default setting is off on start
    previous_selection_ = "Flor_Off";

    //sets first item to unselectable
    QStandardItemModel* model = qobject_cast<QStandardItemModel*>(ui->modeBox->model());
    QModelIndex firstIndex = model->index(0, ui->modeBox->modelColumn(), ui->modeBox->rootModelIndex());
    QStandardItem* firstItem = model->itemFromIndex(firstIndex);
    firstItem->setSelectable(false);

    //set popup width larger
    ui->modeBox->view()->setFixedWidth(130);

    //setup publisher to change modes
    mode_pub_ = nh_.advertise<flor_control_msgs::FlorControlModeCommand>("/flor/controller/mode_command", 5, false);

    // load control modes into dropdown box from parameters
    nh_.getParam("/atlas_controller/allowed_control_modes", allowed_control_modes_);
    for(int i = 0; i < allowed_control_modes_.size(); i++)
        ui->modeBox->addItem(allowed_control_modes_[i].c_str());

    ui->plannerLight->setStyleSheet("QLabel { background-color: white; border:2px solid grey; }");
    ui->footstepLight->setStyleSheet("QLabel { background-color: white; border:2px solid grey; }");
    ui->modeBox->setStyleSheet("QComboBox {selection-color: grey;}");

    ui->plannerLight->setToolTip("waiting for status update");
    ui->moveitLabel->setToolTip("waiting for status update");

    ui->footstepLight->setToolTip("waiting for status update");
    ui->footstepLabel->setToolTip("waiting for status update");

    color_timer_.start(300,this);
    max_flashes_ = 6;
    flashing_move_it_ = false;
    flashing_footstep_ = false;
    colored_moveit_ = false;
    colored_footstep_ = false;
    flash_footstep_counter_ = 0;
    flash_moveit_counter_ = 0;
    white_ = "QLabel {background-color: white; border:2px solid grey;}";
}

void glancehubSbar::timerEvent(QTimerEvent *event)
{
    if(flashing_move_it_)
    {
        if(flash_moveit_counter_ < max_flashes_)
        {
            //flash moveit light
            if(colored_moveit_)
            {
                ui->plannerLight->setStyleSheet(white_);
                colored_moveit_ = !colored_moveit_;
            }
            else
            {
                ui->plannerLight->setStyleSheet(flash_color_moveit_);
                colored_moveit_ = !colored_moveit_;
                flash_moveit_counter_++;
            }
        }
        else
        {
            //flashing done. reset and wait for next call
            //counter is reset by functions that accepts new states
            flashing_move_it_ = false;
        }
    }

    if(flashing_footstep_)
    {
        if(flash_footstep_counter_ < max_flashes_)
        {
            //flash footstep light
            if(colored_footstep_)
            {
                ui->footstepLight->setStyleSheet(white_);
                colored_footstep_ = !colored_footstep_;
            }
            else
            {
                ui->footstepLight->setStyleSheet(flash_color_footstep_);
                colored_footstep_ = !colored_footstep_;
                flash_footstep_counter_++;
            }
        }
        else
        {
            flashing_footstep_ = false;
        }
    }
}

void glancehubSbar::receiveModeChange(int mode)
{
    ui->modelabel->setStyleSheet("QLabel{color:red; }");

    QString newText;
    if (mode >= 0 && mode <  allowed_control_modes_.size())
        newText = QString::fromStdString(allowed_control_modes_[mode]);
    else
        newText = QString::fromStdString("Unknown");

    ui->modelabel->setText(previous_selection_+" -> "+newText);

    flor_control_msgs::FlorControlModeCommand msg;
    msg.header.stamp = ros::Time::now();
    msg.requested_control_mode = mode;
    previous_selection_ = newText;
    mode_pub_.publish(msg);

    //notify on 3d view                     previous is still at the state we want right now
    QString notificationText = QString("Changed Robot Mode to ") + previous_selection_ ;
    //flor stop is an error condition
    if(previous_selection_ == "stop")
        NotificationSystem::Instance()->notifyError(notificationText.toStdString());
    else
        NotificationSystem::Instance()->notifyPassive(notificationText.toStdString());
}

void glancehubSbar::receiveMoveitStatus(bool status)
{
    if(status)
    {
        //moveit failed
        flash_color_moveit_ = "QLabel { background-color: red;border:2px solid grey; }";
        NotificationSystem::Instance()->notifyWarning("Moveit Failed");
    }
    else
    {
        //moveit success
       flash_color_moveit_ = "QLabel { background-color: green; border:2px solid grey; }";
       //NotificationSystem::Instance()->notifyPassive("Moveit Success");
    }
    ui->plannerLight->setToolTip(ghub_->getMoveitStat());
    ui->moveitLabel->setToolTip(ghub_->getMoveitStat());
    flashing_move_it_ = true;
    flash_moveit_counter_ = 0; // reset counter here because we want to flash latest color 10 times (this function may be called multiple times in short span)
}

void glancehubSbar::receiveFootstepStatus(int status)
{
        switch(status)
        {
        case RobotStatusCodes::FOOTSTEP_PLANNER_ACTIVE:
            flash_color_footstep_ = "QLabel { background-color: yellow; border:2px solid grey;}";
            break;
        case RobotStatusCodes::FOOTSTEP_PLANNER_FAILED:
            flash_color_footstep_ = "QLabel { background-color: red; border:2px solid grey; }";

            //notify ui on failure
            NotificationSystem::Instance()->notifyWarning("Footstep Planner Failed");
            break;
        case RobotStatusCodes::FOOTSTEP_PLANNER_SUCCESS:
            flash_color_footstep_ = "QLabel { background-color: green; border:2px solid grey;}";
            break;
        }
    ui->footstepLight->setToolTip(ghub_->getFootstepStat());
    ui->footstepLabel->setToolTip(ghub_->getFootstepStat());
    flashing_footstep_ = true;
    flash_footstep_counter_ = 0;
}

void glancehubSbar::receiveFlorStatus(int status)
{
    updateBoxSelection(status);
    ui->modelabel->setText(""); // only want to display transitions
}

void glancehubSbar::updateBoxSelection(int mode)
{
    QString newText;
    if (mode >= 0 && mode <  allowed_control_modes_.size())
        newText = QString::fromStdString(allowed_control_modes_[mode]);
    else
        newText = QString::fromStdString("Unknown");

    if(newText == "stop")
    {
        ui->modeBox->setCurrentIndex(0);
        return;
    }

    ui->modeBox->setCurrentIndex(mode);

}

glancehubSbar::~glancehubSbar()
{
    delete(ghub_);
    delete ui;
}
