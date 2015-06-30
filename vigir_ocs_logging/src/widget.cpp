#include "widget.h"
#include "ui_widget.h"
#include <time.h>
#include <sstream>
#include "QPlainTextEdit"
#include "QLineEdit"
#include "QMessageBox"
#include "boost/lexical_cast.hpp"
#include <iomanip>
#include <ctime>
#include "QSpinBox"
#include "QLabel"

Widget::Widget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Widget)
{
    ros::start();

    ui->setupUi(this);
    ros::NodeHandle nh("~");
    timer.start(33, this);
    ocs_logging_pub_ = nh.advertise<vigir_ocs_msgs::OCSLogging>("/vigir_logging",        1, false);
    ocs_responce_sub_ = nh.subscribe<std_msgs::String>("/vigir_logging_responce", 5, &Widget::on_responce_recieved, this);
    ocs_responce_pub_ = nh.advertise<std_msgs::String>("/vigir_logging_query", 1, false);
    experiment_directory_ = "/home/vigir/Experiments/";
    if(nh.hasParam("experiment_directory"))
            nh.getParam("experiment_directory",experiment_directory_);
	first = true;

}

Widget::~Widget()
{
    delete ui;
}

void Widget::on_responce_recieved(const std_msgs::String::ConstPtr& msg)
{
    std::cout << "Recieved " << msg->data << std::endl;    
    if(msg->data ==       "video_start")
    {
        std::cout << "Video Start" << std::endl;
        //change text and color of videoLogging to green
        ui->videoLogging->setStyleSheet("color: rgb(0,255,0)");
        ui->videoLogging->setText(QString::fromStdString("Video Running"));
    }
    else if(msg->data == "video_stop")
    {
        //change text and color of videoLogging to red
        std::cout << "Video Start" << std::endl;
        ui->videoLogging->setStyleSheet("color: rgb(255,0,0)");
        ui->videoLogging->setText(QString::fromStdString("Video Stopped"));
    }
    else if(msg->data == "ocs_start")
    {
        //change text and color of OCSLogging to red
        std::cout << "OCS Start" << std::endl;
        ui->OCULogging->setStyleSheet("color: rgb(0,255,0)");
        ui->OCULogging->setText(QString::fromStdString("OCS Running"));

    }
    else if(msg->data == "ocs_stop")
    {
        //change text and color of OCSLogging to red
        std::cout << "OCS Stop" << std::endl;
        ui->OCULogging->setStyleSheet("color: rgb(255,0,0)");
        ui->OCULogging->setText(QString::fromStdString("OCS Stopped"));
    }
    else if(msg->data == "onboard_start")
    {
        //change text and color of onboardLogging to red
        std::cout << "Onboard Start" << std::endl;
        ui->onboardLogging->setStyleSheet("color: rgb(0,255,0)");
        ui->onboardLogging->setText(QString::fromStdString("Onboard Running"));
    }
    else if(msg->data == "onboard_stop")
    {
        //change text and color of onboardLogging to red
        std::cout << "Onboard Stop" << std::endl;
        ui->onboardLogging->setStyleSheet("color: rgb(255,0,0)");
        ui->onboardLogging->setText(QString::fromStdString("Onboard Stopped"));
    }

}

void Widget::timerEvent(QTimerEvent *event)
{
    // check if ros is still running; if not, just kill the application
    if(!ros::ok())
        qApp->quit();

    //Spin at beginning of Qt timer callback, so current ROS time is retrieved
    ros::spinOnce();
    if(first)
    {
        std::cout << "querying for the state of all children..." << std::endl;        
        std_msgs::String query;
        query.data = "?";
        first = false;ros::spinOnce();
        ocs_responce_pub_.publish(query);
        ros::spinOnce();
    }
}


void Widget::on_startButton_clicked()
{
    QRegExp rx("(\s\\\\)");
    std::string expName = (ui->experimentName->text().replace(rx,tr("_"))).toStdString();
    std::cout << "Exp name is " << expName << std::endl;
    boost::filesystem::path folder (std::string(experiment_directory_+expName));
    if(boost::filesystem::exists(folder))
    {
        std::cout << "Folder already exists " << folder.c_str() << std::endl;
        QMessageBox msg;
        msg.setWindowTitle(QString::fromStdString("Logging Error"));
        msg.setInformativeText(QString::fromStdString("Cannot have two experiments with the same name.\n\nPlease rename the experiment to continue."));
        msg.exec();
    }
    else
    {
        if(boost::filesystem::create_directory(folder))
            std::cout<< "Created new folder at " << folder.c_str() << std::endl;
        sendMsg(true);
    }
}

void Widget::on_stopButton_clicked()
{
    sendMsg(false);
}

void Widget::sendMsg(bool run)
{
    vigir_ocs_msgs::OCSLogging msg;
    msg.run = run;
    msg.experiment_name = ui->experimentName->text().toStdString();
    msg.description = ui->descriptionBox->document()->toPlainText().toStdString();
    msg.ustream = ui->ustreamBox->isChecked();
    ocs_logging_pub_.publish(msg);
}

void Widget::on_experimentName_textChanged(const QString &arg1)
{
    if(ui->experimentName->text().toStdString() == "")
        ui->startButton->setEnabled(false);
    else
        ui->startButton->setEnabled(true);
}


