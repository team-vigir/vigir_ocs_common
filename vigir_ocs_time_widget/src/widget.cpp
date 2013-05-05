#include "widget.h"
#include "ui_widget.h"
#include <time.h>
#include <sstream>
#include <iomanip>
#include "rviz/visualization_manager.h"
#include "rviz/render_panel.h"
#include "rviz/display.h"
#include "rviz/frame_manager.h"

Widget::Widget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Widget)
{
    render_panel_ = new rviz::RenderPanel();
    manager_ = new rviz::VisualizationManager( render_panel_ );
    render_panel_->initialize( manager_->getSceneManager(), manager_ );
    manager_->initialize();
    manager_->startUpdate();
    ui->setupUi(this);
    //updateDate();
    //updateTime();
    ros::NodeHandle nh;
    rosTime = nh.subscribe<rosgraph_msgs::Clock>( "/clock", 2, &Widget::rosTimeMsgRecieved, this );
    //QTimer *timer = new QTimer(this);
    //connect(timer, SIGNAL(timeout()), this, SLOT(timerCallback()));
    timer.start(200, this);
    //timer = nh.createTimer(ros::Duration(0.2), &Widget::timerCallback, this, false);
    //timer.start();
    ros::spinOnce();
}

void Widget::timerEvent(QTimerEvent *event)
{
    //std::cout << "Timer Went off! Now updating date and time." << std::endl;
    updateDate();
    updateTime();
}

void Widget::updateDate()
{
    time_t now;
    struct tm *tm;
    //rosgr

    now = time(0);
    if ((tm = localtime (&now)) == NULL) {
        std::cout << "Error reading time from system" << std::endl;
           // printf ("Error extracting time stuff\n");
        return ;
    }
    else
    {
        std::stringstream date;
        date << std::setprecision(2) << tm->tm_mon+1 << "/" << std::setprecision(2) << tm->tm_mday << "/" << std::setprecision(4) << tm->tm_year + 1900;
        ui->dateDisp->setText(QString::fromStdString(date.str()));
    }
}

void Widget::updateTime()
{
    time_t now;
    struct tm *tm;

    now = time(0);
    if ((tm = localtime (&now)) == NULL) {
        std::cout << "Error reading time from system" << std::endl;
           // printf ("Error extracting time stuff\n");
        return ;
    }
    else
    {
        std::stringstream time;
        time << std::setprecision(2) << tm->tm_hour << ":" <<std::setprecision(2) << tm->tm_min << ":" << std::setprecision(2) << tm->tm_sec;
        ui->timeDisp->setText(QString::fromStdString(time.str()));
    }
}

void Widget::rosTimeMsgRecieved(const rosgraph_msgs::Clock::ConstPtr& time)
{
    //std::cout << "Recieved ros sim time message" << std::endl;
    ui->rosDisp->setText(timeFromMsg(time->clock));
	
}


QString Widget::timeFromMsg(const ros::Time stamp)
{
    int sec = stamp.toSec();
    std::stringstream stream;

    stream.str("");
    int day = sec/86400;
    sec -= day * 86400;

    int hour = sec / 3600;
    sec -= hour * 3600;

    int min = sec / 60;
    sec -= min * 60;
    uint32_t nano = (stamp.toSec() - (int)stamp.toSec())*1000;
    stream << std::setw(2) << std::setfill('0') << day << " ";
    stream << std::setw(2) << std::setfill('0') << hour << ":";
    stream << std::setw(2) << std::setfill('0') << min << ":";
    stream << std::setw(2) << std::setfill('0') << sec << ".";
     stream << std::setw(3) << std::setfill('0') << nano;
//    stream << std::setw(3) << std::setfill('0') << (stamp.toNSec()*(0.00001));
    //std::cout << "Nano = " << stamp.toNSec() <<  " full = " << stamp.toSec() << std::endl;
    return QString::fromStdString(stream.str());
}

Widget::~Widget()
{
    delete ui;
}
