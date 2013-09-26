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
    ros::start();

    ui->setupUi(this);

    timer.start(33, this);
}

void Widget::timerEvent(QTimerEvent *event)
{
	// check if ros is still running; if not, just kill the application
    if(!ros::ok())
        qApp->quit();
    
    //Spin at beginning of Qt timer callback, so current ROS time is retrieved
    ros::spinOnce();

    updateDate();
    updateTime();
    ui->rosDisp->setText(timeFromMsg(ros::Time::now()));
}

void Widget::updateDate()
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
        time << std::setw(2) << std::setfill('0') << tm->tm_hour << ":" << std::setw(2) << std::setfill('0') << tm->tm_min << ":" << std::setw(2) << std::setfill('0') << tm->tm_sec;
        ui->timeDisp->setText(QString::fromStdString(time.str()));
    }
}

QString Widget::timeFromMsg(const ros::Time& stamp)
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
