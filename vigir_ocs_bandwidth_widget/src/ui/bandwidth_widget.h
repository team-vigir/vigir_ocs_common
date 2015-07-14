/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013-2015, TORC Robotics, LLC ( Team ViGIR )
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Team ViGIR, TORC Robotics, nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
@TODO_ADD_AUTHOR_INFO
#ifndef BandwidthWidget_H
#define BandwidthWidget_H


#include <QMainWindow>
#include <QWidget>
#include <QRadioButton>
#include <QSpinBox>
#include <QComboBox>
#include <QStringList>

#include <QPainter>
#include <QtGui>

#include <vigir_ocs_msgs/OCSBandwidth.h>
#include <vigir_ocs_msgs/DRCdata.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

namespace Ui {
class BandwidthWidget;
}

class BandwidthWidget : public QWidget
{
    Q_OBJECT

public:
    explicit BandwidthWidget(QWidget *parent = 0);
    ~BandwidthWidget();

    void processBandwidthMessage(const vigir_ocs_msgs::OCSBandwidth::ConstPtr& msg);
    void processDRCData(const vigir_ocs_msgs::DRCdata::ConstPtr& msg);
    void heartbeatRecieved(const std_msgs::String::ConstPtr& msg);
    void updateRateValues();
    void resizeLatencyVector();
private:
    Ui::BandwidthWidget* ui;
    ros::Publisher drc_data_pub_;
    ros::NodeHandle nh_;
    ros::Subscriber ocs_bandwidth_sub_;
    ros::Subscriber topic_heartbeat_sub_;
    ros::Subscriber vrc_data_sub_;

    typedef struct
    {
        std::string node_name;
        unsigned long last_calculated_bytes_up;
        unsigned long last_calculated_bytes_down;
        unsigned long total_bytes_read;
        unsigned long total_bytes_sent;
    } BandwidthStruct;

    std::vector<BandwidthStruct> node_bandwidth_info_;

    QTableWidgetItem* total_bytes_read_item;
    int foobar;
    bool lowBWMode;
    ros::Time modeStartTime;
    double last_max_bytes_up;
    double last_max_bytes_down;
    std::vector<int> latencyNums;
    QTableWidgetItem* total_bytes_sent_item;
    bool bytes_remaining_initialized;
    int32_t down_max;
    int32_t up_max;
    QBasicTimer updateTimer;

    typedef struct
    {
        QBasicTimer timer;
        int alarmsSinceReturn;
    }topicObject;
    std::vector<topicObject*> topicObjectsList;


protected:
    void timerEvent(QTimerEvent *event);
private:
    QBasicTimer timer;
};

#endif // BandwidthWidget_H
