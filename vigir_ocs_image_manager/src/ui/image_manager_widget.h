/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013-2015, Team ViGIR ( TORC Robotics LLC, TU Darmstadt, Virginia Tech, Oregon State University, Cornell University, and Leibniz University Hanover )
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
//@TODO_ADD_AUTHOR_INFO
#ifndef ImageManagerWidget_H
#define ImageManagerWidget_H


#include <QMainWindow>
#include <QWidget>
#include <QRadioButton>
#include <QSpinBox>
#include <QComboBox>
#include <QStringList>

#include <QPainter>
#include <QtGui>

#include <vector>
#include <algorithm>

#include <vigir_ocs_msgs/OCSImageList.h>
#include <vigir_ocs_msgs/OCSImageAdd.h>
#include <vigir_ocs_msgs/OCSKeyEvent.h>

#include <std_msgs/Bool.h>
#include <std_msgs/UInt64.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <ros/ros.h>

#include "hotkey_manager.h"

namespace rviz
{
class RenderPanel;
class VisualizationManager;
}

namespace Ui {
class ImageManagerWidget;
}

class ImageManagerWidget : public QWidget
{
    Q_OBJECT
    
public:
    explicit ImageManagerWidget(QWidget *parent = 0);
    ~ImageManagerWidget();

    void processImageAdd(const vigir_ocs_msgs::OCSImageAdd::ConstPtr msg);
    void processImageList(const vigir_ocs_msgs::OCSImageList::ConstPtr msg);
    void processSelectedImage(const sensor_msgs::Image::ConstPtr msg);
    void processNewKeyEvent(const vigir_ocs_msgs::OCSKeyEvent::ConstPtr pose);
    /*void removeImage(int id);

    void initImageIdMap();*/

public Q_SLOTS:
    void imageClicked(int,int);
    
private:
    void addImage(const unsigned long& id, const std::string& topic, const sensor_msgs::Image& image, const sensor_msgs::CameraInfo& camera_info);
    QString timeFromMsg(const ros::Time& stamp);

    Ui::ImageManagerWidget* ui;

    ros::NodeHandle nh_;

    ros::Publisher image_list_request_pub_;
    ros::Publisher image_selected_pub_;

    ros::Subscriber image_list_sub_;
    ros::Subscriber image_added_sub_;
    ros::Subscriber image_selected_sub_;

    ///Hotkey
    void addHotKeys();
    void toggleImageManagerHotkey();



protected:
    void timerEvent(QTimerEvent *event);
private:
    QBasicTimer timer;
};

#endif // ImageManagerWidget_H
