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
//@TODO_ADD_AUTHOR_INFO
#ifndef MOTION_SELECTOR_H
#define MOTION_SELECTOR_H

#include <QMainWindow>
#include <QTreeWidgetItem>
#include <QBasicTimer>
#include <QDoubleSpinBox>
#include <QPushButton>
#include <QLayout>

#include <vector>
#include <algorithm>

#include <ros/publisher.h>
#include <ros/ros.h>

#include <vigir_ocs_msgs/OCSKeyEvent.h>
#include "hotkey_manager.h"

namespace Ui {
class motion_selector;
}

class motion_selector : public QMainWindow
{
    Q_OBJECT

public:
    explicit motion_selector(QWidget *parent = 0);
    ~motion_selector();

    //void processNewKeyEvent(const vigir_ocs_msgs::OCSKeyEvent::ConstPtr& pose);

private:
    typedef struct
    {
        QString name;
        std::vector<QString> primitives;
    } Group;
    typedef struct
    {
        QString name;
        std::vector<Group> groups;
    } Task;

    typedef struct
    {
        QDoubleSpinBox* timeFactor;
        QPushButton* button;
    } quickButton;

    std::vector<quickButton*> quickButtonList;
    Ui::motion_selector *ui;
    QString filePath;
    void processTextFile( QString path);
    void setupQuickButtons(QString path);
    void printTasks();
    void populateTree();

    ros::NodeHandle nh_;
    ros::Publisher message_pub_;
    float sliderVal;
    std::vector<Task> taskList;
    std::vector<QTreeWidgetItem*> treeItems;
    QBasicTimer timer;

    //std::vector<int> keys_pressed_list_;

    //ros::Subscriber key_event_sub_;

    //hotkey
    void addHotKeys();
    void toggleVisibilityHotkey();


public Q_SLOTS:
    void on_sendCommand_clicked();
    void on_timeFactorSlider_valueChanged(int value);
    void quickButtonClicked();
    void on_enableQuickButtons_clicked();

protected:
    void timerEvent(QTimerEvent *event);
};

#endif // MOTION_SELECTOR_H
