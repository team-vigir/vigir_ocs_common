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
#ifndef MANIPULATION_TEST_H
#define MANIPULATION_TEST_H

#include <QElapsedTimer>
#include <QTextStream>
#include <time.h>
#include <ctime>
#include <QFile>
#include <QDir>
#include "ui/main_view_widget.h"
#include "vigir_ocs_msgs/OCSTemplateAdd.h"
#include "vigir_ocs_msgs/OCSTemplateRemove.h"
#include "vigir_ocs_msgs/OCSTemplateList.h"
#include <ros/ros.h>
#include <QQuaternion>
#include <QVector3D>
#include <QKeyEvent>



class ManipulationTest : public MainViewWidget
{
    Q_OBJECT

public:
    explicit ManipulationTest(QWidget *parent = 0);
    ~ManipulationTest();

public Q_SLOTS:
    void newChallenge();

private:
    void addInteractiveMarker( const vigir_ocs_msgs::OCSInteractiveMarkerAdd::ConstPtr& msg );
    void insertTemplate(QVector3D*  position, QQuaternion* rotation, QString);
    void defineTransforms();

    void templateListCb(const vigir_ocs_msgs::OCSTemplateList::ConstPtr& msg);

    std::vector<QVector3D *> templatePositions;
    std::vector<QQuaternion *> templateRotations;
    std::vector<QString> templateNames;
    QVector3D * startingPosition;
    QQuaternion* startingRotation;

    QFile * results;

    QSet<Qt::Key> keysPressed;
    QPushButton* startChallenge;
    int challengeCount;
    QElapsedTimer * elapsedTimer;

    ros::Publisher template_add_pub_;
    ros::Publisher template_update_pub;
    ros::Publisher template_remove_pub_;
    ros::Subscriber template_list_sub;

    ros::Subscriber interactive_marker_sub;

    vigir_ocs_msgs::OCSTemplateList temList;

    ros::NodeHandle n_;    


};

#endif // MANIPULATION_TEST_H
