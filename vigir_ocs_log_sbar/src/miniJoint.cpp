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

#include "ui_miniJoint.h"
#include "miniJoint.h"

MiniJoint::MiniJoint(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MiniJoint)
{
    ui->setupUi(this);

    timer = new QTimer(this);
    connect(this->parent(),SIGNAL(makeJointActive()),this,SLOT(startActiveTimer()));

    jList = new jointList();

    connect(jList,SIGNAL(sendJointData(int,QString)),this,SLOT(receiveJointData(int,QString)));
    connect(jList,SIGNAL(sendJointData(int,QString)),this->parent(),SLOT(receiveJointData(int, QString)));
    ui->table->setColumnWidth(0,100);
    ui->table->setColumnWidth(1,300);
    //install event filter to get mouse presses
    ui->table->setMouseTracking( true );
    ui->table->viewport()->installEventFilter( this );

    //build animations
    jointFadeIn = new QPropertyAnimation(this, "windowOpacity");
    jointFadeIn->setEasingCurve(QEasingCurve::InOutQuad);
    jointFadeIn->setDuration(500);
    jointFadeIn->setStartValue(0.0);
    jointFadeIn->setEndValue(.74);

    jointFadeOut = new QPropertyAnimation(this, "windowOpacity");
    jointFadeOut->setEasingCurve(QEasingCurve::InOutQuad);
    jointFadeOut->setDuration(300);
    jointFadeOut->setStartValue(0.74);
    jointFadeOut->setEndValue(0.0);

    visible = false;

    connect(jointFadeOut,SIGNAL(finished()),this,SLOT(hideWindow()));
}


bool MiniJoint::eventFilter(QObject* object,QEvent* event)
{
    if (object == ui->table->viewport())
    {
        QMouseEvent *mouseEvent = static_cast<QMouseEvent*>(event);
        if(mouseEvent->button() == 1 && mouseEvent->type() == QEvent::MouseButtonPress)
        {
            toggleJointListWindow();
            return true;
        }
    }
    //must propogate events
    return QMainWindow::eventFilter(object,event);
}

void MiniJoint::toggleJointListWindow()
{
    if(jList->isVisible())
        jList->hide();
    else
        jList->show();
}

void MiniJoint::hideWindow()
{
    this->hide();
}

void MiniJoint::receiveJointData(int status, QString jointName)
{    
     switch(status)
     {
     case 0:
         //all joints must be fine
         ui->table->clearContents();
         ui->table->setRowCount(0);
         break;
     case 1:         
         joint = new QTableWidgetItem();
         jointStatus = new QTableWidgetItem();
         joint->setBackgroundColor(Qt::yellow);
         jointStatus->setBackgroundColor(Qt::yellow);
         ui->table->insertRow(0);
         joint->setText(jointName);
         jointStatus->setText("WARN");
         ui->table->setItem(0,0,jointStatus);
         ui->table->setItem(0,1,joint);
         break;
     case 2:         
         joint = new QTableWidgetItem();
         jointStatus = new QTableWidgetItem();
         joint->setBackgroundColor(Qt::red);
         jointStatus->setBackgroundColor(Qt::red);
         ui->table->insertRow(0);
         joint->setText(jointName);
         jointStatus->setText("ERROR");
         ui->table->setItem(0,0,jointStatus);
         ui->table->setItem(0,1,joint);
         break;
     }

}

void MiniJoint::enterEvent(QEvent * event)
{
    if(timer->isActive())
    {
        this->show();
        jointFadeIn->start();
        visible = true;
    }
}
void MiniJoint::leaveEvent(QEvent * event)
{
    if(visible)
    {
        jointFadeOut->start();
    }
    visible = false;
    timer->stop();

}

void MiniJoint::startActiveTimer()
{
    timer->start(500);
}


MiniJoint::~MiniJoint()
{
    delete(joint);
    delete(jointStatus);
    delete(jointFadeIn);
    delete(jointFadeOut);
    delete ui;
}
