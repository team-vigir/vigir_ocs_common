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

#include "ui_miniError.h"
#include "miniError.h"


MiniError::MiniError(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MiniError)
{
    ui->setupUi(this);

    timer = new QTimer(this);

    robStatus = new robotStatus();
    //connect signals to retrieve error data
    connect(robStatus,SIGNAL(sendErrorData(QString,QString)),this,SLOT(receiveErrorData(QString, QString)));
    connect(robStatus,SIGNAL(sendErrorData(QString,QString)),this->parent(),SLOT(receiveErrorData(QString, QString)));
    //
    connect(this->parent(),SIGNAL(makeErrorActive()),this,SLOT(startActiveTimer()));

    //connect(ui->table,SIGNAL(cellClicked(int,int)),this,SLOT(mouseClick(int,int)));
    //connect(ui->table,SIGNAL(itemClicked(QTableWidgetItem)),this,SLOT(mouseClick(QTableWidgetItem)));
    ui->table->setColumnWidth(0,100);
    ui->table->setColumnWidth(1,300);
    ui->table->setMouseTracking( true );
    ui->table->viewport()->installEventFilter( this );

    //build animations
    errorFadeIn = new QPropertyAnimation(this, "windowOpacity");
    errorFadeIn->setEasingCurve(QEasingCurve::InOutQuad);
    errorFadeIn->setDuration(500);
    errorFadeIn->setStartValue(0.0);
    errorFadeIn->setEndValue(.75);

    errorFadeOut = new QPropertyAnimation(this, "windowOpacity");
    errorFadeOut->setEasingCurve(QEasingCurve::InOutQuad);
    errorFadeOut->setDuration(300);
    errorFadeOut->setStartValue(0.75);
    errorFadeOut->setEndValue(0.0);

    connect(errorFadeOut,SIGNAL(finished()),this,SLOT(hideWindow()));

    newErrors = 0;

    //originalGeometry

    visible = false;
}

void MiniError::hideWindow()
{
    this->hide();
}

bool MiniError::eventFilter(QObject* object,QEvent* event)
{
    if (object == ui->table->viewport())
    {
        QMouseEvent *mouseEvent = static_cast<QMouseEvent*>(event);
        if(mouseEvent->button() == 1 && mouseEvent->type() == QEvent::MouseButtonPress)
        {
            toggleErrorLogWindow();
            return true;
        }

    }
    return QMainWindow::eventFilter(object,event);
}

void MiniError::toggleErrorLogWindow()
{
    if(robStatus->isVisible())
        robStatus->hide();
    else
        robStatus->show();
}

void MiniError::receiveErrorData(QString time, QString message)
{
    //used for insertions
    simTime = new QTableWidgetItem();
    errorMessage = new QTableWidgetItem();

    //insert new row on new error
    ui->table->insertRow(0);
    simTime->setText(time);
    simTime->setBackgroundColor(Qt::red);
    errorMessage->setText(message);
    errorMessage->setBackgroundColor(Qt::red);
    ui->table->setItem(0,0,simTime);
    ui->table->setItem(0,1,errorMessage);
    ui->table->setRowCount(5);// only last 5 errors maximum
    newErrors++;
}

void MiniError::enterEvent(QEvent * event)
{
    if(timer->isActive())
    {
        this->show();
        errorFadeIn->start();
        visible = true;
    }

}

void MiniError::setViewed()
{
    //set errors as viewed
    for(int i=0;i<newErrors;i++)
    {
        ui->table->item(i,0)->setBackgroundColor(Qt::white);
        ui->table->item(i,1)->setBackgroundColor(Qt::white);
    }
    newErrors = 0;
}

void MiniError::leaveEvent(QEvent * event)
{
    if(visible)
    {
        errorFadeOut->start();
    }


    visible = false;
    timer->stop(); //ensure stopped
}

void MiniError::startActiveTimer()
{
    timer->start(500);
}

MiniError::~MiniError()
{
    delete(simTime);
    delete(errorMessage);
    delete(errorFadeIn);
    delete(errorFadeOut);
    delete ui;
}


