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
#include "JoystickWidget.h"
#include "ui_JoystickWidget.h"
#include <QLabel>
#include <QSignalMapper>
#include <QDebug>
#include <ros/package.h>

#include <vigir_ocs_msgs/WindowCodes.h>

JoystickWidget::JoystickWidget(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::JoystickWidget)
{
    ui->setupUi(this);

    ros::NodeHandle nh;

    controller = new vigir_ocs::Controller();
    mapper = new QSignalMapper(this);
    connect(mapper,SIGNAL(mapped(QString)),this,SLOT(setDirection(QString)));

    //map all directional button to unique strings
    mapper->setMapping(this->ui->UpLButton,"L,Up");
    mapper->setMapping(this->ui->UpRButton,"R,Up");
    mapper->setMapping(this->ui->LeftLButton,"L,Left");
    mapper->setMapping(this->ui->LeftRButton,"R,Left");
    mapper->setMapping(this->ui->RightLButton,"L,Right");
    mapper->setMapping(this->ui->RightRButton,"R,Right");
    mapper->setMapping(this->ui->DownLButton,"L,Down");
    mapper->setMapping(this->ui->DownRButton,"R,Down");

    //connect all buttons for mouse presses
    connect(ui->UpLButton,SIGNAL(pressed()),mapper,SLOT(map()));
    connect(ui->UpRButton,SIGNAL(pressed()),mapper,SLOT(map()));
    connect(ui->LeftLButton,SIGNAL(pressed()),mapper,SLOT(map()));
    connect(ui->LeftRButton,SIGNAL(pressed()),mapper,SLOT(map()));
    connect(ui->RightLButton,SIGNAL(pressed()),mapper,SLOT(map()));
    connect(ui->RightRButton,SIGNAL(pressed()),mapper,SLOT(map()));
    connect(ui->DownLButton,SIGNAL(pressed()),mapper,SLOT(map()));
    connect(ui->DownRButton,SIGNAL(pressed()),mapper,SLOT(map()));
    connect(ui->selectTemplate,SIGNAL(pressed()),this,SLOT(selectTemplate()));

    //connect buttons for mouse release(reset labels)
    connect(ui->UpLButton,SIGNAL(released()),this,SLOT(disableLeftLabel()));
    connect(ui->UpRButton,SIGNAL(released()),this,SLOT(disableRightLabel()));
    connect(ui->LeftLButton,SIGNAL(released()),this,SLOT(disableLeftLabel()));
    connect(ui->LeftRButton,SIGNAL(released()),this,SLOT(disableRightLabel()));
    connect(ui->RightLButton,SIGNAL(released()),this,SLOT(disableLeftLabel()));
    connect(ui->RightRButton,SIGNAL(released()),this,SLOT(disableRightLabel()));
    connect(ui->DownLButton,SIGNAL(released()),this,SLOT(disableLeftLabel()));
    connect(ui->DownRButton,SIGNAL(released()),this,SLOT(disableRightLabel()));

    //update combo box based on signal from controller
    connect(controller,SIGNAL(updateTemplateComboBox(int)),this,SLOT(populateTemplateComboBox(int)));
    //selection within combox box
    connect(ui->templateComboBox,SIGNAL(currentIndexChanged(int)),controller,SLOT(changeTemplateID(int)));

    //combo boxes for changing modes
    connect(ui->manipulateModeBox,SIGNAL(currentIndexChanged(int)),controller,SLOT(setManipulation(int)));
    connect(ui->objectModeBox,SIGNAL(currentIndexChanged(int)),controller,SLOT(setObjectMode(int)));

    //update comboBox on external mode change from controller(controller constantly subscribes)
    connect(controller,SIGNAL(updateUIModes(int,int)),this,SLOT(updateModeBoxes(int,int)));

    std::string ip = ros::package::getPath("vigir_ocs_template_joystick")+"/icons/";
    icon_path_ = QString(ip.c_str());

    //place graphics on buttons
    QPixmap upArrow(icon_path_+ "up.png");
    QPixmap rightArrow(icon_path_+ "right.png");
    QPixmap leftArrow(icon_path_+ "left.png");
    QPixmap downArrow(icon_path_+ "down.png");
    QIcon up(upArrow);
    QIcon right(rightArrow);
    QIcon left(leftArrow);
    QIcon down(downArrow);
    ui->UpRButton->setIcon(up);
    ui->UpRButton->setIconSize(upArrow.rect().size()/2);
    ui->UpLButton->setIcon(up);
    ui->UpLButton->setIconSize(upArrow.rect().size()/2);
    ui->LeftLButton->setIcon(left);
    ui->LeftLButton->setIconSize(leftArrow.rect().size()/2);
    ui->LeftRButton->setIcon(left);
    ui->LeftRButton->setIconSize(leftArrow.rect().size()/2);
    ui->DownLButton->setIcon(down);
    ui->DownLButton->setIconSize(downArrow.rect().size()/2);
    ui->DownRButton->setIcon(down);
    ui->DownRButton->setIconSize(downArrow.rect().size()/2);
    ui->RightLButton->setIcon(right);
    ui->RightLButton->setIconSize(rightArrow.rect().size()/2);
    ui->RightRButton->setIcon(right);
    ui->RightRButton->setIconSize(rightArrow.rect().size()/2);

    window_control_sub = nh.subscribe<std_msgs::Int8>( "/flor/ocs/window_control", 5, &JoystickWidget::processWindowControl, this );
    window_control_pub = nh.advertise<std_msgs::Int8>("/flor/ocs/window_control", 1, false);

    //Restore State
    //this->show();
    QSettings settings("OCS", "template_joystick");
    this->restoreGeometry(settings.value("mainWindowGeometry").toByteArray());
    this->geometry_ = this->geometry();
    // create docks, toolbars, etc...
    this->restoreState(settings.value("mainWindowState").toByteArray());
}

JoystickWidget::~JoystickWidget()
{
    delete(controller);
    delete(mapper);
    delete ui;
}

void JoystickWidget::closeEvent(QCloseEvent * event)
{
    QSettings settings("OCS", "template_joystick");
    settings.setValue("mainWindowGeometry", this->saveGeometry());
    settings.setValue("mainWindowState", this->saveState());
    std_msgs::Int8 msg;
    msg.data = -WINDOW_JOYSTICK;
    window_control_pub.publish(msg);
    event->ignore();
}

void JoystickWidget::resizeEvent(QResizeEvent * event)
{
    QSettings settings("OCS", "template_joystick");
    settings.setValue("mainWindowGeometry", this->saveGeometry());
    settings.setValue("mainWindowState", this->saveState());

}

void JoystickWidget::moveEvent(QMoveEvent * event)
{
    QSettings settings("OCS", "template_joystick");
    settings.setValue("mainWindowGeometry", this->saveGeometry());
    settings.setValue("mainWindowState", this->saveState());

}

void JoystickWidget::updateModeBoxes(int manipulationMode,int objectMode)
{
    ui->manipulateModeBox->setCurrentIndex(manipulationMode);
    ui->objectModeBox->setCurrentIndex(objectMode);
}

void JoystickWidget::populateTemplateComboBox(int tempId)
{
    //clear comboBox
    while(ui->templateComboBox->count()>0)
    {
        ui->templateComboBox->removeItem(0);
    }

    std::vector<std::string> templates = controller->getTemplateNames();
    int size = templates.size();
    //add new updated contents from vector
    for(int i =0;i<size;i++)
    {
        ui->templateComboBox->addItem(QString::fromStdString(templates[i]));
    }
    //make item selected
    ui->templateComboBox->setCurrentIndex(tempId);
}

void JoystickWidget::selectTemplate()
{
    controller->changeTemplate();
}

//clear text to display nothing when no button is pressed
void JoystickWidget::disableLeftLabel()
{
    ui->directionL->setText("");
}
void JoystickWidget::disableRightLabel()
{
    ui->directionR->setText("");
}

//set direction based on unique strings
void JoystickWidget::setDirection(QString str)
{
    float x = 0;
    float z = 0;
    float rotX = 0;
    float rotY = 0;
    if(str.compare("L,Up") == 0)
    {
        ui->directionL->setText("Up");
        z = .01;
    }
    else if (str.compare("R,Up") == 0)
    {
        ui->directionR->setText("Up");
        rotY = 5;
    }
    else if (str.compare("L,Left") == 0)
    {
        ui->directionL->setText("Left");
        x = -.01;
    }
    else if (str.compare("R,Left") == 0)
    {
        ui->directionR->setText("Left");
        rotX = -5;
    }
    else if (str.compare("L,Right") == 0)
    {
        ui->directionL->setText("Right");
        x = .01;
    }
    else if (str.compare("R,Right") == 0)
    {
        ui->directionR->setText("Right");
        rotX = 5;
    }
    else if (str.compare("L,Down") == 0)
    {
        ui->directionL->setText("Down");
        z = -.01;
    }
    else if (str.compare("R,Down") == 0)
    {
        ui->directionR->setText("Down");
        rotY = -5;
    }

    controller->buildmsg(x,z,rotX,rotY);
}

void JoystickWidget::keyPressEvent(QKeyEvent *event)
{
    keysPressed.insert((Qt::Key) event->key());
    processKeys();
}

void JoystickWidget::keyReleaseEvent(QKeyEvent *event)
{
    keysPressed.remove((Qt::Key)event->key());

    if(event->key() == Qt::Key_W ||event->key() == Qt::Key_A || event->key() == Qt::Key_S || event->key() == Qt::Key_D )
        disableLeftLabel();
    else if (event->key() == Qt::Key_I ||event->key() == Qt::Key_J || event->key() == Qt::Key_K || event->key() == Qt::Key_L)
        disableRightLabel();
    //ui->glwidget->keyReleaseEvent(event);
}

//handles multiple/single keys pressed
void JoystickWidget::processKeys()
{
    float x = 0;
    float z = 0;
    float rotX = 0;
    float rotY = 0;

    if(keysPressed.contains(Qt::Key_W))
    {
        ui->directionL->setText("Up");
        z = .1;
    }
    if(keysPressed.contains(Qt::Key_A))
    {
        ui->directionL->setText("Left");
        x = -0.1;
    }
    if(keysPressed.contains(Qt::Key_S))
    {
        ui->directionL->setText("Down");
        z = -.1;
    }
    if(keysPressed.contains(Qt::Key_D))
    {
        ui->directionL->setText("Right");
        x = .1;
    }
    if(keysPressed.contains(Qt::Key_I))
    {
        ui->directionR->setText("Up");
        rotY = 5;
    }
    if(keysPressed.contains(Qt::Key_J))
    {
        ui->directionR->setText("Left");
        rotX = -5;
    }
    if(keysPressed.contains(Qt::Key_K))
    {
        ui->directionR->setText("Down");
        rotY = -5;
    }
    if(keysPressed.contains(Qt::Key_L))
    {
        ui->directionR->setText("Right");
        rotX = 5;
    }
    controller->buildmsg(x,z,rotX,rotY);
}

void JoystickWidget::processWindowControl(const std_msgs::Int8::ConstPtr& msg)
{
    if(!isVisible() && msg->data == WINDOW_JOYSTICK)
    {
        this->show();
        this->setGeometry(geometry_);
    }
    else if(isVisible() && (!msg->data || msg->data == -WINDOW_JOYSTICK))
    {
        geometry_ = this->geometry();
        this->hide();
    }
}
