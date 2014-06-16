
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

void MiniJoint::hideWindow()
{
    this->hide();
}

void MiniJoint::receiveJointData(int status, QString jointName)
{
    //ROS_ERROR("Joint %d %s", status, qPrintable(jointName));
     switch(status)
     {
     case 0: //no need to know about joints that are okay
         break;
     case 1:
         removeDuplicates(status,jointName);
         joint = new QTableWidgetItem();
         jointStatus = new QTableWidgetItem();
         ui->table->insertRow(0);
         joint->setText(jointName);
         jointStatus->setText("WARN");
         ui->table->setItem(0,0,jointStatus);
         ui->table->setItem(0,1,joint);
         break;
     case 2:
         removeDuplicates(status,jointName);
         joint = new QTableWidgetItem();
         jointStatus = new QTableWidgetItem();
         ui->table->insertRow(0);
         joint->setText(jointName);
         jointStatus->setText("ERROR");
         ui->table->setItem(0,0,jointStatus);
         ui->table->setItem(0,1,joint);
         break;
     }

}

//searches table for joint. if found will delete entry/row
void MiniJoint::removeDuplicates(int status, QString jointName)
{
    for(int i=0;i<ui->table->rowCount();i++)
    {
         if (ui->table->item(i,1)->text().compare(jointName) == 0) // found joint?
         {
             ui->table->removeRow(i);
             return; // can only have one duplicate max
         }
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
