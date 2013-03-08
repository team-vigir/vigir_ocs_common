#include "status_window.h"
#include "ui_status_window.h"
//#include <flor_ocs_msgs/2DPoint.h>
//#include <flor_ocs_msgs/CenterGrav.h>
#include "jointList.h"
#include <QtGui>
#include <QPainter>

status_window::status_window(QWidget *parent) :
     QWidget(parent),
    ui(new Ui::status_window)
{
    ui->setupUi(this);
    par= QWidget::window ();
    //QPalette palette(ui->stabilityMap->palette());
    //palette.setColor(backgroundRole(), Qt::white);
    //ui->stabilityMap->setPalette(palette);
    numberOfPoints = 3;
    points[0] = QPoint(80,30);
    points[1] = QPoint(60,60);
    points[2] = QPoint(100,60);
    QPoint cg = QPoint(80,50);
    //QPainter painter(ui->stabilityMap);
    //ui->stabilityMap->update();
    painter.begin(ui->stabilityMap);
    //ui->stabilityMap->update();
}

status_window::~status_window()
{
    delete ui;
}
//void status_window::updatePoints(const )
void status_window::paintEvent(QPaintEvent *event)
{
    //if(painter.isActive())
      //  std::cout << "Painter Active!!!" << std::endl;
    painter.setPen(Qt::black);
    for (int i=0;i<numberOfPoints-1;i++)
    {
        painter.drawLine(points[i],points[i+1]);
        //painter.drawLine(ui->stabilityMap->size().width()*points[i*2],ui->stabilityMap->size().height()*points[i*2+1],ui->stabilityMap->size().width()*points[i*2+2],ui->stabilityMap->size().height()*points[i*2+3]);
    }
    painter.drawLine(points[0],points[numberOfPoints-1]);
    painter.setPen(Qt::darkGreen);
    painter.drawEllipse(cg,5,5);
    //ui->stabilityMap->update();
}

//void status_window::paint()
//{
//    painter.drawRect(1,1,20,20);
//}


void status_window::on_showJointButton_clicked()
{
        //ui->stabilityMap->update();
        jntList = new jointList(NULL);
        jntList->show();
    //ui->stabilityMap->update();
}

void status_window::on_showRobotStatus_clicked()
{
    rbtStatus = new robotStatus(NULL);
    rbtStatus->show();
}
