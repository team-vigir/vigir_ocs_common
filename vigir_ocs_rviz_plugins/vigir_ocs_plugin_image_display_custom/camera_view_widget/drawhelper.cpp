#include "drawhelper.h"
#include <cmath>

#include <QWidget>
#include <QMouseEvent>
#include <iostream>
#include <QPainter>
bool mouseClicked = false;
bool flag = false;
QPoint initialPoint;
QPoint finalPoint;
int rectArray [4][20];
int counter =-1;

drawHelper::drawHelper(QWidget *parent) :
    QWidget(parent)
{

    this->resize(QSize(600,600));
    this->setMouseTracking(true);
}

void QWidget::mousePressEvent(QMouseEvent *mouseOne)
{

    mouseClicked = true;
    initialPoint = mouseOne->pos();
   // painter = new QPainter();
}



void QWidget::mouseMoveEvent(QMouseEvent *mouseTwo)
{
    if(mouseClicked == true)
    {
        finalPoint = mouseTwo->pos();
    }

    if(initialPoint.x() > this->parentWidget()->width())
    {
        initialPoint = QPoint(parentWidget()->width(), initialPoint.y());
    }
    else if(initialPoint.x() < 0)
    {
        initialPoint = QPoint(0, initialPoint.y());
    }


    if(initialPoint.y() > this->parentWidget()->width())
    {
        initialPoint = QPoint(parentWidget()->width(), initialPoint.y());
    }
    else if(initialPoint.y() < 0)
    {
        initialPoint = QPoint(0, initialPoint.y());
    }


    if(finalPoint.x() > this->parentWidget()->width())
    {
        finalPoint = QPoint(parentWidget()->width(), finalPoint.y());
    }
    else if(finalPoint.x() < 0)
    {
        finalPoint = QPoint(0, finalPoint.y());
    }

    if(finalPoint.y() > this->parentWidget()->height())
    {
        finalPoint = QPoint(finalPoint.x(),parentWidget()->height());
    }
    else if(finalPoint.y() < 0)
    {
        finalPoint = QPoint(finalPoint.x(), 0);
    }
    if(flag == false)
    {
        counter++;
        flag=true;
    }
    this->repaint();


}

void QWidget::mouseReleaseEvent(QMouseEvent *mouseThree)
{
    if(mouseClicked == true)
    {
        mouseClicked = false;
        finalPoint = mouseThree->pos();



        if(initialPoint.x() > this->parentWidget()->width())
        {
            initialPoint = QPoint(parentWidget()->width(), initialPoint.y());
        }
        else if(initialPoint.x() < 0)
        {
            initialPoint = QPoint(0, initialPoint.y());
        }


        if(initialPoint.y() > this->parentWidget()->width())
        {
            initialPoint = QPoint(parentWidget()->width(), initialPoint.y());
        }
        else if(initialPoint.y() < 0)
        {
            initialPoint = QPoint(0, initialPoint.y());
        }


        if(finalPoint.x() > this->parentWidget()->width())
        {
            finalPoint = QPoint(parentWidget()->width(), finalPoint.y());
        }
        else if(finalPoint.x() < 0)
        {
            finalPoint = QPoint(0, finalPoint.y());
        }

        if(finalPoint.y() > this->parentWidget()->height())
        {
            finalPoint = QPoint(finalPoint.x(),parentWidget()->height());
        }
        else if(finalPoint.y() < 0)
        {
            finalPoint = QPoint(finalPoint.x(), 0);
        }
       // counter++;
        flag = false;
        this->repaint();
    }
  //  picture->paintEvent(QPaintEvent *paint = new QPaintEvent*);
     //std::cout<<"This is hit";


}




void QWidget::paintEvent(QPaintEvent *paint)
{


    QPainter painter(this);
   // painter.begin(this);
    int xValue = 0;
    int yValue = 0;
    painter.setRenderHint(QPainter::Antialiasing);
    painter.setPen(Qt::cyan);
    if(finalPoint.y() < initialPoint.y())
    {
        yValue = finalPoint.y();
    }
    else
    {
        yValue = initialPoint.y();
    }

    if(finalPoint.x() < initialPoint.x())
    {
        xValue = finalPoint.x();
    }
    else
    {
        xValue = initialPoint.x();
    }


        rectArray[0][counter] = xValue;
        rectArray[1][counter] = yValue;
        rectArray[2][counter] = abs( initialPoint.x() - finalPoint.x());
        rectArray[3][counter] = abs(initialPoint.y() - finalPoint.y());
     //   std::cout<< "This is the counter value " << counter ;
        for(int x = 0; x<=counter; x++)
        {
            //  std::cout<< "Gets in here " << rectArray[0][x]<< std::endl;
    //        painter.drawRect(xValue, yValue, abs( initialPoint.x() - finalPoint.x()),
    //                         abs(initialPoint.y() - finalPoint.y()));
            painter.drawRect( rectArray[0][x], rectArray[1][x], rectArray[2][x], rectArray[3][x]);
        }


     //   painter.drawRect(xValue, yValue, abs( initialPoint.x() - finalPoint.x()),
       //                             abs(initialPoint.y() - finalPoint.y()));



   // painter.end();
}


