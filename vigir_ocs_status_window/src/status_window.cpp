#include "status_window.h"
#include "ui_status_window.h"
#include "jointList.h"
#include <QtGui>

status_window::status_window(QWidget *parent) :
     QWidget(parent),
    ui(new Ui::status_window)
{
    ui->setupUi(this);
    par= QWidget::window ();
    QPalette palette(ui->stabilityMap->palette());
    palette.setColor(backgroundRole(), Qt::white);
    ui->stabilityMap->setPalette(palette);

    QPainter painter(ui->stabilityMap);
    painter.setRenderHint(QPainter::Antialiasing);
    painter.setPen(Qt::darkGreen);
    painter.isActive();
}

status_window::~status_window()
{
    delete ui;
}
void status_window::paint()
{
    painter.drawRect(1,1,20,20);
}

void status_window::on_showJointButton_clicked()
{
    if(ui->showJointButton->text() == "Show Joint List")
    {
        ui->showJointButton->setText("Hide Joint List");
        //ui->stabilityMap->update();
        jntList = new jointList(NULL);
        jntList->show();
    }
    else
    {
        //delete jntList;

        ui->showJointButton->setText("Show Joint List");
    }
}
