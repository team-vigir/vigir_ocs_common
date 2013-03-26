#include "map_view_widget.h"
#include "ui_map_view_widget.h"
#include "ui/template_loader_widget.h"
#include "joystick.h"


QRadioButton* joystick;
QRadioButton* wayPoints;
QPushButton* insertWaypoints;
QWidget* joystickWidget;

MapViewWidget::MapViewWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::MapViewWidget)
{
    ui->setupUi(this);
    joystick = this->findChild<QRadioButton *>("joystick_steering");
    wayPoints = this->findChild<QRadioButton *>("waypoint");

    insertWaypoints = this->findChild<QPushButton *>("insert_waypoint");
    joystickWidget= this->findChild<Joystick *>("joystick_widget");
  //  joystickWidget->setSizePolicy(QSizePolicy::Maximum,QSizePolicy::Maximum);
    //joystickWidget->setSize(250, 150);
    insertWaypoints->hide();
    connect(joystick, SIGNAL(toggled(bool)), this, SLOT(hideWaypointButton()));
    connect(wayPoints, SIGNAL(toggled(bool)), this, SLOT(hideJoystick()));


}

MapViewWidget::~MapViewWidget()
{
    delete ui;
}

void MapViewWidget::hideWaypointButton()
{
    insertWaypoints->hide();
    joystickWidget->show();
}

void MapViewWidget::hideJoystick()
{
    joystickWidget->hide();
    insertWaypoints->show();
}
