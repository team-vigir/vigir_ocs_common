#include "map_view_widget.h"
#include "ui_map_view_widget.h"
#include "ui/template_loader_widget.h"
#include "joystick.h"


QRadioButton* joystick;
QRadioButton* wayPoints;
QPushButton* insertWaypoints;

MapViewWidget::MapViewWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::MapViewWidget)
{
    ui->setupUi(this);
    joystick = this->findChild<QRadioButton *>("joystick_steering");
    wayPoints = this->findChild<QRadioButton *>("waypoint");

    insertWaypoints = this->findChild<QPushButton *>("insert_waypoint");
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
}

void MapViewWidget::hideJoystick()
{
    insertWaypoints->show();
}

void MapViewWidget::requestMap()
{
    ui->map_view_->requestMap(ui->map_min_z->value(),ui->map_max_z->value(),ui->map_res->value());
}

void MapViewWidget::requestOctomap()
{
    ui->map_view_->requestOctomap(ui->oct_min_z->value(),ui->oct_max_z->value(),ui->oct_res->value());
}
