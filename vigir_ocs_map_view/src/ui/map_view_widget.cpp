#include "map_view_widget.h"
#include "ui_map_view_widget.h"
#include "ui/template_loader_widget.h"
#include "joystick.h"

MapViewWidget::MapViewWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::MapViewWidget)
{
    ui->setupUi(this);

    ui->insert_waypoint->hide();

    connect(ui->joystick_steering, SIGNAL(toggled(bool)), this, SLOT(hideWaypointButton()));
    connect(ui->waypoint, SIGNAL(toggled(bool)), this, SLOT(hideJoystick()));
}

MapViewWidget::~MapViewWidget()
{
    delete ui;
}

void MapViewWidget::hideWaypointButton()
{
    ui->insert_waypoint->hide();
}

void MapViewWidget::hideJoystick()
{
    ui->insert_waypoint->show();
}

void MapViewWidget::requestMap()
{
    ui->map_view_->requestMap(ui->map_min_z->value(),ui->map_max_z->value(),ui->map_res->value());
}

void MapViewWidget::requestOctomap()
{
    ui->map_view_->requestOctomap(ui->oct_min_z->value(),ui->oct_max_z->value(),ui->oct_res->value());
}
