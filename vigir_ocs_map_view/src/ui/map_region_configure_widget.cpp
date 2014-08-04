#include "map_region_configure_widget.h"
#include "ui_map_region_configure_widget.h"

#include <rviz/displays_panel.h>

MapRegionConfigure::MapRegionConfigure(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::MapRegionConfigure)
{
    ui->setupUi(this);
}

double MapRegionConfigure::getMinHeight()
{
    return ui->minHeight->value();
}

double MapRegionConfigure::getMaxHeight()
{
    return ui->maxHeight->value();
}

double MapRegionConfigure::getResolution()
{
    return ui->resolution->value();
}

MapRegionConfigure::~MapRegionConfigure()
{
    delete ui;
}


