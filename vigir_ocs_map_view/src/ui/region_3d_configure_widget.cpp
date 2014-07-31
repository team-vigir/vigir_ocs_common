#include "region_3d_configure_widget.h"
#include "ui_region_3d_configure_widget.h"

Region3DConfigure::Region3DConfigure(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Region3DConfigure)
{
    ui->setupUi(this);
}

double Region3DConfigure::getMinHeight()
{
    return ui->minHeight->value();
}

double Region3DConfigure::getMaxHeight()
{
    return ui->maxHeight->value();
}

double Region3DConfigure::getVoxelResolution()
{
    return ui->voxRes->value();
}

int Region3DConfigure::getAggregSize()
{
    return ui->aggSize->value();
}

Region3DConfigure::~Region3DConfigure()
{
    delete ui;
}
