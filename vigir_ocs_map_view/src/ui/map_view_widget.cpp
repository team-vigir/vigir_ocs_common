#include "map_view_widget.h"
#include "ui_map_view_widget.h"

MapViewWidget::MapViewWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::MapViewWidget)
{
    ui->setupUi(this);
}

MapViewWidget::~MapViewWidget()
{
    delete ui;
}
