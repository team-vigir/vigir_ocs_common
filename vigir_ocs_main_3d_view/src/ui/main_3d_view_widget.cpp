#include "main_3d_view_widget.h"
#include "ui_main_3d_view_widget.h"

Main3DViewWidget::Main3DViewWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Main3DViewWidget)
{
    ui->setupUi(this);
}

Main3DViewWidget::~Main3DViewWidget()
{
    delete ui;
}
