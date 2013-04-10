#include "graspWidget.h"
#include "ui_graspWidget.h"
#include <QColor>

graspWidget::graspWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::graspWidget)
{
    ui->setupUi(this);
    //QPalette pal = ui->graspGraph->palette();
    //pal.setColor(QPalette::Active, QPalette::Highlight,Qt::blue);
    //pal.setColor(QPalette::Inactive, QPalette::Highlight,Qt::blue);
    //ui->graspGraph->setPalette(pal);
}
//SetStylesheet to change on the fly
graspWidget::~graspWidget()
{
    delete ui;
}
