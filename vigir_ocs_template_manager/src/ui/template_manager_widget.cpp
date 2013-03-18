#include "template_manager_widget.h"
#include "ui_template_manager_widget.h"
#include "stdio.h"
#include <iostream>
#include <QPainter>
#include <QtGui>

TemplateManagerWidget::TemplateManagerWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::TemplateManagerWidget),
    templateDirPath(QString("/opt/vigir/rosbuild_ws/vigir_control/vigir_grasping/templates/")) // should read this from file
{
    ui->setupUi(this);

}

TemplateManagerWidget::~TemplateManagerWidget()
{
    delete ui;
}

