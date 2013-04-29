#include "tools_options_widget.h"
#include "ui_tools_options_widget.h"
#include "stdio.h"
#include <iostream>
#include <QPainter>
#include <QtGui>

#include <ros/package.h>

ToolsOptionsWidget::ToolsOptionsWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::ToolsOptionsWidget)
{
    ui->setupUi(this);
}

ToolsOptionsWidget::~ToolsOptionsWidget()
{
    delete ui;
}
