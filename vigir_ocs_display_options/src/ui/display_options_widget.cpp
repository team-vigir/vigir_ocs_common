#include "display_options_widget.h"
#include "ui_display_options_widget.h"
#include "stdio.h"
#include <iostream>
#include <QPainter>
#include <QtGui>

#include <ros/package.h>

DisplayOptionsWidget::DisplayOptionsWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::DisplayOptionsWidget)
{
    ui->setupUi(this);
}

DisplayOptionsWidget::~DisplayOptionsWidget()
{
    delete ui;
}
