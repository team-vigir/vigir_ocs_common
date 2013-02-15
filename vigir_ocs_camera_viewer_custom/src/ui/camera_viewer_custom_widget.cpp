#include "camera_viewer_custom_widget.h"
#include "ui_camera_viewer_custom_widget.h"
#include "stdio.h"
#include <iostream>
#include <QPainter>


//QPainter painter;
//QWidget* picture;
//QLabel* label1;
//QLabel* label2;
CameraViewerCustomWidget::CameraViewerCustomWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::CameraViewerCustomWidget)
{
    ui->setupUi(this);

    QComboBox *camera = this->findChild<QComboBox *>("comboBox_7");
    connect(camera, SIGNAL(currentIndexChanged(int)), this, SLOT(alterDisplay(int)));
}

CameraViewerCustomWidget::~CameraViewerCustomWidget()
{
    delete ui;
}

void CameraViewerCustomWidget::alterDisplay(int num)
{
    QGroupBox* groupBox = this->findChild<QGroupBox *>("Orientation_3");
    if(num== 0 || num == 1)
    {
        groupBox->show();
        groupBox = this->findChild<QGroupBox *>("groupBox_2");
        groupBox->show();

    }
    else
    {


        groupBox->hide();
        groupBox = this->findChild<QGroupBox *>("groupBox_2");
        groupBox->hide();

    }
}
