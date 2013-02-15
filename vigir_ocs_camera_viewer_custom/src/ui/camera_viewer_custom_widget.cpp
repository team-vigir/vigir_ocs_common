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

    // mouseClicked = false;
//     button = this->findChild<QRadioButton *>("radioButton_9");
//     dynamicButton = this->findChild<QRadioButton *>("radioButton_7");
//     staticButton = this->findChild<QRadioButton *>("radioButton_8");
//     frameBox = this->findChild<QSpinBox *>("spinBox_3");
//     timeBox = this->findChild<QSpinBox *>("spinBox");
     camera = this->findChild<QComboBox *>("comboBox_7");
//     label1 = this->findChild<QLabel *>("label");
//     label2 = this->findChild<QLabel *>("label_2");


//     picture = this->findChild<QWidget *>("widget");

//     drawHelper* helper = new drawHelper(picture);
//     helper->resize(picture->size());
//     helper->show();
     connect(camera, SIGNAL(currentIndexChanged(int)), this, SLOT(alterDisplay(int)));

//     connect(button, SIGNAL(clicked()), this, SLOT(enableSpin()));
//    // connect(button, SIGNAL(clicked()), this, SLOT(changeValue()));
//     connect(dynamicButton, SIGNAL(clicked()), this, SLOT(disableSpin()));
//     connect(staticButton, SIGNAL(clicked()), this, SLOT(disableSpin()));

}

CameraViewerCustomWidget::~CameraViewerCustomWidget()
{
    delete ui;
}

/*

void paintWindow()
{
    std::cout<<"hits";
}
*/
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
/*
void CameraViewerCustomWidget::enableSpin()
{
    if(button->isChecked())
    {
//        frameBox->setEnabled(true);
//        timeBox->setEnabled(true);
//        label1->setEnabled(true);
//        label2->setEnabled(true);


    }
}

void CameraViewerCustomWidget::disableSpin()
{
//    timeBox->setEnabled(false);
//    frameBox->setEnabled(false);
//    label1->setEnabled(false);
//    label2->setEnabled(false);

}*/






