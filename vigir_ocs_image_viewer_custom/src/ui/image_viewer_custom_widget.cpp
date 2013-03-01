#include "image_viewer_custom_widget.h"
#include "ui_image_viewer_custom_widget.h"
#include "stdio.h"

#include <iostream>
#include <sstream>  // Required for stringstreams
#include <string>
#include <QPainter>
#include <QGridLayout>


//QPainter painter;
//QWidget* picture;
//QLabel* label1;
//QLabel* label2;
QComboBox *camera;
QLabel *pitchLabel;
QSlider *pitch;
QGroupBox *feedFPSBox;
QSlider *feedSlider;
QGroupBox *selectedFPSBox;
QSlider *selectedSlider;
QCheckBox *lockBox;
QPushButton *scanButton;
QGroupBox *handBox;
QGroupBox* pitchBox;
QObject* pitchParent;
QPushButton* undoButton;
QComboBox *actionBox;

ImageViewerCustomWidget::ImageViewerCustomWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::ImageViewerCustomWidget)
{
    ui->setupUi(this);

    camera = this->findChild<QComboBox *>("comboBox_7");
    connect(camera, SIGNAL(currentIndexChanged(int)), this, SLOT(alterDisplay(int)));

    pitchLabel = this->findChild<QLabel *>("label_8");
    pitch = this->findChild<QSlider *>("horizontalSlider");
    connect(pitch, SIGNAL(valueChanged(int)), this, SLOT(updatePitch(int)));

    feedFPSBox = this->findChild<QGroupBox *>("Mode_3");
    feedSlider = this->findChild<QSlider *>("horizontalSlider_2");
    connect(feedSlider, SIGNAL(valueChanged(int)), this, SLOT(updateFeedFPS(int)));
    selectedFPSBox = this->findChild<QGroupBox *>("groupBox_2");
    selectedSlider = this->findChild<QSlider *>("horizontalSlider_3");
    connect(selectedSlider, SIGNAL(valueChanged(int)), this, SLOT(updateSelectedFPS(int)));

    lockBox = this->findChild<QCheckBox *>("checkBox");
    connect(lockBox, SIGNAL(toggled(bool)), this, SLOT(isLocked()));

    scanButton= this->findChild<QPushButton *>("pushButton");
    connect(scanButton, SIGNAL(clicked()), this, SLOT(scan()));

    pitchBox = this->findChild<QGroupBox *>("Orientation_3");



    //This last part just creates the hand control box using the position
    // and size of the head pitch control box which it replaces.
    int x = 0; //x coordinate in parent.
    int y = 0; //y coordinate in parent.
    int width = 0; //width of the head pitch control box
    int height = 0; //height of the head pitch control box

    //grabs the parent so the box is placed correctly
    QWidget *parentWidget = pitchBox->parentWidget();
    x=pitchBox->x();
    y=pitchBox->y();
    handBox = new QGroupBox("Hand Controls", pitchBox->parentWidget());


    width = pitchBox->size().width();
    height = pitchBox->size().height();


    handBox->setMinimumSize(width,height);

    handBox->setMaximumSize(width,height);
    handBox->move(x,y);
    QGridLayout *mainLayout = new QGridLayout;


    undoButton = new QPushButton("Undo", handBox);

    actionBox = new QComboBox(handBox);
    //int id = actionBox.itemData(0);
    actionBox->addItem(QString("Open up hand"), 0);
    actionBox->addItem(QString("Look at feet"), 1);
    actionBox->addItem(QString("Option 3"), 2);
    actionBox->addItem(QString("Option 4"), 3);
    actionBox->addItem(QString("Option 5"), 4);

    mainLayout->addWidget(actionBox, 0,0,Qt::AlignCenter);
    mainLayout->addWidget(undoButton, 1,0,Qt::AlignCenter);

    handBox->setLayout(mainLayout);
    handBox->hide();
}

ImageViewerCustomWidget::~ImageViewerCustomWidget()
{
    delete ui;
}

/**
  * This method makes it so that both FPS sliders
  * maintain the same value when locked.
  */
void sliderValues(int lockedValue)
{
    feedSlider->setValue(lockedValue);
    selectedSlider->setValue(lockedValue);
}

/**
  * This method makes either the head pitch options or
  * the hand camera options visible based on which
  * camera is currently selected.
  * 0 and 1 refer to the head cameras, while 2 and 3
  * refer to the hand cameras
  **/
void ImageViewerCustomWidget::alterDisplay(int num)
{

    if(num== 0 || num == 1)
    {
        if(handBox != NULL)
        {

            handBox->hide();
        }

        pitchBox->show();
    }
    else
    {
        pitchBox->hide();


        handBox->show();

    }
}

/**
  * Thid method takes in the selected pitch on the pitch slider
  * and updates the label so that the value is clearly
  * shown.
  **/
void ImageViewerCustomWidget::updatePitch(int value)
{


    std::stringstream ss;//create a stringstream for int to string conversion
    ss << value;
    std::string string = ss.str();
    QString label = QString::fromStdString(string);
    pitchLabel->setText(label);
}

/**
  * Thid method takes in the selected fps on the feed slider
  * and updates the title so that the value is clearly
  * shown.
  **/
void ImageViewerCustomWidget::updateFeedFPS(int fps)
{


    std::string newTitle = "Feed Rate: ";

    std::stringstream ss;//create a stringstream for int to string conversion
    ss << fps;
    std::string string = ss.str();
    newTitle.append(string);
    newTitle.append(" FPS");
    QString label = QString::fromStdString(newTitle);
    if(lockBox->isChecked())
    {
        sliderValues(fps);
    }
    feedFPSBox->setTitle(label);

}

/**
  * Thid method takes in the selected fps on the selected area
  * slider and updates the title so that the value is clearly
  * shown.
  **/
void ImageViewerCustomWidget::updateSelectedFPS(int fps)
{

    std::string newTitle = "Selected Area Rate: ";
    std::stringstream ss;//create a stringstream for int to string conversion
    ss << fps;
    std::string string = ss.str();
    newTitle.append(string);
    newTitle.append(" FPS");
    QString label = QString::fromStdString(newTitle);
    if(lockBox->isChecked())
    {
        sliderValues(fps);
    }
    selectedFPSBox->setTitle(label);
}

/**
  * This method makes it so that the labeling
  * on the scan button changes. Later on, it
  * should toggle the scan behavior.
  **/
void ImageViewerCustomWidget::scan()
{
    if(scanButton->text() == "Start Scanning")
    {
        //execute the scanning action
        scanButton->setText("Stop Scanning");
    }
    else
    {
        //stop the scanning action
        scanButton->setText("Start Scanning");
    }
}


/**
  * This method is just used to make it so that as
  * soon as "Locked" is checked, the values are made
  * to be the same.
  **/
void ImageViewerCustomWidget::isLocked()
{

    if(lockBox->isChecked())
    {
        sliderValues(feedSlider->value());
    }

}
