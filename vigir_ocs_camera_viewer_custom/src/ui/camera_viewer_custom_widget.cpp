#include "camera_viewer_custom_widget.h"
#include "ui_camera_viewer_custom_widget.h"
#include "stdio.h"

#include <iostream>
#include <sstream>  // Required for stringstreams
#include <string>
#include <QPainter>
#include <QObjectList>
#include <QGridLayout>


//QPainter painter;
//QWidget* picture;
//QLabel* label1;
//QLabel* label2;
QComboBox *camera;
QLabel *pitchLabel;
QSlider *pitch;
QGroupBox *feedFPSBox; //frame rate box
QSlider *feedSlider;
QGroupBox *selectedFPSBox;
QSlider *selectedSlider;
QCheckBox *lockBox;
QPushButton *scanButton;
QGroupBox *handBox;
QGroupBox* pitchBox; //head panel
QObject* pitchParent;
QPushButton* undoButton;
QComboBox *actionBox;

QComboBox *feedResolution;
QComboBox *selectedResolution;

QWidget *scrollingArea;
QLayout* scrollLayout;

QGroupBox *displayBox;
QGroupBox *areaFeedBox;
QGroupBox *imageBox;
QGroupBox *cameraBox;


QObjectList displayChildren;
QObjectList imageChildren;
QObjectList resolutionChildren;
QObjectList cameraChildren;
QObjectList headChildren;
QObjectList rateChildren;

int currentIndex=0;
int headControlIndex = 0;

CameraViewerCustomWidget::CameraViewerCustomWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::CameraViewerCustomWidget)
{
    ui->setupUi(this);
    this->setMouseTracking(true);
    camera = this->findChild<QComboBox *>("comboBox_7");
    connect(camera, SIGNAL(currentIndexChanged(int)), this, SLOT(alterDisplay(int)));

    pitchLabel = this->findChild<QLabel *>("label_8");
    pitch = this->findChild<QSlider *>("horizontalSlider");
    connect(pitch, SIGNAL(valueChanged(int)), this, SLOT(updatePitch(int)));

    feedFPSBox = this->findChild<QGroupBox *>("Mode_3");
    feedSlider = this->findChild<QSlider *>("horizontalSlider_2");
    connect(feedSlider, SIGNAL(valueChanged(int)), this, SLOT(updateFeedFPS(int)));
    //selectedFPSBox = this->findChild<QGroupBox *>("groupBox_2");
    selectedSlider = this->findChild<QSlider *>("horizontalSlider_3");
    connect(selectedSlider, SIGNAL(valueChanged(int)), this, SLOT(updateSelectedFPS(int)));

    lockBox = this->findChild<QCheckBox *>("checkBox");
    connect(lockBox, SIGNAL(toggled(bool)), this, SLOT(isLocked()));

    scanButton= this->findChild<QPushButton *>("pushButton");
    connect(scanButton, SIGNAL(clicked()), this, SLOT(scan()));

    pitchBox = this->findChild<QGroupBox *>("Orientation_3");


    feedResolution = this->findChild<QComboBox *>("comboBox_9");
    connect(feedResolution, SIGNAL(currentIndexChanged(int)), this, SLOT(alterChoices(int)));

    selectedResolution = this->findChild<QComboBox *>("comboBox_8");
  //  connect(selectedResolution, SIGNAL(currentIndexChanged(int)), this, SLOT(alterChoices(int)));

    scrollingArea= this->findChild<QWidget *>("scrollAreaWidgetContents");

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
    scrollLayout = scrollingArea->layout();
//    int headControlIndex = scrollLayout.indexOf(pitchBox);
    //handBox = new QGroupBox("Hand Controls", pitchBox);
    handBox = new QGroupBox("Hand Control", scrollingArea);
   // scrollingArea->setWidget(handBox);

    width = pitchBox->size().width();
    height = pitchBox->size().height();


    //handBox->move(x,y);
    handBox->setSizePolicy(QSizePolicy::Maximum,QSizePolicy::Maximum);
    handBox->setMinimumSize(width,height);
    handBox->setMaximumSize(width,height);
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

    displayBox= this->findChild<QGroupBox *>("groupBox_3");
    cameraBox= this->findChild<QGroupBox *>("Selection_3");
    imageBox= this->findChild<QGroupBox *>("groupBox");
    areaFeedBox = this->findChild<QGroupBox *>("groupBox_2");

    displayChildren = displayBox->children();
    imageChildren = imageBox->children();
    resolutionChildren = areaFeedBox->children();
    cameraChildren = cameraBox->children();
    headChildren = pitchBox->children();
    rateChildren = feedFPSBox->children();

    connect(cameraBox, SIGNAL(toggled(bool)), this, SLOT(disableCameraPanel(bool)));
    connect(areaFeedBox, SIGNAL(toggled(bool)), this, SLOT(disableAreaFeedPanel(bool)));
    connect(feedFPSBox, SIGNAL(toggled(bool)), this, SLOT(disableFeedPanel(bool)));
    connect(displayBox, SIGNAL(toggled(bool)), this, SLOT(disableDisplayPanel(bool)));
    connect(pitchBox, SIGNAL(toggled(bool)), this, SLOT(disableHeadPanel(bool)));
    connect(imageBox, SIGNAL(toggled(bool)), this, SLOT(disableImagePanel(bool)));
}

CameraViewerCustomWidget::~CameraViewerCustomWidget()
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
void CameraViewerCustomWidget::alterDisplay(int num)
{

    if(num== 0 || num == 1)
    {
        if(handBox != NULL)
        {
            scrollLayout->removeWidget(handBox);
            scrollLayout->removeWidget(scanButton);
            handBox->hide();

        }
        scrollLayout->addWidget(pitchBox);
        scrollLayout->addWidget(scanButton);
        pitchBox->show();

    }
    else
    {
       // pitchBox->hide();
        scrollLayout->removeWidget(pitchBox);
        scrollLayout->removeWidget(scanButton);
        pitchBox->hide();


     //   handBox->show();
        scrollLayout->addWidget(handBox);
        scrollLayout->addWidget(scanButton);
        handBox->show();


    }
}

/**
  * Thid method takes in the selected pitch on the pitch slider
  * and updates the label so that the value is clearly
  * shown.
  **/
void CameraViewerCustomWidget::updatePitch(int value)
{
    if(value != 0 && value > -10 && value < 10)
        ui->horizontalSlider->setValue(0);
    else
    {
        std::stringstream ss;//create a stringstream for int to string conversion
        ss << value;
        std::string string = ss.str();
        QString label = QString::fromStdString(string);
        pitchLabel->setText(label);

        ui->widget->setCameraPitch(value);
    }
}

/**
  * Thid method takes in the selected fps on the feed slider
  * and updates the title so that the value is clearly
  * shown.
  **/
void CameraViewerCustomWidget::updateFeedFPS(int fps)
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
    ui->label_9->setText(label);
    //feedFPSBox->setTitle(label);

}

/**
  * Thid method takes in the selected fps on the selected area
  * slider and updates the title so that the value is clearly
  * shown.
  **/
void CameraViewerCustomWidget::updateSelectedFPS(int fps)
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
    ui->label_10->setText(label);
    //selectedFPSBox->setTitle(label);
}

void CameraViewerCustomWidget::setFeedToSingleImage()
{
    ui->horizontalSlider_2->setValue(0);
    updateFeedFPS(0);
}

void CameraViewerCustomWidget::setAreaToSingleImage()
{
    ui->horizontalSlider_3->setValue(0);
    updateSelectedFPS(0);
}

/**
  * This method makes it so that the labeling
  * on the scan button changes. Later on, it
  * should toggle the scan behavior.
  **/
void CameraViewerCustomWidget::scan()
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
void CameraViewerCustomWidget::isLocked()
{

    if(lockBox->isChecked())
    {
        sliderValues(feedSlider->value());
    }

}

void CameraViewerCustomWidget::alterChoices(int index)
{
    if(selectedResolution->currentIndex()>index)
    {
        selectedResolution->setCurrentIndex(index);
    }
}

void CameraViewerCustomWidget::disableImagePanel(bool selected)
{
    if(!imageBox->isChecked())
    {
        for(int x = 0; x<imageChildren.count(); x++)
        {
            QWidget * widget = (QWidget *)imageChildren.at(x);
            widget->hide();
        }
    }
    else
    {
        for(int x = 0; x<imageChildren.count(); x++)
        {
            QWidget * widget = (QWidget *)imageChildren.at(x);
            widget->show();
        }
    }
}

void CameraViewerCustomWidget::disableDisplayPanel(bool selected)
{
    if(!displayBox->isChecked())
    {
        for(int x = 0; x<displayChildren.count(); x++)
        {
            QWidget * widget = (QWidget *)displayChildren.at(x);
            widget->hide();
        }
    }
    else
    {
        for(int x = 0; x<displayChildren.count(); x++)
        {
            QWidget * widget = (QWidget *)displayChildren.at(x);
            widget->show();
        }
    }
}
void CameraViewerCustomWidget::disableCameraPanel(bool selected)
{
    if(!cameraBox->isChecked())
    {
        for(int x = 0; x<cameraChildren.count(); x++)
        {
            QWidget * widget = (QWidget *)cameraChildren.at(x);
            widget->hide();
        }
    }
    else
    {
        for(int x = 0; x<cameraChildren.count(); x++)
        {
            QWidget * widget = (QWidget *)cameraChildren.at(x);
            widget->show();
        }
    }

}
void CameraViewerCustomWidget::disableHeadPanel(bool selected)
{
    if(!pitchBox->isChecked())
    {
        pitchBox->setMinimumSize(0,0);
        for(int x = 0; x<headChildren.count(); x++)
        {
            QWidget * widget = (QWidget *)headChildren.at(x);
            widget->hide();
        }
    }
    else
    {
        pitchBox->setMinimumSize(0,161);
        for(int x = 0; x<headChildren.count(); x++)
        {
            QWidget * widget = (QWidget *)headChildren.at(x);
            widget->show();
        }
    }
}
void CameraViewerCustomWidget::disableAreaFeedPanel(bool selected)
{
    if(!areaFeedBox->isChecked())
    {
        for(int x = 0; x<resolutionChildren.count(); x++)
        {
            QWidget * widget = (QWidget *)resolutionChildren.at(x);
            widget->hide();
        }
    }
    else
    {
        for(int x = 0; x<resolutionChildren.count(); x++)
        {
            QWidget * widget = (QWidget *)resolutionChildren.at(x);
            widget->show();
        }
    }

}

void CameraViewerCustomWidget::disableFeedPanel(bool selected)
{
    if(!feedFPSBox->isChecked())
    {
        feedFPSBox->setMinimumSize(0,0);

        for(int x = 0; x<rateChildren.count(); x++)
        {
            QWidget * widget = (QWidget *)rateChildren.at(x);
            widget->hide();
        }
    }
    else
    {

        feedFPSBox->setMinimumSize(0,161);

        for(int x = 0; x<rateChildren.count(); x++)
        {
            QWidget * widget = (QWidget *)rateChildren.at(x);
            widget->show();
        }
    }

}



/**
void CameraViewerCustomWidget::mouseMoveEvent(QMouseEvent *event)
{
    QPoint point = event->pos();
    std::cout<<"Prints when mouse is moved"<<std::endl;
    int x = this->size().width();
    int y = this->size().height();
    std::cout<<"width is "<<x<<std::endl;
    std::cout<<"height is "<<y<<std::endl;

/**
    if(((point.x()<selectedArea[0] && point.x()>selectedArea[2]) ||
       (point.x()>selectedArea[0] && point.x()<selectedArea[2])) &&
       ((point.y()<selectedArea[1] && point.y()>selectedArea[3]) ||
       (point.y()>selectedArea[1] && point.y()<selectedArea[3])))
    {
    //    xButton->show();
    }8
    else
    {
    //    xButton->hide();
    }

}**/
