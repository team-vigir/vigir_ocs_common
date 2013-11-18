#include "camera_view_widget.h"
#include "ui_camera_view_widget.h"
#include "stdio.h"

#include <iostream>
#include <sstream>  // Required for stringstreams
#include <string>
#include <QPainter>
#include <QObjectList>
#include <QGridLayout>

CameraViewWidget::CameraViewWidget(QWidget *parent, rviz::VisualizationManager* context) :
    QWidget(parent),
    ui(new Ui::CameraViewWidget)
{
    ui->setupUi(this);

    camera_view_ = new vigir_ocs::CameraView(parent, context);

    QHBoxLayout* position_layout = new QHBoxLayout();
    position_layout->setMargin(0);
    position_layout->addWidget(camera_view_);
    ui->camera_view_->setLayout(position_layout);

    camera_view_->init();

    this->setMouseTracking(true);

    connect(ui->feed_slider, SIGNAL(valueChanged(int)), this, SLOT(updateFeedFPS(int)));
    connect(ui->selected_slider, SIGNAL(valueChanged(int)), this, SLOT(updateSelectedFPS(int)));
    connect(ui->lock_box, SIGNAL(toggled(bool)), this, SLOT(isLocked()));

    //connect(ui->start_scanning, SIGNAL(clicked()), this, SLOT(scan()));

    connect(ui->feed_resolution, SIGNAL(currentIndexChanged(int)), this, SLOT(alterChoices(int)));

    Q_FOREACH( QDoubleSpinBox * sp, findChildren<QDoubleSpinBox*>() ) {
        sp->installEventFilter( this );
        sp->setFocusPolicy( Qt::StrongFocus );
    }
    Q_FOREACH( QSpinBox * sp, findChildren<QSpinBox*>() ) {
        sp->installEventFilter( this );
        sp->setFocusPolicy( Qt::StrongFocus );
    }
    Q_FOREACH( QComboBox * sp, findChildren<QComboBox*>() ) {
        sp->installEventFilter( this );
        sp->setFocusPolicy( Qt::StrongFocus );
    }
    Q_FOREACH( QSlider * sp, findChildren<QSlider*>() ) {
        sp->installEventFilter( this );
        sp->setFocusPolicy( Qt::StrongFocus );
    }

    // manually connecting UI signals to camera slots
    connect(ui->camera,              SIGNAL(currentIndexChanged(int)), camera_view_, SLOT(changeCameraTopic(int)));
    connect(ui->feed_resolution,     SIGNAL(currentIndexChanged(int)), camera_view_, SLOT(changeFullImageResolution(int)));
    connect(ui->selected_resolution, SIGNAL(currentIndexChanged(int)), camera_view_, SLOT(changeCropImageResolution(int)));
    connect(ui->feed_slider,         SIGNAL(valueChanged(int)),        camera_view_, SLOT(changeCameraSpeed(int)));
    connect(ui->selected_slider,     SIGNAL(valueChanged(int)),        camera_view_, SLOT(changeCropCameraSpeed(int)));
    connect(ui->transparency_slider, SIGNAL(valueChanged(int)),        camera_view_, SLOT(changeAlpha(int)));
    connect(ui->applyAreaFeed,       SIGNAL(clicked()),                camera_view_, SLOT(applyAreaChanges()));
    connect(ui->applyCameraFeed,     SIGNAL(clicked()),                camera_view_, SLOT(applyFeedChanges()));
    connect(ui->getCameraImage,      SIGNAL(clicked()),                camera_view_, SLOT(requestSingleFeedImage()));
    connect(ui->getAreaImage,        SIGNAL(clicked()),                camera_view_, SLOT(requestSingleAreaImage()));
    connect(ui->getCameraImage,      SIGNAL(clicked()),                this,         SLOT(setFeedToSingleImage()));
    connect(ui->getAreaImage,        SIGNAL(clicked()),                this,         SLOT(setAreaToSingleImage()));

    key_event_sub_ = n_.subscribe<flor_ocs_msgs::OCSKeyEvent>( "/flor/ocs/key_event", 5, &CameraViewWidget::processNewKeyEvent, this );
}

CameraViewWidget::~CameraViewWidget()
{
    delete ui;
}

/**
  * This method makes it so that both FPS sliders
  * maintain the same value when locked.
  */
void CameraViewWidget::sliderValues(int lockedValue)
{
    ui->feed_slider->setValue(lockedValue);
    ui->selected_slider->setValue(lockedValue);
}

/**
  * Thid method takes in the selected pitch on the pitch slider
  * and updates the label so that the value is clearly
  * shown.
  **/
void CameraViewWidget::updatePitch(int value)
{
    camera_view_->setCameraPitch(value);
}

/**
  * Thid method takes in the selected fps on the feed slider
  * and updates the title so that the value is clearly
  * shown.
  **/
void CameraViewWidget::updateFeedFPS(int fps)
{
    std::string newTitle = "Feed Rate: ";

    std::stringstream ss;//create a stringstream for int to string conversion
    ss << fps;
    std::string string = ss.str();
    newTitle.append(string);
    newTitle.append(" FPS");
    QString label = QString::fromStdString(newTitle);
    if(ui->lock_box->isChecked())
    {
        sliderValues(fps);
    }
    ui->label_9->setText(label);
    //ui->feed_fps_box->setTitle(label);

}

/**
  * Thid method takes in the selected fps on the selected area
  * slider and updates the title so that the value is clearly
  * shown.
  **/
void CameraViewWidget::updateSelectedFPS(int fps)
{

    std::string newTitle = "Selected Area Rate: ";
    std::stringstream ss;//create a stringstream for int to string conversion
    ss << fps;
    std::string string = ss.str();
    newTitle.append(string);
    newTitle.append(" FPS");
    QString label = QString::fromStdString(newTitle);
    if(ui->lock_box->isChecked())
    {
        sliderValues(fps);
    }
    ui->label_10->setText(label);
    //selectedFPSBox->setTitle(label);
}

void CameraViewWidget::setFeedToSingleImage()
{
    ui->feed_slider->setValue(0);
    updateFeedFPS(0);
}

void CameraViewWidget::setAreaToSingleImage()
{
    ui->selected_slider->setValue(0);
    updateSelectedFPS(0);
}

/**
  * This method makes it so that the labeling
  * on the scan button changes. Later on, it
  * should toggle the scan behavior.
  **/
void CameraViewWidget::scan()
{
    /*if(ui->start_scanning->text() == "Start Scanning")
    {
        //execute the scanning action
        ui->start_scanning->setText("Stop Scanning");
    }
    else
    {
        //stop the scanning action
        ui->start_scanning->setText("Start Scanning");
    }*/
}

/**
  * This method is just used to make it so that as
  * soon as "Locked" is checked, the values are made
  * to be the same.
  **/
void CameraViewWidget::isLocked()
{

    if(ui->lock_box->isChecked())
    {
        sliderValues(ui->feed_slider->value());
    }

}

void CameraViewWidget::alterChoices(int index)
{
    if(ui->selected_resolution->currentIndex()>index)
    {
        ui->selected_resolution->setCurrentIndex(index);
    }
}

void CameraViewWidget::enableGroup(bool selected)
{
    QGroupBox *group_box = (QGroupBox*)QObject::sender();
    QObjectList children = group_box->children();
    if(!selected)
    {
        for(int x = 0; x<children.count(); x++)
        {
            QWidget * widget = (QWidget *)children.at(x);
            widget->hide();
        }
    }
    else
    {
        for(int x = 0; x<children.count(); x++)
        {
            QWidget * widget = (QWidget *)children.at(x);
            widget->show();
        }
    }

    /*ui->octomap->hide();
    ui->lidar_point_cloud->hide();
    ui->stereo_point_cloud->hide();
    ui->laser_scan->hide();*/
}

bool CameraViewWidget::eventFilter( QObject * o, QEvent * e )
{
    if ( e->type() == QEvent::Wheel &&
         (qobject_cast<QAbstractSpinBox*>( o ) || qobject_cast<QAbstractSlider*>( o ) || qobject_cast<QComboBox*>( o )))
    {
        e->ignore();
        return true;
    }
    return QWidget::eventFilter( o, e );
}

void CameraViewWidget::processNewKeyEvent(const flor_ocs_msgs::OCSKeyEvent::ConstPtr &key_event)
{
    // store key state
    if(key_event->state)
        keys_pressed_list_.push_back(key_event->key);
    else
        keys_pressed_list_.erase(std::remove(keys_pressed_list_.begin(), keys_pressed_list_.end(), key_event->key), keys_pressed_list_.end());

    // process hotkeys
    std::vector<int>::iterator key_is_pressed;

    key_is_pressed = std::find(keys_pressed_list_.begin(), keys_pressed_list_.end(), 37);
    if(key_event->key == 41 && key_event->state && key_is_pressed != keys_pressed_list_.end()) // ctrl+f
        camera_view_->requestSingleFeedImage();
    if(key_event->key == 54 && key_event->state && key_is_pressed != keys_pressed_list_.end()) // ctrl+c
        camera_view_->requestSingleAreaImage();

}
