#include "camera_viewer_custom_widget.h"
#include "ui_camera_viewer_custom_widget.h"
#include "stdio.h"

#include <iostream>
#include <sstream>  // Required for stringstreams
#include <string>
#include <QPainter>
#include <QObjectList>
#include <QGridLayout>

CameraViewerCustomWidget::CameraViewerCustomWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::CameraViewerCustomWidget)
{
    ui->setupUi(this);
    this->setMouseTracking(true);

    connect(ui->pitch, SIGNAL(valueChanged(int)), this, SLOT(updatePitch(int)));

    connect(ui->feed_slider, SIGNAL(valueChanged(int)), this, SLOT(updateFeedFPS(int)));
    connect(ui->selected_slider, SIGNAL(valueChanged(int)), this, SLOT(updateSelectedFPS(int)));
    connect(ui->lock_box, SIGNAL(toggled(bool)), this, SLOT(isLocked()));

    connect(ui->start_scanning, SIGNAL(clicked()), this, SLOT(scan()));

    connect(ui->feed_resolution, SIGNAL(currentIndexChanged(int)), this, SLOT(alterChoices(int)));

    connect(ui->camera_box, SIGNAL(toggled(bool)), this, SLOT(enableGroup(bool)));
    connect(ui->area_feed_box, SIGNAL(toggled(bool)), this, SLOT(enableGroup(bool)));
    connect(ui->feed_fps_box, SIGNAL(toggled(bool)), this, SLOT(enableGroup(bool)));
    connect(ui->display_box, SIGNAL(toggled(bool)), this, SLOT(enableGroup(bool)));
    connect(ui->pitch_box, SIGNAL(toggled(bool)), this, SLOT(enableGroup(bool)));
    connect(ui->image_box, SIGNAL(toggled(bool)), this, SLOT(enableGroup(bool)));

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

    ui->octomap->hide();
    ui->lidar_point_cloud->hide();
    ui->stereo_point_cloud->hide();
    ui->laser_scan->hide();
}

CameraViewerCustomWidget::~CameraViewerCustomWidget()
{
    delete ui;
}

/**
  * This method makes it so that both FPS sliders
  * maintain the same value when locked.
  */
void CameraViewerCustomWidget::sliderValues(int lockedValue)
{
    ui->feed_slider->setValue(lockedValue);
    ui->selected_slider->setValue(lockedValue);
}

/**
  * Thid method takes in the selected pitch on the pitch slider
  * and updates the label so that the value is clearly
  * shown.
  **/
void CameraViewerCustomWidget::updatePitch(int value)
{
    if(value != 0 && value > -10 && value < 10)
        ui->pitch->setValue(0);
    else
    {
        std::stringstream ss;//create a stringstream for int to string conversion
        ss << value;
        std::string string = ss.str();
        QString label = QString::fromStdString(string);
        ui->pitch_label->setText(label);

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
void CameraViewerCustomWidget::updateSelectedFPS(int fps)
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

void CameraViewerCustomWidget::setFeedToSingleImage()
{
    ui->feed_slider->setValue(0);
    updateFeedFPS(0);
}

void CameraViewerCustomWidget::setAreaToSingleImage()
{
    ui->selected_slider->setValue(0);
    updateSelectedFPS(0);
}

/**
  * This method makes it so that the labeling
  * on the scan button changes. Later on, it
  * should toggle the scan behavior.
  **/
void CameraViewerCustomWidget::scan()
{
    if(ui->start_scanning->text() == "Start Scanning")
    {
        //execute the scanning action
        ui->start_scanning->setText("Stop Scanning");
    }
    else
    {
        //stop the scanning action
        ui->start_scanning->setText("Start Scanning");
    }
}

/**
  * This method is just used to make it so that as
  * soon as "Locked" is checked, the values are made
  * to be the same.
  **/
void CameraViewerCustomWidget::isLocked()
{

    if(ui->lock_box->isChecked())
    {
        sliderValues(ui->feed_slider->value());
    }

}

void CameraViewerCustomWidget::alterChoices(int index)
{
    if(ui->selected_resolution->currentIndex()>index)
    {
        ui->selected_resolution->setCurrentIndex(index);
    }
}

void CameraViewerCustomWidget::enableGroup(bool selected)
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

    ui->octomap->hide();
    ui->lidar_point_cloud->hide();
    ui->stereo_point_cloud->hide();
    ui->laser_scan->hide();
}

bool CameraViewerCustomWidget::eventFilter( QObject * o, QEvent * e )
{
    if ( e->type() == QEvent::Wheel &&
         (qobject_cast<QAbstractSpinBox*>( o ) || qobject_cast<QAbstractSlider*>( o ) || qobject_cast<QComboBox*>( o )))
    {
        e->ignore();
        return true;
    }
    return QWidget::eventFilter( o, e );
}
