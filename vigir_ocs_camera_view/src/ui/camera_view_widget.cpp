#include "camera_view_widget.h"
#include "ui_camera_view_widget.h"
#include "stdio.h"
#include <ros/package.h>

#include <iostream>
#include <sstream>  // Required for stringstreams
#include <string>
#include <QPainter>
#include <QObjectList>
#include <QGridLayout>
#include <QWidgetAction>
#include <QToolTip>

CameraViewWidget::CameraViewWidget(QWidget *parent, vigir_ocs::Base3DView* copy_from) :
    QWidget(parent),
    ui(new Ui::CameraViewWidget)
{
    ui->setupUi(this);

    camera_view_ = new vigir_ocs::CameraView(parent, copy_from);

    // update the UI with the camera names
    std::vector<std::string> camera_names = camera_view_->getCameraNames();
    for(int i = 0; i < camera_names.size(); i++)
        ui->camera->addItem(camera_names[i].c_str());
    ui->camera->setCurrentIndex(camera_view_->getDefaultCamera());

    QHBoxLayout* position_layout = new QHBoxLayout();
    position_layout->setMargin(0);
    position_layout->addWidget(camera_view_);
    ui->camera_view_->setLayout(position_layout);

    this->setMouseTracking(true);

    // setting up the slider menus for all buttons
    feed_slider = new QSlider(Qt::Vertical);
    feed_slider->setRange(0, 30);
    feed_slider->setFixedHeight(200);
    feed_slider->setTickPosition(QSlider::NoTicks);

    QWidgetAction *wa_feed = new QWidgetAction(0);
    wa_feed->setDefaultWidget(feed_slider);
    image_feed_menu_.addAction(wa_feed);

    ui->get_camera_feed->setMenu(&image_feed_menu_);
    image_feed_menu_.installEventFilter(this);

    area_slider = new QSlider(Qt::Vertical);
    area_slider->setRange(0, 30);
    area_slider->setFixedHeight(200);
    area_slider->setTickPosition(QSlider::NoTicks);

    QWidgetAction *wa_area = new QWidgetAction(0);
    wa_area->setDefaultWidget(area_slider);
    area_feed_menu_.addAction(wa_area);

    ui->get_area_feed->setMenu(&area_feed_menu_);
    area_feed_menu_.installEventFilter(this);

    transparency_slider = new QSlider(Qt::Vertical);
    transparency_slider->setRange(0, 100);
    transparency_slider->setFixedHeight(200);
    transparency_slider->setTickPosition(QSlider::NoTicks);

    QWidgetAction *wa_transparency = new QWidgetAction(0);
    wa_transparency->setDefaultWidget(transparency_slider);
    image_transparency_menu_.addAction(wa_transparency);

    ui->image_transparency->setMenu(&image_transparency_menu_);
    image_transparency_menu_.installEventFilter(this);

    connect(feed_slider, SIGNAL(valueChanged(int)), this, SLOT(updateFeedFPS(int)));
    connect(area_slider, SIGNAL(valueChanged(int)), this, SLOT(updateSelectedFPS(int)));

    //////connect(ui->start_scanning, SIGNAL(clicked()), this, SLOT(scan()));

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
    connect(ui->get_camera_image,    SIGNAL(clicked()),                camera_view_, SLOT(requestSingleFeedImage()));
    connect(ui->get_area_image,      SIGNAL(clicked()),                camera_view_, SLOT(requestSingleAreaImage()));
    connect(ui->get_camera_image,    SIGNAL(clicked()),                this,         SLOT(setFeedToSingleImage()));
    connect(ui->get_area_image,      SIGNAL(clicked()),                this,         SLOT(setAreaToSingleImage()));
    connect(ui->get_camera_feed,     SIGNAL(clicked()),                this,         SLOT(imageFeedButtonClicked()));
    connect(feed_slider,             SIGNAL(sliderMoved(int)),         this,         SLOT(imageFeedSliderChanged(int)));
    connect(feed_slider,             SIGNAL(sliderReleased()),         this,         SLOT(imageFeedSliderReleased()));
    connect(ui->get_area_feed,       SIGNAL(clicked()),                this,         SLOT(areaFeedButtonClicked()));
    connect(area_slider,             SIGNAL(sliderMoved(int)),         this,         SLOT(areaFeedSliderChanged(int)));
    connect(area_slider,             SIGNAL(sliderReleased()),         this,         SLOT(areaFeedSliderReleased()));
    connect(ui->image_transparency,  SIGNAL(clicked()),                this,         SLOT(transparencyButtonClicked()));
    connect(transparency_slider,     SIGNAL(sliderMoved(int)),         this,         SLOT(transparencySliderChanged(int)));
    connect(area_slider,             SIGNAL(sliderReleased()),         this,         SLOT(transparencySliderReleased()));

    // setup icons path
    std::string ip = ros::package::getPath("vigir_ocs_camera_view")+"/icons/";
    icon_path_ = QString(ip.c_str());

    // setup icons
//    QPixmap pixmap1(icon_path_+"get_image_step.png");
//    QIcon icon1(pixmap1);
//    ui->get_camera_image->setIcon(icon1);

    loadButtonIconAndStyle(ui->get_camera_image,"get_image_step.png");

    ui->get_camera_image->setStyleSheet(QString("QPushButton  { ") +
                                      " background-color: qlineargradient(spread:pad, x1:0, y1:0, x2:0, y2:1, stop:0 rgba(240, 240, 240, 255), stop:1 rgba(222, 222, 222, 255));" +
                                      " border-style: solid;" +
                                      " border-width: 1px;" +
                                      " border-radius: 1px;" +
                                      " border-color: gray;" +
                                      " padding: 0px;" +
                                      " image: url(" + icon_path_ + "get_image_step.png" + ");" +
                                      "}" +
                                      "QPushButton:pressed  {" +
                                      " padding-top:1px; padding-left:1px;" +
                                      " background-color: rgb(180,180,180);" +
                                      " border-style: inset;" +
                                      " image: url(" + icon_path_ + "get_image_step.png" + ");" +
                                      "}");
//    QPixmap pixmap2(icon_path_+"get_image_play.png");
//    QIcon icon2(pixmap2);
//    ui->get_camera_feed->setIcon(icon2);

    loadButtonIconAndStyle(ui->get_camera_feed,"get_image_play.png");

//    QPixmap pixmap3(icon_path_+"mg_step.png");
//    QIcon icon3(pixmap3);
//    ui->get_area_image->setIcon(icon3);

    loadButtonIconAndStyle(ui->get_area_image,"mg_step.png");


//    QPixmap pixmap4(icon_path_+"mg_play.png");
//    QIcon icon4(pixmap4);
//    ui->get_area_feed->setIcon(icon4);

    loadButtonIconAndStyle(ui->get_area_feed,"mg_play.png");

//    QPixmap pixmap5(icon_path_+"image_transparency.png");
//    QIcon icon5(pixmap5);
//    ui->image_transparency->setIcon(icon5);

    loadButtonIconAndStyle(ui->image_transparency,"image_transparency.png");

    // workaround to be able to use images from stylesheet without knowing the path in advance
    QString stylesheet = ui->camera->styleSheet() + "\n" +
            "QComboBox::down-arrow {\n" +
            " image: url(" + icon_path_ + "down_arrow.png" + ");\n" +
            "}";
    ui->camera->setStyleSheet(stylesheet);
    ui->feed_resolution->setStyleSheet(stylesheet);
    ui->selected_resolution->setStyleSheet(stylesheet);

    key_event_sub_ = n_.subscribe<flor_ocs_msgs::OCSKeyEvent>( "/flor/ocs/key_event", 5, &CameraViewWidget::processNewKeyEvent, this );

}

CameraViewWidget::~CameraViewWidget()
{
    delete ui;
}

void CameraViewWidget::loadButtonIconAndStyle(QPushButton* btn, QString image_name)
{
    //used for all buttons that need to be oriented to top left
    btn->setStyleSheet(QString("QPushButton  { ") +
                                      " background-color: qlineargradient(spread:pad, x1:0, y1:0, x2:0, y2:1, stop:0 rgba(240, 240, 240, 255), stop:1 rgba(222, 222, 222, 255));" +
                                      " border-style: solid;" +
                                      " border-width: 1px;" +
                                      " border-radius: 1px;" +
                                      " border-color: gray;" +
                                      " padding: 0px;" +
                                      " image: url(" + icon_path_ + image_name + ");" +
                                      " image-position: top left"
                                      "}" +
                                      "QPushButton:pressed  {" +
                                      " padding-top:1px; padding-left:1px;" +
                                      " background-color: rgb(180,180,180);" +
                                      " border-style: inset;" +
                                      " image: url(" + icon_path_ + image_name + ");" +
                                      "}");
}

/**
  * This method makes it so that both FPS sliders
  * maintain the same value when locked.
  */
void CameraViewWidget::sliderValues(int lockedValue)
{
    //ui->feed_slider->setValue(lockedValue);
    //ui->area_slider->setValue(lockedValue);
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
  * Thid method sets the current pitch value used for creating a trajectory
  **/
void CameraViewWidget::updateCurrentPitch(int value)
{
    camera_view_->setCurrentCameraPitch(value);
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

    feed_slider->setValue(fps);
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
}

void CameraViewWidget::setFeedToSingleImage()
{
    feed_slider->setValue(0);
    updateFeedFPS(0);
}

void CameraViewWidget::setAreaToSingleImage()
{
    area_slider->setValue(0);
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

void CameraViewWidget::alterChoices(int index)
{
    if(ui->selected_resolution->currentIndex()>index)
    {
        ui->selected_resolution->setCurrentIndex(index);
    }
}

void CameraViewWidget::imageFeedSliderChanged(int fps)
{
    if(fps != feed_slider->value())
    {
        camera_view_->changeCameraSpeed(fps);
        feed_slider->setValue(fps);
        feed_slider->setToolTip(QString::number(fps)+"fps");
        QPoint p = QCursor::pos();
        QToolTip::showText(QPoint( p.x()+4, p.y()+4 ), QString::number(fps)+"fps");
    }
}

void CameraViewWidget::imageFeedSliderReleased()
{
    camera_view_->applyFeedChanges();
    QToolTip::hideText();
}

void CameraViewWidget::imageFeedButtonClicked()
{
    //QPoint p = QCursor::pos();
    //QAction* selected_item = image_feed_menu_.exec(p);
    ui->get_camera_feed->showMenu();
}

void CameraViewWidget::areaFeedSliderChanged(int fps)
{
    if(fps != area_slider->value())
    {
        camera_view_->changeCropCameraSpeed(fps);
        area_slider->setValue(fps);
        area_slider->setToolTip(QString::number(fps)+"fps");
        QPoint p = QCursor::pos();
        QToolTip::showText(QPoint( p.x()+4, p.y()+4 ), QString::number(fps)+"fps");
    }
}

void CameraViewWidget::areaFeedSliderReleased()
{
    camera_view_->applyAreaChanges();
    QToolTip::hideText();
}

void CameraViewWidget::areaFeedButtonClicked()
{
    //QPoint p = QCursor::pos();
    //QAction* selected_item = area_feed_menu_.exec(p);
    ui->get_area_feed->showMenu();
}

void CameraViewWidget::transparencySliderChanged(int alpha)
{
    camera_view_->changeAlpha(alpha);
    transparency_slider->setToolTip(QString::number(alpha)+"%");
    QPoint p = QCursor::pos();
    QToolTip::showText(QPoint( p.x()+4, p.y()+4 ), QString::number(alpha)+"%");
}

void CameraViewWidget::transparencySliderReleased()
{
    QToolTip::hideText();
}

void CameraViewWidget::transparencyButtonClicked()
{
    //QPoint p = QCursor::pos();
    //QAction* selected_item = image_transparency_menu_.exec(p);
    ui->image_transparency->showMenu();
}

bool CameraViewWidget::eventFilter( QObject * o, QEvent * e )
{
    if ( e->type() == QEvent::Wheel &&
         (qobject_cast<QAbstractSpinBox*>( o ) || qobject_cast<QAbstractSlider*>( o ) || qobject_cast<QComboBox*>( o )))
    {
        e->ignore();
        return true;
    }
    //move menus to respective buttons
    if ( e->type() == QEvent::Show && qobject_cast<QMenu*>( o ))
    {
        QPoint p;
        if(((QMenu*)o) == ui->get_camera_feed->menu())
        {
            p.setX(0);
            p.setY(ui->get_camera_feed->geometry().height());
            p = ui->get_camera_feed->mapToGlobal(p);
        }
        else if(((QMenu*)o) == ui->get_area_feed->menu())
        {
            p.setX(0);
            p.setY(ui->get_area_feed->geometry().height());
            p = ui->get_area_feed->mapToGlobal(p);
        }
        else if(((QMenu*)o) == ui->image_transparency->menu())
        {
            p.setX(0);
            p.setY(ui->image_transparency->geometry().height());
            p = ui->image_transparency->mapToGlobal(p);
        }
        ((QMenu*)o)->move(p);
        return true;
    }
    return QWidget::eventFilter( o, e );
}

void CameraViewWidget::processNewKeyEvent(const flor_ocs_msgs::OCSKeyEvent::ConstPtr &key_event)
{
    // store key state
    if(key_event->state)
        keys_pressed_list_.push_back(key_event->keycode);
    else
        keys_pressed_list_.erase(std::remove(keys_pressed_list_.begin(), keys_pressed_list_.end(), key_event->keycode), keys_pressed_list_.end());

    // process hotkeys
}
