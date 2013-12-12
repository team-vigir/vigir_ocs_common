#include "main_camera_view_widget.h"
#include "ui_main_camera_view_widget.h"
#include "ui/template_loader_widget.h"
#include "camera_view_widget.h"
#include "base_3d_view.h"
#include <ros/package.h>

#include <rviz/displays_panel.h>

MainCameraViewWidget::MainCameraViewWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::MainCameraViewWidget)
{
    ui->setupUi(this);

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

    ui->insert_waypoint->hide();

    // setup default views
    views_list_["Top Left"] = new CameraViewWidget();
    views_list_["Top Right"] = new CameraViewWidget(0,((CameraViewWidget*)views_list_["Top Left"])->getCameraView());
    views_list_["Bottom Left"] = new CameraViewWidget(0,((CameraViewWidget*)views_list_["Top Left"])->getCameraView());
    views_list_["Bottom Right"] = new CameraViewWidget(0,((CameraViewWidget*)views_list_["Top Left"])->getCameraView());

    QHBoxLayout* aux_layout;

    aux_layout = new QHBoxLayout();
    aux_layout->setMargin(0);
    aux_layout->addWidget(views_list_["Top Left"]);
    ui->center_parent_->setLayout(aux_layout);

    aux_layout = new QHBoxLayout();
    aux_layout->setMargin(0);
    aux_layout->addWidget(views_list_["Top Right"]);
    ui->top_right_parent_->setLayout(aux_layout);

    aux_layout = new QHBoxLayout();
    aux_layout->setMargin(0);
    aux_layout->addWidget(views_list_["Bottom Left"]);
    ui->bottom_left_parent_->setLayout(aux_layout);

    aux_layout = new QHBoxLayout();
    aux_layout->setMargin(0);
    aux_layout->addWidget(views_list_["Bottom Right"]);
    ui->bottom_right_parent_->setLayout(aux_layout);

    std::map<std::string, QWidget*>::iterator iter;

    for (iter = views_list_.begin(); iter != views_list_.end(); ++iter)
    {
        if(iter->second == views_list_["Top Left"])
        {
            //((CameraViewWidget*)iter->second)->getCameraView()->updateRenderMask(true);

            ((CameraViewWidget*)iter->second)->getCameraView()->simulationRobotToggled(true);
            ((CameraViewWidget*)iter->second)->getCameraView()->simulationRobotToggled(false);

            // connect UI to perspective functions
            //QObject::connect(ui->camera_tool, SIGNAL(toggled(bool)), ((CameraViewWidget*)iter->second)->getCameraView(), SLOT(cameraToggled(bool)));
            QObject::connect(ui->footstep_planning, SIGNAL(toggled(bool)), ((CameraViewWidget*)iter->second)->getCameraView(), SLOT(footstepPlanningToggled(bool)));
            QObject::connect(ui->footstep_pose_walk, SIGNAL(pressed()), ((CameraViewWidget*)iter->second)->getCameraView(), SLOT(defineWalkPosePressed()));
            QObject::connect(ui->footstep_pose_step, SIGNAL(pressed()), ((CameraViewWidget*)iter->second)->getCameraView(), SLOT(defineStepPosePressed()));
            QObject::connect(ui->grasp_model, SIGNAL(toggled(bool)), ((CameraViewWidget*)iter->second)->getCameraView(), SLOT(graspModelToggled(bool)));
            QObject::connect(ui->grid_map, SIGNAL(toggled(bool)), ((CameraViewWidget*)iter->second)->getCameraView(), SLOT(gridMapToggled(bool)));
            QObject::connect(ui->insert_waypoint, SIGNAL(pressed()), ((CameraViewWidget*)iter->second)->getCameraView(), SLOT(insertWaypoint()));
            QObject::connect(ui->laser_scan_2, SIGNAL(toggled(bool)), ((CameraViewWidget*)iter->second)->getCameraView(), SLOT(laserScanToggled(bool)));
            QObject::connect(ui->lidar_point_cloud_2, SIGNAL(toggled(bool)), ((CameraViewWidget*)iter->second)->getCameraView(), SLOT(lidarPointCloudToggled(bool)));
            QObject::connect(ui->octomap_2, SIGNAL(toggled(bool)), ((CameraViewWidget*)iter->second)->getCameraView(), SLOT(markerArrayToggled(bool)));
            QObject::connect(ui->point_cloud_request, SIGNAL(toggled(bool)), ((CameraViewWidget*)iter->second)->getCameraView(), SLOT(requestedPointCloudToggled(bool)));
            QObject::connect(ui->request_point_cloud_, SIGNAL(clicked()), ((CameraViewWidget*)iter->second)->getCameraView(), SLOT(publishPointCloudWorldRequest()));
            QObject::connect(ui->reset_map, SIGNAL(clicked()), ((CameraViewWidget*)iter->second)->getCameraView(), SLOT(clearMapRequests()));
            QObject::connect(ui->reset_point_cloud, SIGNAL(clicked()), ((CameraViewWidget*)iter->second)->getCameraView(), SLOT(clearPointCloudRaycastRequests()));
            QObject::connect(ui->reset_point_cloud_lidar, SIGNAL(clicked()), ((CameraViewWidget*)iter->second)->getCameraView(), SLOT(clearPointCloudRegionRequests()));
            QObject::connect(ui->reset_point_cloud_stereo, SIGNAL(clicked()), ((CameraViewWidget*)iter->second)->getCameraView(), SLOT(clearPointCloudStereoRequests()));
            QObject::connect(ui->robot_model_2, SIGNAL(toggled(bool)), ((CameraViewWidget*)iter->second)->getCameraView(), SLOT(robotModelToggled(bool)));
            QObject::connect(ui->simulation_robot, SIGNAL(toggled(bool)), ((CameraViewWidget*)iter->second)->getCameraView(), SLOT(simulationRobotToggled(bool)));
            QObject::connect(ui->stereo_point_cloud_2, SIGNAL(toggled(bool)), ((CameraViewWidget*)iter->second)->getCameraView(), SLOT(stereoPointCloudToggled(bool)));
            QObject::connect(ui->template_tool, SIGNAL(toggled(bool)), ((CameraViewWidget*)iter->second)->getCameraView(), SLOT(markerTemplateToggled(bool)));
            QObject::connect(ui->template_widget, SIGNAL(insertTemplate(QString)), ((CameraViewWidget*)iter->second)->getCameraView(), SLOT(insertTemplate(QString)));
            QObject::connect(ui->template_widget, SIGNAL(templatePathChanged(QString)), ((CameraViewWidget*)iter->second)->getCameraView(), SLOT(templatePathChanged(QString)));
            QObject::connect(ui->templates, SIGNAL(toggled(bool)), ((CameraViewWidget*)iter->second)->getCameraView(), SLOT(templatesToggled(bool)));
            QObject::connect(ui->widget_tool, SIGNAL(toggled(bool)), ((CameraViewWidget*)iter->second)->getCameraView(), SLOT(markerRobotToggled(bool)));
        }
        else
        {
            //((CameraViewWidget*)iter->second)->getCameraView()->updateRenderMask(true);
        }
        QObject::connect(ui->camera_tool, SIGNAL(toggled(bool)), ((CameraViewWidget*)iter->second)->getCameraView(), SLOT(selectionToolToggle(bool)));
        ((CameraViewWidget*)iter->second)->getCameraView()->updateRenderMask(true);
        QObject::connect(((CameraViewWidget*)iter->second)->getCameraView(), SIGNAL(setInitialized()), this, SLOT(cameraInitialized()));

    }

    std::string ip = ros::package::getPath("vigir_ocs_main_camera_view")+"/icons/";
    icon_path_ = QString(ip.c_str());

    position_widget_ = new QWidget(this);
    position_widget_->setStyleSheet("background-color: rgb(108, 108, 108);color: rgb(108, 108, 108);border-color: rgb(0, 0, 0);");
    position_widget_->setMaximumSize(46,22);

    one_view_button_ = new QPushButton("",this);
    QPixmap pixmap1(icon_path_+"one_view_checked.png");
    QIcon ButtonIcon1(pixmap1);
    one_view_button_->setIcon(ButtonIcon1);
    one_view_button_->setIconSize(pixmap1.rect().size());
    one_view_button_->setCheckable(true);
    one_view_button_->setChecked(true);
    one_view_button_->setMaximumSize(20,20);
    one_view_button_->adjustSize();
    one_view_button_->setFlat(true);
    one_view_button_->setStyleSheet("QPushButton:checked{color: rgb(180, 68, 68);border-style: inset;padding: 0px;border:0px;position: relative;top: 0px; left: 0px;}"
                                    "QPushButton:unchecked{color: rgb(180, 68, 68);border-style: inset;padding: 0px;border:0px;position: relative;top: 0px; left: 0px;}");
    QObject::connect(one_view_button_, SIGNAL(clicked()), this, SLOT(oneViewToggle()));

    four_view_button_ = new QPushButton("",this);
    QPixmap pixmap2(icon_path_+"four_view.png");
    QIcon ButtonIcon2(pixmap2);
    four_view_button_->setIcon(ButtonIcon2);
    four_view_button_->setIconSize(pixmap2.rect().size());
    four_view_button_->setCheckable(true);
    four_view_button_->setMaximumSize(20,20);
    four_view_button_->adjustSize();
    four_view_button_->setFlat(true);
    four_view_button_->setStyleSheet("QPushButton:checked{color: rgb(180, 68, 68);border-style: inset;padding: 0px;border:1px;position: relative;top: 0px; left: 0px;}"
                                     "QPushButton:unchecked{color: rgb(180, 68, 68);border-style: inset;padding: 0px;border:1px;position: relative;top: 0px; left: 0px;}");
    QObject::connect(four_view_button_, SIGNAL(clicked()), this, SLOT(fourViewToggle()));

    QHBoxLayout* position_layout = new QHBoxLayout();
    position_layout->setMargin(2);
    position_layout->setSpacing(6);
    position_layout->addWidget(one_view_button_);
    position_layout->addWidget(four_view_button_);
    position_widget_->setLayout(position_layout);
    position_widget_->setGeometry(0,39,46,22);

    rviz::DisplaysPanel* displays_panel = new rviz::DisplaysPanel(this);
    displays_panel->initialize( ((CameraViewWidget*)views_list_["Top Left"])->getCameraView()->getVisualizationManager());

    QVBoxLayout* displays_layout = new QVBoxLayout();
    displays_layout->setMargin(0);
    displays_layout->addWidget(displays_panel);
    ui->rviz_options->setLayout(displays_layout);
    
    connect(ui->pitch, SIGNAL(valueChanged(int)), this, SLOT(updatePitch(int)));

    views_initialized_ = 0;

    fourViewToggle();

    key_event_sub_ = nh_.subscribe<flor_ocs_msgs::OCSKeyEvent>( "/flor/ocs/key_event", 5, &MainCameraViewWidget::processNewKeyEvent, this );
}

MainCameraViewWidget::~MainCameraViewWidget()
{
    delete ui;
}

void MainCameraViewWidget::updatePitch(int value)
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

        ((CameraViewWidget*)views_list_["Top Left"])->updatePitch(value);
    }
}

bool MainCameraViewWidget::eventFilter( QObject * o, QEvent * e )
{
    if ( e->type() == QEvent::Wheel &&
         (qobject_cast<QAbstractSpinBox*>( o ) || qobject_cast<QAbstractSlider*>( o ) || qobject_cast<QComboBox*>( o )))
    {
        e->ignore();
        return true;
    }
    return QWidget::eventFilter( o, e );
}

void MainCameraViewWidget::oneViewToggle()
{
    if(four_view_button_->isChecked())
        ui->center_parent_->setLayout(ui->top_left_parent_->layout());
    one_view_button_->setChecked(true);
    four_view_button_->setChecked(false);
    ui->view_stack_->setCurrentIndex(0);

    QPixmap pixmap1(icon_path_+"one_view_checked.png");
    QIcon ButtonIcon1(pixmap1);
    one_view_button_->setIcon(ButtonIcon1);
    one_view_button_->setIconSize(pixmap1.rect().size());

    QPixmap pixmap2(icon_path_+"four_view.png");
    QIcon ButtonIcon2(pixmap2);
    four_view_button_->setIcon(ButtonIcon2);
    four_view_button_->setIconSize(pixmap2.rect().size());

    std::map<std::string, QWidget*>::iterator iter;

    for (iter = views_list_.begin(); iter != views_list_.end(); ++iter)
    {
        if(iter->second == views_list_["Top Left"])
            ((CameraViewWidget*)iter->second)->getCameraView()->updateRenderMask(true);
        else
            ((CameraViewWidget*)iter->second)->getCameraView()->updateRenderMask(false);
    }
}

void MainCameraViewWidget::fourViewToggle()
{
    if(one_view_button_->isChecked())
        ui->top_left_parent_->setLayout(ui->center_parent_->layout());
    one_view_button_->setChecked(false);
    four_view_button_->setChecked(true);
    ui->view_stack_->setCurrentIndex(1);

    QPixmap pixmap1(icon_path_+"one_view.png");
    QIcon ButtonIcon1(pixmap1);
    one_view_button_->setIcon(ButtonIcon1);
    one_view_button_->setIconSize(pixmap1.rect().size());

    QPixmap pixmap2(icon_path_+"four_view_checked.png");
    QIcon ButtonIcon2(pixmap2);
    four_view_button_->setIcon(ButtonIcon2);
    four_view_button_->setIconSize(pixmap2.rect().size());

    std::map<std::string, QWidget*>::iterator iter;

    for (iter = views_list_.begin(); iter != views_list_.end(); ++iter)
        ((CameraViewWidget*)iter->second)->getCameraView()->updateRenderMask(true);
}

void MainCameraViewWidget::cameraInitialized()
{
    views_initialized_++;
    std::cout << "Cameras initialized: " << views_initialized_ << std::endl;

    if(views_initialized_ == views_list_.size()-1)
        oneViewToggle();
}

void MainCameraViewWidget::processNewKeyEvent(const flor_ocs_msgs::OCSKeyEvent::ConstPtr &key_event)
{
    // store key state
    if(key_event->state)
        keys_pressed_list_.push_back(key_event->key);
    else
        keys_pressed_list_.erase(std::remove(keys_pressed_list_.begin(), keys_pressed_list_.end(), key_event->key), keys_pressed_list_.end());

    bool ctrl_is_pressed = (std::find(keys_pressed_list_.begin(), keys_pressed_list_.end(), 37) != keys_pressed_list_.end());
    bool shift_is_pressed = (std::find(keys_pressed_list_.begin(), keys_pressed_list_.end(), 50) != keys_pressed_list_.end());
    bool alt_is_pressed = (std::find(keys_pressed_list_.begin(), keys_pressed_list_.end(), 64) != keys_pressed_list_.end());

    // process hotkeys
    if(key_event->key == 12 && key_event->state && ctrl_is_pressed) // '3' - get single image in the main view
        ((CameraViewWidget*)views_list_["Top Left"])->getCameraView()->requestSingleFeedImage();
    else if(key_event->key == 13 && key_event->state && ctrl_is_pressed) // '4' - set main view to 5 fps
    {
        ((CameraViewWidget*)views_list_["Top Left"])->imageFeedSliderChanged(5);
        ((CameraViewWidget*)views_list_["Top Left"])->imageFeedSliderReleased();
    }
}
