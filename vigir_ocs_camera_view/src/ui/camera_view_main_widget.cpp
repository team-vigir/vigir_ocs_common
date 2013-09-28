#include "camera_view_main_widget.h"
#include "ui_camera_view_main_widget.h"
#include "ui/template_loader_widget.h"
#include "camera_view_widget.h"
#include "base_3d_view.h"
#include <ros/package.h>

CameraViewMainWidget::CameraViewMainWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::CameraViewMainWidget)
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

    /*ui->octomap_2->hide();
    ui->lidar_point_cloud_2->hide();
    ui->stereo_point_cloud_2->hide();
    ui->laser_scan_2->hide();*/

    QHBoxLayout* aux_layout;

    // setup default views
    views_list["Top Left"] = new CameraViewWidget();
    aux_layout = new QHBoxLayout();
    aux_layout->setMargin(0);
    aux_layout->addWidget(views_list["Top Left"]);
    ui->center_parent_->setLayout(aux_layout);

    views_list["Top Right"] = new CameraViewWidget();
    aux_layout = new QHBoxLayout();
    aux_layout->setMargin(0);
    aux_layout->addWidget(views_list["Top Right"]);
    ui->top_right_parent_->setLayout(aux_layout);

    views_list["Bottom Left"] = new CameraViewWidget();
    aux_layout = new QHBoxLayout();
    aux_layout->setMargin(0);
    aux_layout->addWidget(views_list["Bottom Left"]);
    ui->bottom_left_parent_->setLayout(aux_layout);

    views_list["Bottom Right"] = new CameraViewWidget();
    aux_layout = new QHBoxLayout();
    aux_layout->setMargin(0);
    aux_layout->addWidget(views_list["Bottom Right"]);
    ui->bottom_right_parent_->setLayout(aux_layout);

    std::map<std::string, QWidget*>::iterator iter;

    for (iter = views_list.begin(); iter != views_list.end(); ++iter)
    {
        //((vigir_ocs::Base3DView*)iter->second)->simulationRobotToggled(true);
        //((vigir_ocs::Base3DView*)iter->second)->simulationRobotToggled(false);

        // connect UI to perspective functions
        QObject::connect(ui->camera_tool, SIGNAL(toggled(bool)), iter->second, SLOT(cameraToggled(bool)));
        QObject::connect(ui->footstep_planning, SIGNAL(toggled(bool)), iter->second, SLOT(footstepPlanningToggled(bool)));
        QObject::connect(ui->footstep_pose, SIGNAL(pressed()), iter->second, SLOT(vectorPressed()));
        QObject::connect(ui->grasp_model, SIGNAL(toggled(bool)), iter->second, SLOT(graspModelToggled(bool)));
        QObject::connect(ui->grid_map, SIGNAL(toggled(bool)), iter->second, SLOT(gridMapToggled(bool)));
        QObject::connect(ui->insert_waypoint, SIGNAL(pressed()), iter->second, SLOT(insertWaypoint()));
        QObject::connect(ui->laser_scan_2, SIGNAL(toggled(bool)), iter->second, SLOT(laserScanToggled(bool)));
        QObject::connect(ui->lidar_point_cloud_2, SIGNAL(toggled(bool)), iter->second, SLOT(lidarPointCloudToggled(bool)));
        QObject::connect(ui->octomap_2, SIGNAL(toggled(bool)), iter->second, SLOT(markerArrayToggled(bool)));
        QObject::connect(ui->point_cloud_request, SIGNAL(toggled(bool)), iter->second, SLOT(requestedPointCloudToggled(bool)));
        QObject::connect(ui->request_point_cloud_, SIGNAL(clicked()), iter->second, SLOT(publishPointCloudWorldRequest()));
        QObject::connect(ui->reset_map, SIGNAL(clicked()), iter->second, SLOT(clearMapRequests()));
        QObject::connect(ui->reset_point_cloud, SIGNAL(clicked()), iter->second, SLOT(clearPointCloudRequests()));
        QObject::connect(ui->robot_model_2, SIGNAL(toggled(bool)), iter->second, SLOT(robotModelToggled(bool)));
        QObject::connect(ui->simulation_robot, SIGNAL(toggled(bool)), iter->second, SLOT(simulationRobotToggled(bool)));
        QObject::connect(ui->stereo_point_cloud_2, SIGNAL(toggled(bool)), iter->second, SLOT(stereoPointCloudToggled(bool)));
        QObject::connect(ui->template_tool, SIGNAL(toggled(bool)), iter->second, SLOT(markerTemplateToggled(bool)));
        QObject::connect(ui->template_widget, SIGNAL(insertTemplate(QString)), iter->second, SLOT(insertTemplate(QString)));
        QObject::connect(ui->template_widget, SIGNAL(templatePathChanged(QString)), iter->second, SLOT(templatePathChanged(QString)));
        QObject::connect(ui->templates, SIGNAL(toggled(bool)), iter->second, SLOT(templatesToggled(bool)));
        QObject::connect(ui->widget_tool, SIGNAL(toggled(bool)), iter->second, SLOT(markerRobotToggled(bool)));
    }

    std::string ip = ros::package::getPath("vigir_ocs_camera_view")+"/icons/";
    icon_path_ = QString(ip.c_str());

    position_widget_ = new QWidget(this);
    position_widget_->setStyleSheet("background-color: rgb(108, 108, 108);color: rgb(108, 108, 108);border-color: rgb(0, 0, 0);");
    position_widget_->setMaximumSize(46,22);

    one_view_button_ = new QPushButton("",this);//𐌎", this);
    QPixmap pixmap1(icon_path_+"one_view_checked.png");
    QIcon ButtonIcon1(pixmap1);
    one_view_button_->setIcon(ButtonIcon1);
    one_view_button_->setIconSize(pixmap1.rect().size());
    one_view_button_->setCheckable(true);
    one_view_button_->setChecked(true);
    //one_view_button_->setStyleSheet("color: rgb(108, 108, 108);");
    one_view_button_->setMaximumSize(20,20);
    one_view_button_->adjustSize();
    one_view_button_->setFlat(true);
    one_view_button_->setStyleSheet("QPushButton:checked{color: rgb(180, 68, 68);border-style: inset;padding: 0px;border:0px;position: relative;top: 0px; left: 0px;}"
                                    "QPushButton:unchecked{color: rgb(180, 68, 68);border-style: inset;padding: 0px;border:0px;position: relative;top: 0px; left: 0px;}");
    QObject::connect(one_view_button_, SIGNAL(clicked()), this, SLOT(oneViewToggle()));

    four_view_button_ = new QPushButton("",this);//𐌎", this);
    QPixmap pixmap2(icon_path_+"four_view.png");
    QIcon ButtonIcon2(pixmap2);
    four_view_button_->setIcon(ButtonIcon2);
    four_view_button_->setIconSize(pixmap2.rect().size());
    four_view_button_->setCheckable(true);
    //four_view_button_->setStyleSheet("color: rgb(108, 108, 108);");
    four_view_button_->setMaximumSize(20,20);
    four_view_button_->adjustSize();
    four_view_button_->setFlat(true);
    four_view_button_->setStyleSheet("QPushButton:checked{color: rgb(180, 68, 68);border-style: inset;padding: 0px;border:1px;position: relative;top: 0px; left: 0px;}"
                                     "QPushButton:unchecked{color: rgb(180, 68, 68);border-style: inset;padding: 0px;border:1px;position: relative;top: 0px; left: 0px;}");
    QObject::connect(four_view_button_, SIGNAL(clicked()), this, SLOT(fourViewToggle()));

    QHBoxLayout* position_layout = new QHBoxLayout();
    //position_layout->setMaximumSize(18,18);
    position_layout->setMargin(2);
    position_layout->setSpacing(6);
    position_layout->addWidget(one_view_button_);
    position_layout->addWidget(four_view_button_);
    position_widget_->setLayout(position_layout);
    
    connect(ui->pitch, SIGNAL(valueChanged(int)), this, SLOT(updatePitch(int)));
}

CameraViewMainWidget::~CameraViewMainWidget()
{
    delete ui;
}

void CameraViewMainWidget::updatePitch(int value)
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

        ((CameraViewWidget*)views_list["Top Left"])->updatePitch(value);
    }
}

bool CameraViewMainWidget::eventFilter( QObject * o, QEvent * e )
{
    if ( e->type() == QEvent::Wheel &&
         (qobject_cast<QAbstractSpinBox*>( o ) || qobject_cast<QAbstractSlider*>( o ) || qobject_cast<QComboBox*>( o )))
    {
        e->ignore();
        return true;
    }
    return QWidget::eventFilter( o, e );
}

void CameraViewMainWidget::oneViewToggle()
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
}

void CameraViewMainWidget::fourViewToggle()
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
}


