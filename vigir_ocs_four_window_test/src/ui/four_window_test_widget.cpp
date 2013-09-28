#include "four_window_test_widget.h"
#include "ui_four_window_test_widget.h"
#include "ui/template_loader_widget.h"

FourWindowTestWidget::FourWindowTestWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::FourWindowTestWidget)
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

    ui->octomap_2->hide();
    ui->lidar_point_cloud_2->hide();
    ui->stereo_point_cloud_2->hide();
    ui->laser_scan_2->hide();

    ui->ortho_view_top_->setViewPlane("XY");
    ui->ortho_view_right_->setViewPlane("XZ");
    ui->ortho_view_front_->setViewPlane("YZ");

    ui->perspective_view_->simulationRobotToggled(true);
    ui->ortho_view_top_->simulationRobotToggled(true);
    ui->ortho_view_right_->simulationRobotToggled(true);
    ui->ortho_view_front_->simulationRobotToggled(true);

    ui->perspective_view_->simulationRobotToggled(false);
    ui->ortho_view_top_->simulationRobotToggled(false);
    ui->ortho_view_right_->simulationRobotToggled(false);
    ui->ortho_view_front_->simulationRobotToggled(false);

    // connect UI to perspective functions
    QObject::connect(ui->camera_tool, SIGNAL(toggled(bool)), ui->perspective_view_, SLOT(cameraToggled(bool)));
    QObject::connect(ui->footstep_planning, SIGNAL(toggled(bool)), ui->perspective_view_, SLOT(footstepPlanningToggled(bool)));
    QObject::connect(ui->footstep_pose, SIGNAL(pressed()), ui->perspective_view_, SLOT(vectorPressed()));
    QObject::connect(ui->grasp_model, SIGNAL(toggled(bool)), ui->perspective_view_, SLOT(graspModelToggled(bool)));
    QObject::connect(ui->grid_map, SIGNAL(toggled(bool)), ui->perspective_view_, SLOT(gridMapToggled(bool)));
    QObject::connect(ui->insert_waypoint, SIGNAL(pressed()), ui->perspective_view_, SLOT(insertWaypoint()));
    QObject::connect(ui->laser_scan_2, SIGNAL(toggled(bool)), ui->perspective_view_, SLOT(laserScanToggled(bool)));
    QObject::connect(ui->lidar_point_cloud_2, SIGNAL(toggled(bool)), ui->perspective_view_, SLOT(lidarPointCloudToggled(bool)));
    QObject::connect(ui->octomap_2, SIGNAL(toggled(bool)), ui->perspective_view_, SLOT(markerArrayToggled(bool)));
    QObject::connect(ui->point_cloud_request, SIGNAL(toggled(bool)), ui->perspective_view_, SLOT(requestedPointCloudToggled(bool)));
    QObject::connect(ui->request_point_cloud_, SIGNAL(clicked()), ui->perspective_view_, SLOT(publishPointCloudWorldRequest()));
    QObject::connect(ui->reset_map, SIGNAL(clicked()), ui->perspective_view_, SLOT(clearMapRequests()));
    QObject::connect(ui->reset_point_cloud, SIGNAL(clicked()), ui->perspective_view_, SLOT(clearPointCloudRequests()));
    QObject::connect(ui->robot_model_2, SIGNAL(toggled(bool)), ui->perspective_view_, SLOT(robotModelToggled(bool)));
    QObject::connect(ui->simulation_robot, SIGNAL(toggled(bool)), ui->perspective_view_, SLOT(simulationRobotToggled(bool)));
    QObject::connect(ui->stereo_point_cloud_2, SIGNAL(toggled(bool)), ui->perspective_view_, SLOT(stereoPointCloudToggled(bool)));
    QObject::connect(ui->template_tool, SIGNAL(toggled(bool)), ui->perspective_view_, SLOT(markerTemplateToggled(bool)));
    QObject::connect(ui->template_widget, SIGNAL(insertTemplate(QString)), ui->perspective_view_, SLOT(insertTemplate(QString)));
    QObject::connect(ui->template_widget, SIGNAL(templatePathChanged(QString)), ui->perspective_view_, SLOT(templatePathChanged(QString)));
    QObject::connect(ui->templates, SIGNAL(toggled(bool)), ui->perspective_view_, SLOT(templatesToggled(bool)));
    QObject::connect(ui->widget_tool, SIGNAL(toggled(bool)), ui->perspective_view_, SLOT(markerRobotToggled(bool)));



    // connect UI to perspective functions
    QObject::connect(ui->camera_tool, SIGNAL(toggled(bool)), ui->ortho_view_top_, SLOT(cameraToggled(bool)));
    QObject::connect(ui->footstep_planning, SIGNAL(toggled(bool)), ui->ortho_view_top_, SLOT(footstepPlanningToggled(bool)));
    QObject::connect(ui->footstep_pose, SIGNAL(pressed()), ui->ortho_view_top_, SLOT(vectorPressed()));
    QObject::connect(ui->grasp_model, SIGNAL(toggled(bool)), ui->ortho_view_top_, SLOT(graspModelToggled(bool)));
    QObject::connect(ui->grid_map, SIGNAL(toggled(bool)), ui->ortho_view_top_, SLOT(gridMapToggled(bool)));
    QObject::connect(ui->insert_waypoint, SIGNAL(pressed()), ui->ortho_view_top_, SLOT(insertWaypoint()));
    QObject::connect(ui->laser_scan_2, SIGNAL(toggled(bool)), ui->ortho_view_top_, SLOT(laserScanToggled(bool)));
    QObject::connect(ui->lidar_point_cloud_2, SIGNAL(toggled(bool)), ui->ortho_view_top_, SLOT(lidarPointCloudToggled(bool)));
    QObject::connect(ui->octomap_2, SIGNAL(toggled(bool)), ui->ortho_view_top_, SLOT(markerArrayToggled(bool)));
    QObject::connect(ui->point_cloud_request, SIGNAL(toggled(bool)), ui->ortho_view_top_, SLOT(requestedPointCloudToggled(bool)));
    QObject::connect(ui->request_point_cloud_, SIGNAL(clicked()), ui->ortho_view_top_, SLOT(publishPointCloudWorldRequest()));
    QObject::connect(ui->reset_map, SIGNAL(clicked()), ui->ortho_view_top_, SLOT(clearMapRequests()));
    QObject::connect(ui->reset_point_cloud, SIGNAL(clicked()), ui->ortho_view_top_, SLOT(clearPointCloudRequests()));
    QObject::connect(ui->robot_model_2, SIGNAL(toggled(bool)), ui->ortho_view_top_, SLOT(robotModelToggled(bool)));
    QObject::connect(ui->simulation_robot, SIGNAL(toggled(bool)), ui->ortho_view_top_, SLOT(simulationRobotToggled(bool)));
    QObject::connect(ui->stereo_point_cloud_2, SIGNAL(toggled(bool)), ui->ortho_view_top_, SLOT(stereoPointCloudToggled(bool)));
    QObject::connect(ui->template_tool, SIGNAL(toggled(bool)), ui->ortho_view_top_, SLOT(markerTemplateToggled(bool)));
    QObject::connect(ui->template_widget, SIGNAL(insertTemplate(QString)), ui->ortho_view_top_, SLOT(insertTemplate(QString)));
    QObject::connect(ui->template_widget, SIGNAL(templatePathChanged(QString)), ui->ortho_view_top_, SLOT(templatePathChanged(QString)));
    QObject::connect(ui->templates, SIGNAL(toggled(bool)), ui->ortho_view_top_, SLOT(templatesToggled(bool)));
    QObject::connect(ui->widget_tool, SIGNAL(toggled(bool)), ui->ortho_view_top_, SLOT(markerRobotToggled(bool)));


    // connect UI to perspective functions
    QObject::connect(ui->camera_tool, SIGNAL(toggled(bool)), ui->ortho_view_right_, SLOT(cameraToggled(bool)));
    QObject::connect(ui->footstep_planning, SIGNAL(toggled(bool)), ui->ortho_view_right_, SLOT(footstepPlanningToggled(bool)));
    QObject::connect(ui->footstep_pose, SIGNAL(pressed()), ui->ortho_view_right_, SLOT(vectorPressed()));
    QObject::connect(ui->grasp_model, SIGNAL(toggled(bool)), ui->ortho_view_right_, SLOT(graspModelToggled(bool)));
    QObject::connect(ui->grid_map, SIGNAL(toggled(bool)), ui->ortho_view_right_, SLOT(gridMapToggled(bool)));
    QObject::connect(ui->insert_waypoint, SIGNAL(pressed()), ui->ortho_view_right_, SLOT(insertWaypoint()));
    QObject::connect(ui->laser_scan_2, SIGNAL(toggled(bool)), ui->ortho_view_right_, SLOT(laserScanToggled(bool)));
    QObject::connect(ui->lidar_point_cloud_2, SIGNAL(toggled(bool)), ui->ortho_view_right_, SLOT(lidarPointCloudToggled(bool)));
    QObject::connect(ui->octomap_2, SIGNAL(toggled(bool)), ui->ortho_view_right_, SLOT(markerArrayToggled(bool)));
    QObject::connect(ui->point_cloud_request, SIGNAL(toggled(bool)), ui->ortho_view_right_, SLOT(requestedPointCloudToggled(bool)));
    QObject::connect(ui->request_point_cloud_, SIGNAL(clicked()), ui->ortho_view_right_, SLOT(publishPointCloudWorldRequest()));
    QObject::connect(ui->reset_map, SIGNAL(clicked()), ui->ortho_view_right_, SLOT(clearMapRequests()));
    QObject::connect(ui->reset_point_cloud, SIGNAL(clicked()), ui->ortho_view_right_, SLOT(clearPointCloudRequests()));
    QObject::connect(ui->robot_model_2, SIGNAL(toggled(bool)), ui->ortho_view_right_, SLOT(robotModelToggled(bool)));
    QObject::connect(ui->simulation_robot, SIGNAL(toggled(bool)), ui->ortho_view_right_, SLOT(simulationRobotToggled(bool)));
    QObject::connect(ui->stereo_point_cloud_2, SIGNAL(toggled(bool)), ui->ortho_view_right_, SLOT(stereoPointCloudToggled(bool)));
    QObject::connect(ui->template_tool, SIGNAL(toggled(bool)), ui->ortho_view_right_, SLOT(markerTemplateToggled(bool)));
    QObject::connect(ui->template_widget, SIGNAL(insertTemplate(QString)), ui->ortho_view_right_, SLOT(insertTemplate(QString)));
    QObject::connect(ui->template_widget, SIGNAL(templatePathChanged(QString)), ui->ortho_view_right_, SLOT(templatePathChanged(QString)));
    QObject::connect(ui->templates, SIGNAL(toggled(bool)), ui->ortho_view_right_, SLOT(templatesToggled(bool)));
    QObject::connect(ui->widget_tool, SIGNAL(toggled(bool)), ui->ortho_view_right_, SLOT(markerRobotToggled(bool)));


    // connect UI to perspective functions
    QObject::connect(ui->camera_tool, SIGNAL(toggled(bool)), ui->ortho_view_front_, SLOT(cameraToggled(bool)));
    QObject::connect(ui->footstep_planning, SIGNAL(toggled(bool)), ui->ortho_view_front_, SLOT(footstepPlanningToggled(bool)));
    QObject::connect(ui->footstep_pose, SIGNAL(pressed()), ui->ortho_view_front_, SLOT(vectorPressed()));
    QObject::connect(ui->grasp_model, SIGNAL(toggled(bool)), ui->ortho_view_front_, SLOT(graspModelToggled(bool)));
    QObject::connect(ui->grid_map, SIGNAL(toggled(bool)), ui->ortho_view_front_, SLOT(gridMapToggled(bool)));
    QObject::connect(ui->insert_waypoint, SIGNAL(pressed()), ui->ortho_view_front_, SLOT(insertWaypoint()));
    QObject::connect(ui->laser_scan_2, SIGNAL(toggled(bool)), ui->ortho_view_front_, SLOT(laserScanToggled(bool)));
    QObject::connect(ui->lidar_point_cloud_2, SIGNAL(toggled(bool)), ui->ortho_view_front_, SLOT(lidarPointCloudToggled(bool)));
    QObject::connect(ui->octomap_2, SIGNAL(toggled(bool)), ui->ortho_view_front_, SLOT(markerArrayToggled(bool)));
    QObject::connect(ui->point_cloud_request, SIGNAL(toggled(bool)), ui->ortho_view_front_, SLOT(requestedPointCloudToggled(bool)));
    QObject::connect(ui->request_point_cloud_, SIGNAL(clicked()), ui->ortho_view_front_, SLOT(publishPointCloudWorldRequest()));
    QObject::connect(ui->reset_map, SIGNAL(clicked()), ui->ortho_view_front_, SLOT(clearMapRequests()));
    QObject::connect(ui->reset_point_cloud, SIGNAL(clicked()), ui->ortho_view_front_, SLOT(clearPointCloudRequests()));
    QObject::connect(ui->robot_model_2, SIGNAL(toggled(bool)), ui->ortho_view_front_, SLOT(robotModelToggled(bool)));
    QObject::connect(ui->simulation_robot, SIGNAL(toggled(bool)), ui->ortho_view_front_, SLOT(simulationRobotToggled(bool)));
    QObject::connect(ui->stereo_point_cloud_2, SIGNAL(toggled(bool)), ui->ortho_view_front_, SLOT(stereoPointCloudToggled(bool)));
    QObject::connect(ui->template_tool, SIGNAL(toggled(bool)), ui->ortho_view_front_, SLOT(markerTemplateToggled(bool)));
    QObject::connect(ui->template_widget, SIGNAL(insertTemplate(QString)), ui->ortho_view_front_, SLOT(insertTemplate(QString)));
    QObject::connect(ui->template_widget, SIGNAL(templatePathChanged(QString)), ui->ortho_view_front_, SLOT(templatePathChanged(QString)));
    QObject::connect(ui->templates, SIGNAL(toggled(bool)), ui->ortho_view_front_, SLOT(templatesToggled(bool)));
    QObject::connect(ui->widget_tool, SIGNAL(toggled(bool)), ui->ortho_view_front_, SLOT(markerRobotToggled(bool)));
}

FourWindowTestWidget::~FourWindowTestWidget()
{
    delete ui;
}

bool FourWindowTestWidget::eventFilter( QObject * o, QEvent * e )
{
    if ( e->type() == QEvent::Wheel &&
         (qobject_cast<QAbstractSpinBox*>( o ) || qobject_cast<QAbstractSlider*>( o ) || qobject_cast<QComboBox*>( o )))
    {
        e->ignore();
        return true;
    }
    return QWidget::eventFilter( o, e );
}
