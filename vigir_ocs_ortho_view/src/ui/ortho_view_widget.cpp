#include "ortho_view_widget.h"
#include "ui_ortho_view_widget.h"
#include "ui/template_loader_widget.h"

OrthoViewWidget::OrthoViewWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::OrthoViewWidget)
{
    ui->setupUi(this);

    ortho_view_ = new vigir_ocs::OrthoView();

    QHBoxLayout* aux_layout = new QHBoxLayout();
    aux_layout->setMargin(0);
    aux_layout->addWidget(ortho_view_);
    ui->ortho_view_->setLayout(aux_layout);

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

    // connect UI buttons to
    QObject::connect(ui->camera_tool_3, SIGNAL(toggled(bool)), ui->ortho_view_, SLOT(cameraToggled(bool)));
    QObject::connect(ui->footstep_planning, SIGNAL(toggled(bool)), ui->ortho_view_, SLOT(footstepPlanningToggled(bool)));
    QObject::connect(ui->footstep_pose, SIGNAL(pressed()), ui->ortho_view_, SLOT(vectorPressed()));
    QObject::connect(ui->grasp_model, SIGNAL(toggled(bool)), ui->ortho_view_, SLOT(graspModelToggled(bool)));
    QObject::connect(ui->grid_map, SIGNAL(toggled(bool)), ui->ortho_view_, SLOT(gridMapToggled(bool)));
    QObject::connect(ui->laser_scan_2, SIGNAL(toggled(bool)), ui->ortho_view_, SLOT(laserScanToggled(bool)));
    QObject::connect(ui->lidar_point_cloud_2, SIGNAL(toggled(bool)), ui->ortho_view_, SLOT(lidarPointCloudToggled(bool)));
    QObject::connect(ui->octomap_2, SIGNAL(toggled(bool)), ui->ortho_view_, SLOT(markerArrayToggled(bool)));
    QObject::connect(ui->point_cloud_request, SIGNAL(toggled(bool)), ui->ortho_view_, SLOT(cameraToggled(bool)));
    QObject::connect(ui->request_point_cloud_, SIGNAL(clicked()), ui->ortho_view_, SLOT(publishPointCloudWorldRequest()));
    QObject::connect(ui->reset_map, SIGNAL(clicked()), ui->ortho_view_, SLOT(clearMapRequests()));
    QObject::connect(ui->reset_point_cloud, SIGNAL(clicked()), ui->ortho_view_, SLOT(clearPointCloudRequests()));
    QObject::connect(ui->robot_model_2, SIGNAL(toggled(bool)), ui->ortho_view_, SLOT(robotModelToggled(bool)));
    QObject::connect(ui->simulation_robot, SIGNAL(toggled(bool)), ui->ortho_view_, SLOT(simulationRobotToggled(bool)));
    QObject::connect(ui->stereo_point_cloud_2, SIGNAL(toggled(bool)), ui->ortho_view_, SLOT(stereoPointCloudToggled(bool)));
    QObject::connect(ui->template_tool_3, SIGNAL(toggled(bool)), ui->ortho_view_, SLOT(markerTemplateToggled(bool)));
    QObject::connect(ui->template_widget_3, SIGNAL(insertTemplate(QString)), ui->ortho_view_, SLOT(insertTemplate(QString)));
    QObject::connect(ui->template_widget_3, SIGNAL(templatePathChanged(QString)), ui->ortho_view_, SLOT(templatePathChanged(QString)));
    QObject::connect(ui->templates, SIGNAL(toggled(bool)), ui->ortho_view_, SLOT(templatesToggled(bool)));
}

OrthoViewWidget::~OrthoViewWidget()
{
    delete ui;
}

bool OrthoViewWidget::eventFilter( QObject * o, QEvent * e )
{
    if ( e->type() == QEvent::Wheel &&
         (qobject_cast<QAbstractSpinBox*>( o ) || qobject_cast<QAbstractSlider*>( o ) || qobject_cast<QComboBox*>( o )))
    {
        e->ignore();
        return true;
    }
    return QWidget::eventFilter( o, e );
}
