#include "map_view_widget.h"
#include "ui_map_view_widget.h"
#include "ui/template_loader_widget.h"
#include "joystick.h"

MapViewWidget::MapViewWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::MapViewWidget)
{
    ui->setupUi(this);

    ui->insert_waypoint->hide();

    connect(ui->joystick_steering, SIGNAL(toggled(bool)), this, SLOT(hideWaypointButton()));
    connect(ui->waypoint, SIGNAL(toggled(bool)), this, SLOT(hideJoystick()));

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

    //ui->octomap_2->hide();
    //ui->lidar_point_cloud_2->hide();
    //ui->stereo_point_cloud_2->hide();
    //ui->laser_scan_2->hide();
}

MapViewWidget::~MapViewWidget()
{
    delete ui;
}

void MapViewWidget::hideWaypointButton()
{
    ui->insert_waypoint->hide();
}

void MapViewWidget::hideJoystick()
{
    ui->insert_waypoint->show();
}

void MapViewWidget::requestMap()
{
    ui->map_view_->requestMap(ui->map_min_z->value(),ui->map_max_z->value(),ui->map_res->value());
}

void MapViewWidget::requestOctomap()
{
    ui->map_view_->requestOctomap(ui->oct_min_z->value(),ui->oct_max_z->value(),ui->oct_res->value());
}

bool MapViewWidget::eventFilter( QObject * o, QEvent * e )
{
    if ( e->type() == QEvent::Wheel &&
         (qobject_cast<QAbstractSpinBox*>( o ) || qobject_cast<QAbstractSlider*>( o ) || qobject_cast<QComboBox*>( o )))
    {
        e->ignore();
        return true;
    }
    return QWidget::eventFilter( o, e );
}
