#include "perspective_view_widget.h"
#include "ui_perspective_view_widget.h"
#include "ui/template_loader_widget.h"

PerspectiveViewWidget::PerspectiveViewWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::PerspectiveViewWidget)
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
}

PerspectiveViewWidget::~PerspectiveViewWidget()
{
    delete ui;
}

bool PerspectiveViewWidget::eventFilter( QObject * o, QEvent * e )
{
    if ( e->type() == QEvent::Wheel &&
         (qobject_cast<QAbstractSpinBox*>( o ) || qobject_cast<QAbstractSlider*>( o ) || qobject_cast<QComboBox*>( o )))
    {
        e->ignore();
        return true;
    }
    return QWidget::eventFilter( o, e );
}
