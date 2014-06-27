#include "map_view_widget.h"
#include "ui_map_view_widget.h"
#include "ui/template_loader_widget.h"

#include <rviz/displays_panel.h>

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

    ui->insert_waypoint->hide();
    ui->joystick_steering->hide();
    ui->waypoint->hide();

    rviz::DisplaysPanel* displays_panel = new rviz::DisplaysPanel(this);
    displays_panel->initialize( ui->map_view_->getVisualizationManager());

    QVBoxLayout* displays_layout = new QVBoxLayout();
    displays_layout->setMargin(0);
    displays_layout->addWidget(displays_panel);
    ui->rviz_options->setLayout(displays_layout);

    key_event_sub_ = n_.subscribe<flor_ocs_msgs::OCSKeyEvent>( "/flor/ocs/key_event", 5, &MapViewWidget::processNewKeyEvent, this );

    ((vigir_ocs::Base3DView*)ui->map_view_)->setTemplateTree(ui->template_widget_3->getTreeRoot());

    //hide sidebar elements that aren't necessary
    //ui->Template->hide();
    //ui->Tools->hide();
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
    ui->map_view_->requestOctomap(ui->oct_min_z->value(),ui->oct_max_z->value(),0.05);
}

void MapViewWidget::requestPointCloud()
{
  ui->map_view_->requestPointCloud(ui->oct_min_z->value(),ui->oct_max_z->value(),ui->vox_res->value(),ui->point_cloud_type->currentIndex(),ui->agg_size->value());
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

void MapViewWidget::processNewKeyEvent(const flor_ocs_msgs::OCSKeyEvent::ConstPtr &key_event)
{
    // store key state
    if(key_event->state)
        keys_pressed_list_.push_back(key_event->key);
    else
        keys_pressed_list_.erase(std::remove(keys_pressed_list_.begin(), keys_pressed_list_.end(), key_event->key), keys_pressed_list_.end());

    // process hotkeys
    bool ctrl_is_pressed = (std::find(keys_pressed_list_.begin(), keys_pressed_list_.end(), 37) != keys_pressed_list_.end());
    bool shift_is_pressed = (std::find(keys_pressed_list_.begin(), keys_pressed_list_.end(), 50) != keys_pressed_list_.end());
    bool alt_is_pressed = (std::find(keys_pressed_list_.begin(), keys_pressed_list_.end(), 64) != keys_pressed_list_.end());
    //std::vector<int>::iterator key_is_pressed;

    //key_is_pressed = std::find(keys_pressed_list_.begin(), keys_pressed_list_.end(), 37);
    //if(key_event->key == 32 && key_event->state && key_is_pressed != keys_pressed_list_.end()) // ctrl+o
    //    requestOctomap();
    //if(key_event->key == 58 && key_event->state && key_is_pressed != keys_pressed_list_.end()) // ctrl+m
    //    requestMap();
    if(key_event->key == 11 && key_event->state && ctrl_is_pressed) // '2' - unfiltered
    {
        if(ui->map_view_->hasValidSelection())
            ui->map_view_->requestPointCloud(ui->oct_min_z->value(),ui->oct_max_z->value(),ui->vox_res->value(),1,ui->agg_size->value());
        else
            ui->map_view_->requestPointCloud(1);
    }
    else if(key_event->key == 12 && key_event->state && ctrl_is_pressed) // '3' - stereo
    {
        if(ui->map_view_->hasValidSelection())
            ui->map_view_->requestPointCloud(ui->oct_min_z->value(),ui->oct_max_z->value(),ui->vox_res->value(),2,ui->agg_size->value());
        else
            ui->map_view_->requestPointCloud(2);
    }
}
