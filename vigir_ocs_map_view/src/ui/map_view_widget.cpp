#include "map_view_widget.h"
#include "ui_map_view_widget.h"
#include "ui/template_loader_widget.h"
#include <rviz/displays_panel.h>

MapViewWidget::MapViewWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::MapViewWidget)
{
    ui->setupUi(this);

    QObject::connect(ui->robot_joint_markers,SIGNAL(toggled(bool)), ((vigir_ocs::Base3DView*)ui->map_view_), SLOT(robotJointMarkerToggled(bool)));
    QObject::connect(ui->robot_occlusion_rendering,SIGNAL(toggled(bool)), ((vigir_ocs::Base3DView*)ui->map_view_), SLOT(robotOcclusionToggled(bool)));

    std::string ip = ros::package::getPath("vigir_ocs_map_view")+"/icons/";
    icon_path_ = QString(ip.c_str());

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
    ui->Template->hide();
    ui->Tools->hide();
    ui->Navigation->hide();

    statusBar = new StatusBar();
    //need to connect position text
    connect(((vigir_ocs::Base3DView*)ui->map_view_),SIGNAL(sendPositionText(QString)),statusBar,SLOT(receivePositionText(QString)));
    connect(((vigir_ocs::Base3DView*)ui->map_view_),SIGNAL(sendFPS(int)),statusBar,SLOT(receiveFPS(int)));

    ui->sbarLayout->addWidget(statusBar);

    //connect context menu requests that depend on ui values
    connect(ui->map_view_,SIGNAL(UIrequestAreaMap()),this,SLOT(requestMap()));
    connect(ui->map_view_,SIGNAL(UIrequestOctomap()),this,SLOT(requestOctomap()));

    // connect emergency stop button to glancehub
    stop_mapper_ = new QSignalMapper(this);
    connect(stop_mapper_,SIGNAL(mapped(int)),statusBar->getGlanceSbar(),SLOT(receiveModeChange(int)));

    //map all toggles button to their identifiers
    stop_mapper_->setMapping(((vigir_ocs::Base3DView*)ui->map_view_),flor_control_msgs::FlorControlModeCommand::FLOR_STOP);

    //connect all buttons for mouse presses
    connect(((vigir_ocs::Base3DView*)ui->map_view_),SIGNAL(emergencyStop()),stop_mapper_,SLOT(map()));

    //Restore State
    QSettings settings("OCS", "map_view");
    this->restoreGeometry(settings.value("mainWindowGeometry").toByteArray());

    //setup toolbar and necessary components
    setupToolbar();

    //setup sidebar toggle button
    sidebar_toggle_ = new QPushButton(this);
    sidebar_toggle_->setStyleSheet("font: 9pt \"MS Shell Dlg 2\";background-color: rgb(0, 0, 0);color: rgb(108, 108, 108);border-color: rgb(0, 0, 0); ");
    sidebar_toggle_->setMaximumSize(25,25);

    QPixmap pix(icon_path_+"drawer.png");
    QIcon Btn(pix);
    sidebar_toggle_->setIcon(Btn);
    sidebar_toggle_->setIconSize(pix.rect().size() / 8);

    connect(sidebar_toggle_,SIGNAL(clicked()),this,SLOT(toggleSidebarVisibility()));

    ocs_sync_sub_ = n_.subscribe<flor_ocs_msgs::OCSSynchronize>( "/flor/ocs/synchronize", 5, &MapViewWidget::synchronizeToggleButtons, this );
    ocs_sync_pub_ = n_.advertise<flor_ocs_msgs::OCSSynchronize>( "/flor/ocs/synchronize", 5, false);

    timer.start(100, this);
}

MapViewWidget::~MapViewWidget()
{
    delete ui;
}

void MapViewWidget::timerEvent(QTimerEvent *event)
{
    sidebar_toggle_->setGeometry(ui->viewContainer->geometry().topRight().x() - 25 ,ui->viewContainer->geometry().top() + 43,25,25);
}

void MapViewWidget::changeCheckBoxState(QCheckBox* checkBox, Qt::CheckState state)
{
    //set checkbox state without calling callbacks
    checkBox->blockSignals(true);
    checkBox->setCheckState(state);
    checkBox->blockSignals(false);
}

void MapViewWidget::synchronizeToggleButtons(const flor_ocs_msgs::OCSSynchronize::ConstPtr &msg)
{
    for(int i=0;i<msg->properties.size();i++)
    {
        if(msg->properties[i].compare("LIDAR Point Cloud") == 0)
        {
            if(!msg->reset[i])
            {
                if(msg->visible[i])
                    changeCheckBoxState(ui->lidar_point_cloud_2,Qt::Checked);
                else
                    changeCheckBoxState(ui->lidar_point_cloud_2,Qt::Unchecked);
            }
        }
        else if(msg->properties[i].compare("Stereo Point Cloud") == 0)
        {
            if(!msg->reset[i])
            {
                if(msg->visible[i])
                    changeCheckBoxState(ui->stereo_point_cloud_2,Qt::Checked);
                else
                    changeCheckBoxState(ui->stereo_point_cloud_2,Qt::Unchecked);
            }
        }
        else if(msg->properties[i].compare("Raycast Point Cloud") == 0)
        {
            if(!msg->reset[i])
            {
                if(msg->visible[i])
                    changeCheckBoxState(ui->point_cloud_request,Qt::Checked);
                else
                    changeCheckBoxState(ui->point_cloud_request,Qt::Unchecked);
            }
        }
        else if(msg->properties[i].compare("Octomap") == 0)
        {
            if(!msg->reset[i])
            {
                if(msg->visible[i])
                    changeCheckBoxState(ui->octomap_2,Qt::Checked);
                else
                    changeCheckBoxState(ui->octomap_2,Qt::Unchecked);
            }
        }
        else if(msg->properties[i].compare("Ground map") == 0)
        {
            if(!msg->reset[i])
            {
                if(msg->visible[i])
                    changeCheckBoxState(ui->grid_map,Qt::Checked);
                else
                    changeCheckBoxState(ui->grid_map,Qt::Unchecked);
            }
        }
        else if(msg->properties[i].compare("Notification System") == 0)
        {
            if(!msg->reset[i])
            {
                if(msg->visible[i])
                    changeCheckBoxState(ui->notification_system,Qt::Checked);
                else
                    changeCheckBoxState(ui->notification_system,Qt::Unchecked);
            }
        }
    }
}

void MapViewWidget::toggleSidebarVisibility()
{
    if(ui->scrollArea->isVisible())
        ui->scrollArea->hide();
    else
        ui->scrollArea->show();
}

void MapViewWidget::setupToolbar()
{
    mapRegionConfig = new MapRegionConfigure();
    region3dConfig = new Region3DConfigure();

    //set menu to popup a config widget
    QWidgetAction *wa = new QWidgetAction(0);
    wa->setDefaultWidget(region3dConfig);
    regionMenu.addAction(wa);
    //associate button with menu
    ui->regionConfig->setMenu(&regionMenu);
    //need to install event filter for widget positioning
    regionMenu.installEventFilter(this);

    QWidgetAction *wa2 = new QWidgetAction(0);
    wa2->setDefaultWidget(mapRegionConfig);
    mapMenu.addAction(wa2);
    ui->mapConfig->setMenu(&mapMenu);
    mapMenu.installEventFilter(this);

    //connect buttons to bringup config widgets
    connect(ui->mapConfig,SIGNAL(clicked()),this,SLOT(toggleMapConfig()));
    connect(ui->regionConfig,SIGNAL(clicked()),this,SLOT(toggleRegionConfig()));

    //set button style
    loadButtonIconAndStyle(ui->regionConfig,"configIcon.png");
    loadButtonIconAndStyle(ui->mapConfig,"configIcon.png");
    loadButtonIconAndStyle(ui->request_point_cloud,""); //TO-DO: get cool icons
    loadButtonIconAndStyle(ui->request_octomap,"");
    loadButtonIconAndStyle(ui->request_map,"");

    //place arrow on combo box
    QString comboStyle = ui->point_cloud_type->styleSheet() + "\n" +
            "QComboBox::down-arrow {\n" +
            " image: url(" + icon_path_ + "down_arrow.png" + ");\n" +
            "}";
    ui->point_cloud_type->setStyleSheet(comboStyle);
}

void MapViewWidget::loadButtonIconAndStyle(QPushButton* btn, QString image_name)
{
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

void MapViewWidget::toggleMapConfig()
{
    ui->mapConfig->showMenu();
}

void MapViewWidget::toggleRegionConfig()
{
    ui->regionConfig->showMenu();
}

void MapViewWidget::closeEvent(QCloseEvent *event)
{
        QSettings settings("OCS", "map_view");
        settings.setValue("mainWindowGeometry", this->saveGeometry());        
}

void MapViewWidget::resizeEvent(QResizeEvent * event)
{    
    QSettings settings("OCS", "map_view");
    settings.setValue("mainWindowGeometry", this->saveGeometry());    
}

void MapViewWidget::moveEvent(QMoveEvent * event)
{
    QSettings settings("OCS", "map_view");
    settings.setValue("mainWindowGeometry", this->saveGeometry());    
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
    if(ui->map_view_->hasValidSelection())
    {
        if(ui->grid_map->checkState() == Qt::Unchecked)
        {            
            //set sidebar checkbox to be enabled and sync(handled through callback in views)
            flor_ocs_msgs::OCSSynchronize msg;
            msg.properties.push_back("Ground map");
            msg.reset.push_back(false);
            msg.visible.push_back(true);
            ocs_sync_pub_.publish(msg);
        }        
        //generate map
        ui->map_view_->requestMap(mapRegionConfig->getMinHeight(),mapRegionConfig->getMaxHeight(),mapRegionConfig->getResolution());       
    }
}

void MapViewWidget::requestOctomap()
{    
    if(ui->map_view_->hasValidSelection())
    {
        if(ui->octomap_2->checkState() == Qt::Unchecked)
        {
            flor_ocs_msgs::OCSSynchronize msg;
            msg.properties.push_back("Octomap");
            msg.reset.push_back(false);
            msg.visible.push_back(true);
            ocs_sync_pub_.publish(msg);
        }

        ui->map_view_->requestOctomap(region3dConfig->getMinHeight(),region3dConfig->getMaxHeight(),region3dConfig->getVoxelResolution());
    }

}

void MapViewWidget::requestPointCloud()
{      
    if(ui->map_view_->hasValidSelection())
    {
        //still need to sync checkbox, but based on pointcloud type
        if(ui->point_cloud_type->currentIndex() <= 1 && ui->lidar_point_cloud_2->checkState() == Qt::Unchecked) //lidar
        {
            flor_ocs_msgs::OCSSynchronize msg;
            msg.properties.push_back("LIDAR Point Cloud");
            msg.reset.push_back(false);
            msg.visible.push_back(true);
            ocs_sync_pub_.publish(msg);
        }
        else if (ui->point_cloud_type->currentIndex() > 1 && ui->stereo_point_cloud_2->checkState() == Qt::Unchecked) // stereo cases
        {
            flor_ocs_msgs::OCSSynchronize msg;
            msg.properties.push_back("Stereo Point Cloud");
            msg.reset.push_back(false);
            msg.visible.push_back(true);
            ocs_sync_pub_.publish(msg);
        }
        ui->map_view_->requestPointCloud(region3dConfig->getMinHeight(),region3dConfig->getMaxHeight(),region3dConfig->getVoxelResolution(),ui->point_cloud_type->currentIndex(),region3dConfig->getAggregSize());
    }
}

bool MapViewWidget::eventFilter( QObject * o, QEvent * e )
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
        if(((QMenu*)o) == ui->regionConfig->menu())
        {
            //get bottom left of button
            p.setX(0);
            p.setY(ui->regionConfig->geometry().height());
            //get global position of bottom left of button
            p = ui->regionConfig->mapToGlobal(p);
        }
        else if(((QMenu*)o) == ui->mapConfig->menu())
        {
            p.setX(0);
            p.setY(ui->mapConfig->geometry().height());
            p = ui->mapConfig->mapToGlobal(p);
        }
        ((QMenu*)o)->move(p); // move widget to position
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
            ui->map_view_->requestPointCloud(region3dConfig->getMinHeight(),region3dConfig->getMaxHeight(),region3dConfig->getVoxelResolution(),1,region3dConfig->getAggregSize());        
        else
            ui->map_view_->requestPointCloud(1);
    }
    else if(key_event->key == 12 && key_event->state && ctrl_is_pressed) // '3' - stereo
    {
        if(ui->map_view_->hasValidSelection())                   
            ui->map_view_->requestPointCloud(region3dConfig->getMinHeight(),region3dConfig->getMaxHeight(),region3dConfig->getVoxelResolution(),2,region3dConfig->getAggregSize());        
        else
            ui->map_view_->requestPointCloud(2);
    }
}
