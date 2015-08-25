#include "map_view_widget.h"
#include "ui_map_view_widget.h"
#include "ui/template_loader_widget.h"
#include <rviz/displays_panel.h>
#include "base_context_menu.h"
#include "map_context_menu.h"

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

    //key_event_sub_ = n_.subscribe<vigir_ocs_msgs::OCSKeyEvent>( "/flor/ocs/key_event", 5, &MapViewWidget::processNewKeyEvent, this );

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
    connect(stop_mapper_,SIGNAL(mapped(int)),statusBar->getGlanceSbar(),SLOT(modeChanged(int)));

    //map all toggles button to their identifiers
    stop_mapper_->setMapping(((vigir_ocs::Base3DView*)ui->map_view_), vigir_control_msgs::VigirControlModeCommand::EMERGENCY_STOP);//vigir_control_msgs::VigirControlModeCommand::FLOR_STOP);

    //connect all buttons for mouse presses
    connect(((vigir_ocs::Base3DView*)ui->map_view_),SIGNAL(emergencyStop()),stop_mapper_,SLOT(map()));

    //Restore State
    QSettings settings("OCS", "map_view");
    this->restoreGeometry(settings.value("mainWindowGeometry").toByteArray());

    //setup toolbar and necessary components
    setupToolbar();

    //setup template tree for context menu
    ((vigir_ocs::Base3DView*)ui->map_view_)->getBaseContextMenu()->setTemplateTree(ui->template_widget_3->getTreeRoot());

    //setup sidebar toggle button
    sidebar_toggle_ = new QPushButton(this);
    sidebar_toggle_->setStyleSheet("font: 9pt \"MS Shell Dlg 2\";background-color: rgb(0, 0, 0);color: rgb(108, 108, 108);border-color: rgb(0, 0, 0); ");
    sidebar_toggle_->setMaximumSize(25,25);

    QPixmap pix(icon_path_+"drawer.png");
    QIcon Btn(pix);
    sidebar_toggle_->setIcon(Btn);
    sidebar_toggle_->setIconSize(pix.rect().size() / 8);

    connect(sidebar_toggle_,SIGNAL(clicked()),this,SLOT(toggleSidebarVisibility()));

    ocs_sync_sub_ = n_.subscribe<vigir_ocs_msgs::OCSSynchronize>( "/flor/ocs/synchronize", 5, &MapViewWidget::synchronizeToggleButtons, this );
    ocs_sync_pub_ = n_.advertise<vigir_ocs_msgs::OCSSynchronize>( "/flor/ocs/synchronize", 5, false);

    //publisher and subscriber for window visibility control
    window_control_sub_ = n_.subscribe<std_msgs::Int8>( "/flor/ocs/window_control", 5, &MapViewWidget::processWindowControl, this );
    window_control_pub_ = n_.advertise<std_msgs::Int8>( "/flor/ocs/window_control", 1, false);

    timer.start(100, this);

    addHotkeys();
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

void MapViewWidget::synchronizeToggleButtons(const vigir_ocs_msgs::OCSSynchronize::ConstPtr &msg)
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
        else if(msg->properties[i].compare("Update Ghost Opacity") == 0)
        {
            if(!msg->reset[i])
            {
                if(msg->visible[i])
                    changeCheckBoxState(ui->update_ghost_opacity,Qt::Checked);
                else
                    changeCheckBoxState(ui->update_ghost_opacity,Qt::Unchecked);
            }
        }
        else if(msg->properties[i].compare("Frustum Display") == 0)
        {
            if(!msg->reset[i])
            {
                if(msg->visible[i])
                    changeCheckBoxState(ui->camera_frustum,Qt::Checked);
                else
                    changeCheckBoxState(ui->camera_frustum,Qt::Unchecked);
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
    footstep_configure_widget_ = new FootstepConfigure();

    //connect to update footstep paramaters from ui
    connect(footstep_configure_widget_,SIGNAL(sendFootstepParamaters(double,int,double,int)),
            ((vigir_ocs::Base3DView*)ui->map_view_)->getFootstepVisManager(),SLOT(updateFootstepParamaters(double,int,double,int)));
    connect(ui->use3dPlanning,SIGNAL(clicked(bool)),
            ((vigir_ocs::Base3DView*)ui->map_view_)->getFootstepVisManager(),SLOT(update3dPlanning(bool)));
    connect(((vigir_ocs::Base3DView*)ui->map_view_)->getFootstepVisManager(),SIGNAL(setFootstepParamaters(double,int,double,int)),
            footstep_configure_widget_,SLOT(updateFootstepParamaters(double,int,double,int)));
    connect(((vigir_ocs::Base3DView*)ui->map_view_)->getFootstepVisManager(),SIGNAL(set3dPlanning(bool)),
            this,SLOT(update3dPlanning(bool)));

    footstep_configure_widget_->emitCurrentConfig();

    //set menu to popup a config widget
    QWidgetAction *wa = new QWidgetAction(0);
    wa->setDefaultWidget(region3dConfig);
    regionMenu.addAction(wa);
    //associate button with menu
    ui->regionConfig->setMenu(&regionMenu);
    //need to install event filter for widget positioning
    regionMenu.installEventFilter(this);

    //set menu to popup a config widget
    QWidgetAction *wa2 = new QWidgetAction(0);
    wa2->setDefaultWidget(mapRegionConfig);
    mapMenu.addAction(wa2);    
    ui->mapConfig->setMenu(&mapMenu);    
    mapMenu.installEventFilter(this);

    //set menu to popup a config widget for footstep Params
    QWidgetAction *wa3 = new QWidgetAction(0);
    wa3->setDefaultWidget(footstep_configure_widget_);
    footstep_menu_.addAction(wa3);    
    ui->footstepConfigBtn->setMenu(&footstep_menu_);    
    footstep_menu_.installEventFilter(this);

    //connect buttons to bringup config widgets
    connect(ui->mapConfig,SIGNAL(clicked()),this,SLOT(toggleMapConfig()));
    connect(ui->regionConfig,SIGNAL(clicked()),this,SLOT(toggleRegionConfig()));
    connect(ui->footstepConfigBtn,SIGNAL(clicked()),this,SLOT(toggleFootstepConfig()));

    //set button style
    loadButtonIconAndStyle(ui->regionConfig,"configIcon.png");
    loadButtonIconAndStyle(ui->mapConfig,"configIcon.png");
    loadButtonIconAndStyle(ui->request_point_cloud,""); //TO-DO: get cool icons
    loadButtonIconAndStyle(ui->request_octomap,"");
    loadButtonIconAndStyle(ui->request_map,"");
    //loadButtonIcon(ui->basicStepBtn, "footBasicIcon.png");
    loadButtonIconAndStyle(ui->stepBtn, "footAdvancedIcon.png");
    loadButtonIconAndStyle(ui->footstepParamBtn, "footParamIcon.png");
    loadButtonIconAndStyle(ui->footstepConfigBtn,"configIcon.png");

    //use signalmapper to avoid having one function for each one of the toggle buttons
    toggle_mapper_ = new QSignalMapper(this);
    connect(toggle_mapper_,SIGNAL(mapped(int)),this,SLOT(toggleWindow(int)));

    //map all toggles button to their identifiers
    //toggle_mapper_->setMapping(this->ui->basicStepBtn,WINDOW_FOOTSTEP_BASIC);
    toggle_mapper_->setMapping(this->ui->stepBtn,WINDOW_FOOTSTEP_ADVANCED);
    toggle_mapper_->setMapping(this->ui->footstepParamBtn,WINDOW_FOOTSTEP_PARAMETER);

    //connect all buttons for mouse presses
    //connect(ui->basicStepBtn,SIGNAL(toggled(bool)),toggle_mapper_,SLOT(map()));
    connect(ui->stepBtn,SIGNAL(toggled(bool)),toggle_mapper_,SLOT(map()));
    connect(ui->footstepParamBtn,SIGNAL(toggled(bool)),toggle_mapper_,SLOT(map()));

    //place arrow on combo box
    QString comboStyle = ui->point_cloud_type->styleSheet() + "\n" +
            "QComboBox::down-arrow {\n" +
            " image: url(" + icon_path_ + "down_arrow.png" + ");\n" +
            "}";
    ui->point_cloud_type->setStyleSheet(comboStyle);

    //set for footstep param box
    comboStyle = ui->footstepParamSetBox->styleSheet() + "\n" +
            "QComboBox::down-arrow {\n" +
            " image: url(" + icon_path_ + "down_arrow.png" + ");\n" +
            "}";
    ui->footstepParamSetBox->setStyleSheet(comboStyle);

    //populate parameter box
    connect(ui->footstepParamSetBox,SIGNAL(currentIndexChanged(QString)),((vigir_ocs::Base3DView*)ui->map_view_)->getFootstepVisManager(),SLOT(setFootstepParameterSet(QString)));
    connect(((vigir_ocs::Base3DView*)ui->map_view_)->getFootstepVisManager(),SIGNAL(populateFootstepParameterSetBox(std::vector<std::string>)),this,SLOT(populateFootstepParameterSetBox(std::vector<std::string>)));
    connect(((vigir_ocs::Base3DView*)ui->map_view_)->getFootstepVisManager(),SIGNAL(setFootstepParameterSetBox(std::string)),this,SLOT(setFootstepParameterSetBox(std::string)));
}

void MapViewWidget::loadButtonIconAndStyle(QPushButton* btn, QString image_name)
{
    btn->setStyleSheet(QString("QPushButton  { ") +
                       " font: 9pt \"Ubuntu\";" +
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
                       " font: 9pt \"Ubuntu\";" +
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

void MapViewWidget::toggleFootstepConfig()
{
    ui->footstepConfigBtn->showMenu();
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
            vigir_ocs_msgs::OCSSynchronize msg;
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
            vigir_ocs_msgs::OCSSynchronize msg;
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
            vigir_ocs_msgs::OCSSynchronize msg;
            msg.properties.push_back("LIDAR Point Cloud");
            msg.reset.push_back(false);
            msg.visible.push_back(true);
            ocs_sync_pub_.publish(msg);
        }
        else if (ui->point_cloud_type->currentIndex() > 1 && ui->stereo_point_cloud_2->checkState() == Qt::Unchecked) // stereo cases
        {
            vigir_ocs_msgs::OCSSynchronize msg;
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
        else if(((QMenu*)o) == ui->footstepConfigBtn->menu())
        {
            p.setX(0);
            p.setY(ui->footstepConfigBtn->geometry().height());
            p = ui->footstepConfigBtn->mapToGlobal(p);
        }

        ((QMenu*)o)->move(p); // move widget to position
        return true;
    }
//    if( qobject_cast<QComboBox*>( o ) && qobject_cast<QComboBox*>( o ) == ui->footstepParamSetBox)
//    {
//        e->ignore();
//        return true;
//    }
    if( qobject_cast<QCheckBox*>( o ) && qobject_cast<QCheckBox*>( o ) == ui->use3dPlanning)
    {
        e->ignore();
        return true;
    }

    return QWidget::eventFilter( o, e );
}

void MapViewWidget::addHotkeys()
{
    HotkeyManager::Instance()->addHotkeyFunction("ctrl+2",boost::bind(&MapViewWidget::unfilteredHotkey,this));
    HotkeyManager::Instance()->addHotkeyFunction("ctrl+3",boost::bind(&MapViewWidget::stereoHotkey,this));
}

void MapViewWidget::unfilteredHotkey()
{
    if(ui->map_view_->hasValidSelection())
        ui->map_view_->requestPointCloud(region3dConfig->getMinHeight(),region3dConfig->getMaxHeight(),region3dConfig->getVoxelResolution(),1,region3dConfig->getAggregSize());
    else
        ui->map_view_->requestPointCloud(1);
}
void MapViewWidget::stereoHotkey()
{
    if(ui->map_view_->hasValidSelection())
        ui->map_view_->requestPointCloud(region3dConfig->getMinHeight(),region3dConfig->getMaxHeight(),region3dConfig->getVoxelResolution(),2,region3dConfig->getAggregSize());
    else
        ui->map_view_->requestPointCloud(2);
}

void MapViewWidget::toggleWindow(int window)
{
    std_msgs::Int8 cmd;
    cmd.data = ((QPushButton*)toggle_mapper_->mapping(window))->isChecked() ? window : -window;
    window_control_pub_.publish(cmd);
}

void MapViewWidget::processWindowControl(const std_msgs::Int8::ConstPtr &visible)
{
    char visibility = visible->data;

    switch(abs(visibility))
    {
        case HIDE_ALL_WINDOWS:
            //ui->basicStepBtn->setChecked(false);
            ui->stepBtn->setChecked(false);
            ui->footstepParamBtn->setChecked(false);
            break;
        case WINDOW_FOOTSTEP_BASIC:
            //ui->basicStepBtn->setChecked(visibility > 0 ? true : false);
            break;
        case WINDOW_FOOTSTEP_ADVANCED:
            ui->stepBtn->setChecked(visibility > 0 ? true : false);
            break;
        case WINDOW_FOOTSTEP_PARAMETER:
            ui->footstepParamBtn->setChecked(visibility > 0 ? true : false);
            break;
        default:
            break;
    }
}

void MapViewWidget::populateFootstepParameterSetBox(std::vector<std::string> parameter_sets)
{
    // first we need to check if they're different than the one we have
    bool need_clear = false;
    for(int i = 0; i < parameter_sets.size(); i++)
    {
        if(QString(parameter_sets[i].c_str()) != ui->footstepParamSetBox->itemText(i))
        {
            need_clear = true;
            break;
        }
    }

    // then we only repopulate the combo box if needed
    if(need_clear)
    {
        ui->footstepParamSetBox->clear();
        for(int i = 0; i < parameter_sets.size(); i++)
        {
            ui->footstepParamSetBox->addItem(QString(parameter_sets[i].c_str()));
        }
    }
}

void MapViewWidget::setFootstepParameterSetBox(std::string parameter_set)
{
    for(int i = 0; i < ui->footstepParamSetBox->count(); i++)
    {
        if(QString(parameter_set.c_str()) == ui->footstepParamSetBox->itemText(i))
        {
            ui->footstepParamSetBox->installEventFilter(this);
            ui->footstepParamSetBox->setCurrentIndex(i);
            ui->footstepParamSetBox->removeEventFilter(this);
        }
    }
}

void MapViewWidget::update3dPlanning(bool checked)
{
    ui->use3dPlanning->installEventFilter(this);
    ui->use3dPlanning->setChecked(checked);
    ui->use3dPlanning->removeEventFilter(this);
}
