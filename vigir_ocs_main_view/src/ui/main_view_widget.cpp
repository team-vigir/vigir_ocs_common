#include "main_view_widget.h"
#include "ui_main_view_widget.h"
#include "base_context_menu.h"

MainViewWidget::MainViewWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::MainViewWidget)
{       
    ui->setupUi(this);
    //will not call destructor immediately without setting attribute
    this->setAttribute(Qt::WA_DeleteOnClose);

    //remove mousewheel support from objects in sidebar
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

    //hide waypoint button since we're not using it anymore
    ui->insert_waypoint->hide();

    //subscribe to the global hotkey message
   // key_event_sub_ = n_.subscribe<flor_ocs_msgs::OCSKeyEvent>( "/flor/ocs/key_event", 5, &MainViewWidget::processNewKeyEvent, this );

    //setup lidar spin
    lidar_spin_rate_pub_ = n_.advertise<std_msgs::Float64>("/multisense/set_spindle_speed",10,false);
    QObject::connect(ui->lidar_spin_rate, SIGNAL(valueChanged(double)), this, SLOT(setLidarSpinRate(double)));

    QHBoxLayout* aux_layout;

    // setup default views
    views_list_["Top Left"] = new vigir_ocs::PerspectiveView(NULL, "/world", "MainView");
    views_list_["Top Right"] = new vigir_ocs::OrthoView(((vigir_ocs::PerspectiveView*)views_list_["Top Left"]),"/pelvis", "MainView"); //views_list_["Top Right"] = new vigir_ocs::OrthoView();
    views_list_["Bottom Left"] = new vigir_ocs::OrthoView(((vigir_ocs::PerspectiveView*)views_list_["Top Left"]),"/pelvis", "MainView"); //views_list_["Bottom Left"] = new vigir_ocs::OrthoView();
    views_list_["Bottom Right"] = new vigir_ocs::OrthoView(((vigir_ocs::PerspectiveView*)views_list_["Top Left"]),"/pelvis", "MainView"); //views_list_["Bottom Right"] = new vigir_ocs::OrthoView();

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

    ((vigir_ocs::OrthoView*)views_list_["Top Right"])->setViewPlane("XY");
    ((vigir_ocs::OrthoView*)views_list_["Bottom Left"])->setViewPlane("XZ");
    ((vigir_ocs::OrthoView*)views_list_["Bottom Right"])->setViewPlane("YZ");

    std::map<std::string, QWidget*>::iterator iter;

    for (iter = views_list_.begin(); iter != views_list_.end(); ++iter)
    {
        // only need to connect to the main rviz instance (maybe not all of them?
        if(iter->second == views_list_["Top Left"])
        {
            ((vigir_ocs::Base3DView*)iter->second)->updateRenderMask(true);

            ((vigir_ocs::Base3DView*)iter->second)->simulationRobotToggled(true);
            ((vigir_ocs::Base3DView*)iter->second)->simulationRobotToggled(false);

            // connect UI to perspective functions
            QObject::connect(ui->camera_tool, SIGNAL(toggled(bool)), iter->second, SLOT(cameraToggled(bool)));
            QObject::connect(ui->footstep_planning, SIGNAL(toggled(bool)), iter->second, SLOT(footstepPlanningToggled(bool)));
            QObject::connect(ui->footstep_pose, SIGNAL(pressed()), iter->second, SLOT(defineFootstepGoal()));
            QObject::connect(ui->grasp_model, SIGNAL(toggled(bool)), iter->second, SLOT(graspModelToggled(bool)));
            QObject::connect(ui->grid_map, SIGNAL(toggled(bool)), iter->second, SLOT(gridMapToggled(bool)));
            QObject::connect(ui->insert_waypoint, SIGNAL(pressed()), iter->second, SLOT(insertWaypoint()));
            QObject::connect(ui->laser_scan_2, SIGNAL(toggled(bool)), iter->second, SLOT(laserScanToggled(bool)));
            QObject::connect(ui->lidar_point_cloud_2, SIGNAL(toggled(bool)), iter->second, SLOT(lidarPointCloudToggled(bool)));
            QObject::connect(ui->octomap_2, SIGNAL(toggled(bool)), iter->second, SLOT(markerArrayToggled(bool)));
            QObject::connect(ui->point_cloud_request, SIGNAL(toggled(bool)), iter->second, SLOT(requestedPointCloudToggled(bool)));
            QObject::connect(ui->request_point_cloud_, SIGNAL(clicked()), iter->second, SLOT(publishPointCloudWorldRequest()));
            QObject::connect(ui->reset_map, SIGNAL(clicked()), iter->second, SLOT(clearMapRequests()));
            QObject::connect(ui->reset_point_cloud, SIGNAL(clicked()), iter->second, SLOT(clearPointCloudRaycastRequests()));
            QObject::connect(ui->reset_point_cloud_lidar, SIGNAL(clicked()), iter->second, SLOT(clearPointCloudRegionRequests()));
            QObject::connect(ui->reset_point_cloud_stereo, SIGNAL(clicked()), iter->second, SLOT(clearPointCloudStereoRequests()));
            QObject::connect(ui->robot_model_2, SIGNAL(toggled(bool)), iter->second, SLOT(robotModelToggled(bool)));
            QObject::connect(ui->simulation_robot, SIGNAL(toggled(bool)), iter->second, SLOT(simulationRobotToggled(bool)));
            QObject::connect(ui->stereo_point_cloud_2, SIGNAL(toggled(bool)), iter->second, SLOT(stereoPointCloudToggled(bool)));
            QObject::connect(ui->template_tool, SIGNAL(toggled(bool)), iter->second, SLOT(markerTemplateToggled(bool)));
            QObject::connect(ui->template_widget, SIGNAL(insertTemplate(QString)), iter->second, SLOT(insertTemplate(QString)));
            QObject::connect(ui->template_widget, SIGNAL(templatePathChanged(QString)), iter->second, SLOT(templatePathChanged(QString)));
            QObject::connect(ui->templates, SIGNAL(toggled(bool)), iter->second, SLOT(templatesToggled(bool)));
            QObject::connect(ui->widget_tool, SIGNAL(toggled(bool)), iter->second, SLOT(markerRobotToggled(bool)));
            QObject::connect(ui->robot_joint_markers,SIGNAL(toggled(bool)), iter->second, SLOT(robotJointMarkerToggled(bool)));
            QObject::connect(ui->robot_occlusion_rendering,SIGNAL(toggled(bool)), iter->second, SLOT(robotOcclusionToggled(bool)));            
            QObject::connect(ui->notification_system,SIGNAL(toggled(bool)), iter->second, SLOT(notificationSystemToggled(bool)));
            QObject::connect(ui->update_ghost_opacity,SIGNAL(toggled(bool)), iter->second, SLOT(updateGhostRobotOpacityToggled(bool)));
            QObject::connect(ui->camera_frustum,SIGNAL(toggled(bool)), iter->second, SLOT(cameraFrustumToggled(bool)));
        }
        else
        {
            ((vigir_ocs::Base3DView*)iter->second)->updateRenderMask(false);
        }
    }

    std::string ip = ros::package::getPath("vigir_ocs_main_view")+"/icons/";
    icon_path_ = QString(ip.c_str());

    //contains both view buttons for alignment
    position_widget_ = new QWidget(views_list_["Top Left"]);
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

    rviz::DisplaysPanel* displays_panel = new rviz::DisplaysPanel(this);
    displays_panel->setMaximumWidth(225);
    displays_panel->initialize( ((vigir_ocs::PerspectiveView*)views_list_["Top Left"])->getVisualizationManager());

    rviz::ViewsPanel* views_panel = new rviz::ViewsPanel(this);
    views_panel->setViewManager(((vigir_ocs::PerspectiveView*)views_list_["Top Left"])->getVisualizationManager()->getViewManager());

    QVBoxLayout* displays_layout = new QVBoxLayout();
    displays_layout->setMargin(0);
    displays_layout->addWidget(displays_panel);
    displays_layout->addWidget(views_panel);
    ui->rviz_options->setLayout(displays_layout);

    QObject::connect(ui->ft_sensor, SIGNAL(toggled(bool)), this, SLOT(ft_sensorToggled(bool)));
    QObject::connect(ui->zero_left, SIGNAL(pressed()), this, SLOT(zero_leftPressed()));
    QObject::connect(ui->zero_right, SIGNAL(pressed()), this, SLOT(zero_rightPressed()));

    //reset for the hand f/t sensors
    ft_zero_pub_ = n_.advertise<std_msgs::Int8>("/flor/controller/zero_hand_wrench",1,false);

    //publisher and subscriber for window visibility control
    window_control_sub_ = n_.subscribe<std_msgs::Int8>( "/flor/ocs/window_control", 5, &MainViewWidget::processWindowControl, this );
    window_control_pub_ = n_.advertise<std_msgs::Int8>( "/flor/ocs/window_control", 1, false);

    //initialize footstep configuration widget
    footstep_configure_widget_ = new FootstepConfigure();
    //connect to update footstep paramaters from ui
    connect(footstep_configure_widget_,SIGNAL(sendFootstepParamaters(double,int,double,int,bool)),
            ((vigir_ocs::Base3DView*) views_list_["Top Left"])->getFootstepVisManager(),SLOT(updateFootstepParamaters(double,int,double,int,bool)));

    // setup all buttons/icons in the toolbar
    setupToolbar();

    //publisher for the interactive marker mode
    //interactive_marker_mode_pub_ = n_.advertise<std_msgs::Int8>("/flor/ocs/interactive_marker_server/set_mode",1,false);

    statusBar = new StatusBar(this);

    //connect view to update position data and fps
    connect(views_list_["Top Left"],SIGNAL(sendPositionText(QString)),statusBar,SLOT(receivePositionText(QString)));
    connect(views_list_["Top Left"],SIGNAL(sendFPS(int)),statusBar,SLOT(receiveFPS(int)));

    ui->statusLayout->addWidget(statusBar);
    grasp_toggle_button_ = new QPushButton(this);

    grasp_toggle_button_->setStyleSheet("font: 9pt \"MS Shell Dlg 2\";background-color: rgb(0, 0, 0);color: rgb(108, 108, 108);border-color: rgb(0, 0, 0); ");
    grasp_toggle_button_->setMaximumSize(60,30);
    grasp_toggle_button_->adjustSize();

    QPixmap pixmap(icon_path_+"graspUp.png");
    QIcon ButtonIcon(pixmap);
    grasp_toggle_button_->setIcon(ButtonIcon);
    grasp_toggle_button_->setIconSize(pixmap.rect().size()/5);

    connect(grasp_toggle_button_, SIGNAL(clicked()), this, SLOT(graspWidgetToggle()));

    //put two grasp widgets into container to have same focus
    graspContainer = new QWidget(this);
    QHBoxLayout * graspLayout = new QHBoxLayout();
    graspLayout->setSpacing(1);
    graspLayout->setMargin(1);
    graspLayout->setContentsMargins(0,0,0,0);
    graspContainer->setLayout(graspLayout);    
    graspContainer->hide();

    //Left Grasp Widget
    QWidget * leftGrasp = new QWidget(graspContainer);
    QVBoxLayout * leftLayout = new QVBoxLayout();
    leftLayout->setSpacing(0);
    leftLayout->setMargin(0);
    leftLayout->setContentsMargins(0,0,0,0);
    QLabel * leftLabel = new QLabel("Left Hand");
    leftLabel->setStyleSheet("font: 8pt \"Ubuntu\";");
    leftLabel->setAlignment(Qt::AlignCenter);
    leftLayout->addWidget(leftLabel);
    leftGraspWidget = new graspWidget(graspContainer, "left", "l_hand");
    leftGraspWidget->show();
    leftLayout->addWidget(leftGraspWidget);
    leftGrasp->setLayout(leftLayout);

    Qt::WindowFlags left_flags = leftGraspWidget->windowFlags();
    //flags |= Qt::WindowStaysOnTopHint;
    left_flags |= Qt::FramelessWindowHint;
    left_flags |= Qt::Dialog;
    graspContainer->setWindowFlags(left_flags);

    //set border color of left grasp widget
    QFrame* leftFrame = new QFrame(graspContainer);
    leftFrame->setLayout(leftLayout);
    leftFrame->setFrameStyle(QFrame::Panel| QFrame::Plain);
    leftFrame->setLineWidth(2);
    leftFrame->setObjectName("leftFrame");
    leftFrame->setStyleSheet("#leftFrame {color: yellow;}");
    graspContainer->layout()->addWidget(leftFrame); //adds graspwidgets as well

    //Right Grasp Widget
    QWidget * rightGrasp = new QWidget(graspContainer);
    QVBoxLayout * rightLayout = new QVBoxLayout();
    rightLayout->setSpacing(0);
    rightLayout->setMargin(0);
    rightLayout->setContentsMargins(0,0,0,0);
    QLabel * rightLabel = new QLabel("Right Hand");
    rightLabel->setStyleSheet("font: 8pt \"Ubuntu\";");
    rightLabel->setAlignment(Qt::AlignCenter);
    rightLayout->addWidget(rightLabel);
    rightGraspWidget = new graspWidget(graspContainer, "right", "r_hand");
    rightGraspWidget->show();
    rightLayout->addWidget(rightGraspWidget);
    rightGrasp->setLayout(rightLayout);

    Qt::WindowFlags right_flags = rightGraspWidget->windowFlags();
    //flags |= Qt::WindowStaysOnTopHint;
    right_flags |= Qt::FramelessWindowHint;
    right_flags |= Qt::Dialog;
    graspContainer->setWindowFlags(right_flags);

    //set border color of right grasp widget
    QFrame* rightFrame = new QFrame(graspContainer);
    rightFrame->setLayout(rightLayout);
    rightFrame->setFrameStyle(QFrame::WinPanel | QFrame::Plain);
    rightFrame->setLineWidth(2);
    rightFrame->setObjectName("rightFrame");
    rightFrame->setStyleSheet("#rightFrame {color: cyan;}");
    graspContainer->layout()->addWidget(rightFrame);

    graspFadeIn = new QPropertyAnimation(graspContainer, "windowOpacity");
    graspFadeIn->setEasingCurve(QEasingCurve::InOutQuad);
    graspFadeIn->setDuration(500);
    graspFadeIn->setStartValue(0.0);
    graspFadeIn->setEndValue(.8);

    graspFadeOut = new QPropertyAnimation(graspContainer, "windowOpacity");
    graspFadeOut->setEasingCurve(QEasingCurve::InOutQuad);
    graspFadeOut->setDuration(300);
    graspFadeOut->setStartValue(0.8);
    graspFadeOut->setEndValue(0.0);

    connect(graspFadeOut,SIGNAL(finished()),this,SLOT(hideGraspWidgets()));

    //send template tree to base3dview
    ((vigir_ocs::Base3DView*)views_list_["Top Left"])->getBaseContextMenu()->setTemplateTree(ui->template_widget->getTreeRoot());
    ((vigir_ocs::Base3DView*)views_list_["Top Right"])->getBaseContextMenu()->setTemplateTree(ui->template_widget->getTreeRoot());
    ((vigir_ocs::Base3DView*)views_list_["Bottom Left"])->getBaseContextMenu()->setTemplateTree(ui->template_widget->getTreeRoot());
    ((vigir_ocs::Base3DView*)views_list_["Bottom Right"])->getBaseContextMenu()->setTemplateTree(ui->template_widget->getTreeRoot());

    sidebar_toggle_ = new QPushButton(this);
    sidebar_toggle_->setStyleSheet("font: 9pt \"MS Shell Dlg 2\";background-color: rgb(0, 0, 0);color: rgb(108, 108, 108);border-color: rgb(0, 0, 0); ");
    sidebar_toggle_->setMaximumSize(25,25);

    QPixmap pix(icon_path_+"drawer.png");
    QIcon Btn(pix);
    sidebar_toggle_->setIcon(Btn);
    sidebar_toggle_->setIconSize(pix.rect().size() / 8);

    connect(sidebar_toggle_,SIGNAL(clicked()),this,SLOT(toggleSidebarVisibility()));

    //initialize behavior relay with notifications
    notification_container_ = new QWidget(this);
    notification_container_->setStyleSheet("background-color: rgb(30, 30, 30);color: rgb(108, 108, 108);border-color: rgb(0, 0, 0); ");
    notification_container_->setWindowOpacity(0);
    //notification_container_->setAttribute(Qt::WA_TranslucentBackground);
    //notification_container_->setStyleSheet("background:transparent;");
    notification_container_->setMinimumHeight(70);
    notification_container_->setMaximumHeight(70);
    notification_container_->setMaximumWidth(340);
    notification_container_->setMinimumWidth(340);
    notification_container_->adjustSize();
    notification_layout_ = new QVBoxLayout();
    notification_layout_->setMargin(0);
    notification_layout_->setSpacing(0);
    notification_container_->setLayout(notification_layout_);
    notification_container_->hide();

    behavior_relay_ = new BehaviorRelay();
    connect(behavior_relay_,SIGNAL(updateUI()),this,SLOT(updateBehaviorNotifications()));



    timer.start(100, this);

    //hide items in toolbar that are no longer needed
    ui->Tools->hide();
    ui->Navigation->hide();
    ui->Template->hide();


    //need local reference to ghost control to access some of its functionality for context menu
    snap_ghost_pub_ = n_.advertise<std_msgs::Bool>("/flor/ocs/snap_ghost_context",1,false);
    use_torso_pub_ = n_.advertise<std_msgs::Bool>("/flor/ocs/use_torso_context",1,false);
    //TODO, get this synced from ghost control ui
    use_torso_checked_ = false;

    // connect emergency stop button to glancehub
    stop_mapper_ = new QSignalMapper(this);
    connect(stop_mapper_,SIGNAL(mapped(int)),statusBar->getGlanceSbar(),SLOT(receiveModeChange(int)));

    //map all toggles button to their identifiers                              @todo - verify
    stop_mapper_->setMapping(((vigir_ocs::Base3DView*) views_list_["Top Left"]),1);//flor_control_msgs::FlorControlModeCommand::FLOR_STOP);

    //connect all buttons for mouse presses
    connect(((vigir_ocs::Base3DView*) views_list_["Top Left"]),SIGNAL(emergencyStop()),stop_mapper_,SLOT(map()));

    //Restore State
    QSettings settings("OCS", "main_view");
    this->restoreGeometry(settings.value("mainWindowGeometry").toByteArray());
    // create docks, toolbars, etc...
    //this->restoreState(settings.value("mainWindowState").toByteArray());

    ocs_sync_sub_ = n_.subscribe<flor_ocs_msgs::OCSSynchronize>( "/flor/ocs/synchronize", 5, &MainViewWidget::synchronizeToggleButtons, this );

    //useful to getting access to callbacks for context menu
    primary_view_ = (vigir_ocs::Base3DView*) views_list_["Top Left"];

    //create context menu and add to base3dview
    main_view_context_menu_ = new MainViewContextMenu(this);
}


void MainViewWidget::updateBehaviorNotifications()
{
    //show certain amount of notifications in 3d view
    int i=0;
    //old notifications may have been deleted, replace current notifications with top 3 from relay
    while (i < (int)behavior_relay_->getNotifications().size() && i < behavior_relay_->getMaxNotificationsShown())
    {
        BehaviorNotification* notification = behavior_relay_->getNotifications()[i];       
        notification->show();        
        notification_layout_->insertWidget(0,notification);//insert top down
        i++;
    }    
    //toggle visibility of behavior container in ui
    if(notification_layout_->count() == 0)
    {
        notification_container_->hide();
    }
    else
        notification_container_->show();
}

void MainViewWidget::setLidarSpinRate(double spin_rate)
{    
    //published in radians directly from widget
    std_msgs::Float64 msg;
    msg.data = spin_rate;
    lidar_spin_rate_pub_.publish(msg);
}

void MainViewWidget::toggleSidebarVisibility()
{
    if(ui->scrollArea->isVisible())
        ui->scrollArea->hide();
    else
        ui->scrollArea->show();
}

void MainViewWidget::changeCheckBoxState(QCheckBox* checkBox, Qt::CheckState state)
{
    //set checkbox state without calling callbacks
    checkBox->blockSignals(true);
    checkBox->setCheckState(state);
    checkBox->blockSignals(false);
}

void MainViewWidget::synchronizeToggleButtons(const flor_ocs_msgs::OCSSynchronize::ConstPtr &msg)
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

void MainViewWidget::modeCB(const flor_ocs_msgs::OCSControlMode::ConstPtr& msg)
{
    if(msg->manipulationMode != 0 || msg->manipulationMode != 1 || msg->manipulationMode != 2)
        return;
    controlModes = *msg;
    //update comboBox
    ui->modeBox->setCurrentIndex(controlModes.manipulationMode);
    //need to change object mode as well?

}

void MainViewWidget::closeEvent(QCloseEvent *event)
{
    QSettings settings("OCS", "main_view");
    settings.setValue("mainWindowGeometry", this->saveGeometry());    
}

void MainViewWidget::resizeEvent(QResizeEvent * event)
{    
    QSettings settings("OCS", "main_view");
    settings.setValue("mainWindowGeometry", this->saveGeometry());    
}

void MainViewWidget::moveEvent(QMoveEvent * event)
{    
    QSettings settings("OCS", "main_view");
    settings.setValue("mainWindowGeometry", this->saveGeometry());    
}




void MainViewWidget::hideGraspWidgets()
{
    graspContainer->hide();
}

//callback functions for context menu
void MainViewWidget::contextToggleWindow(int window)
{
    switch(window)
    {
    case WINDOW_JOYSTICK:
        if(!ui->joystickBtn->isChecked())
            ui->joystickBtn->setChecked(true);
        else
            ui->joystickBtn->setChecked(false);
        break;
    case WINDOW_JOINT_CONTROL:
        if(!ui->jointControlBtn->isChecked())
            ui->jointControlBtn->setChecked(true);
        else
            ui->jointControlBtn->setChecked(false);
        break;
    case WINDOW_BDI_PELVIS_POSE:
        if(!ui->pelvisControlBtn->isChecked())
            ui->pelvisControlBtn->setChecked(true);
        else
            ui->pelvisControlBtn->setChecked(false);
        break;
//    case WINDOW_FOOTSTEP_BASIC:
//        if(!ui->basicStepBtn->isChecked())
//            ui->basicStepBtn->setChecked(true);
//        else
//            ui->basicStepBtn->setChecked(false);
//        break;
    case WINDOW_FOOTSTEP_ADVANCED:
        if(!ui->stepBtn->isChecked())
            ui->stepBtn->setChecked(true);
        else
            ui->stepBtn->setChecked(false);
        break;
    case WINDOW_FOOTSTEP_PARAMETER:
        if(!ui->footstepParamBtn->isChecked())
            ui->footstepParamBtn->setChecked(true);
        else
            ui->footstepParamBtn->setChecked(false);
        break;
    case WINDOW_GHOST_CONFIG:
        if(!ui->ghostControlBtn->isChecked())
            ui->ghostControlBtn->setChecked(true);
        else
            ui->ghostControlBtn->setChecked(false);
        break;
    case WINDOW_POSITION_MODE:
        if(!ui->positionModeBtn->isChecked())
            ui->positionModeBtn->setChecked(true);
        else
            ui->positionModeBtn->setChecked(false);
        break;
    case WINDOW_PLANNER_CONFIG:
        if(!ui->plannerConfigBtn->isChecked())
            ui->plannerConfigBtn->setChecked(true);
        else
            ui->plannerConfigBtn->setChecked(false);
        break;
    }
}

void MainViewWidget::setCameraMode()
{
    ui->modeBox->setCurrentIndex(2);
    setManipulationMode(2);

}
void MainViewWidget::setWorldMode()
{
    ui->modeBox->setCurrentIndex(1);
    setManipulationMode(1);
}
void MainViewWidget::setObjectManipulationMode()
{
    ui->modeBox->setCurrentIndex(0);
    setManipulationMode(0);
}

void MainViewWidget::updateContextMenu()
{
    //change default checkable values of context items.. must as they are created with menu
    //main_view_context_menu_->setAllCheckable();

    //update context menu elements with checks
    main_view_context_menu_->setItemCheckState("Joystick",ui->joystickBtn->isChecked());
    main_view_context_menu_->setItemCheckState("Joint Control",ui->jointControlBtn->isChecked());
    main_view_context_menu_->setItemCheckState("Pelvis Pose",ui->pelvisControlBtn->isChecked());
    //main_view_context_menu_->setItemCheckState("Basic Footstep Interface",ui->basicStepBtn->isChecked());
    main_view_context_menu_->setItemCheckState("Advanced Footstep Interface",ui->stepBtn->isChecked());
    main_view_context_menu_->setItemCheckState("Footstep Parameter Control",ui->footstepParamBtn->isChecked());
    main_view_context_menu_->setItemCheckState("Ghost Control",ui->ghostControlBtn->isChecked());
    main_view_context_menu_->setItemCheckState("Position Mode",ui->positionModeBtn->isChecked());
    main_view_context_menu_->setItemCheckState("Planner Configuration",ui->plannerConfigBtn->isChecked());
    main_view_context_menu_->setItemCheckState("Use Torso",use_torso_checked_);

    switch(ui->modeBox->currentIndex())
    {
    case 0:
        main_view_context_menu_->setItemCheckState("Object",true);
        main_view_context_menu_->setItemCheckState("World",false);
        main_view_context_menu_->setItemCheckState("Camera",false);
        break;
    case 1:
        main_view_context_menu_->setItemCheckState("Object",false);
        main_view_context_menu_->setItemCheckState("World",true);
        main_view_context_menu_->setItemCheckState("Camera",false);
        break;
    case 2:
        main_view_context_menu_->setItemCheckState("Object",false);
        main_view_context_menu_->setItemCheckState("World",false);
        main_view_context_menu_->setItemCheckState("Camera",true);
        break;
    }    

}

//yes these next 3 functions are awful
//need to coordinate checkbox in ghost widget and main view context menu, also call use torso function
void MainViewWidget::useTorsoContextMenu()
{
    std_msgs::Bool msg;
    //message is only used as a signal,  TEMPORARY, need to make ghost manager
    snap_ghost_pub_.publish(msg);
}

//use_torso not synced in context menu
//void MainViewWidget::useTorsoChecked(std_msgs::BoolConstPtr & msg)
//{
//    use_torso_checked_ = msg->data;
//}

void MainViewWidget::snapGhostContextMenu()
{
    std_msgs::Bool msg;
    //message is only used as a signal,  TEMPORARY, need to make ghost manager
    snap_ghost_pub_.publish(msg);
}

void MainViewWidget::setManipulationMode(int mode)
{
    // update template joystick
    flor_ocs_msgs::OCSControlMode msg;
    msg.manipulationMode =  mode;
    mode_pub_.publish(msg);

    //notify ui
    NotificationSystem::Instance()->notifyPassive("Changed Interaction Mode");
}

void MainViewWidget::setObjectMode(int mode)
{
    flor_ocs_msgs::OCSControlMode msg;
    msg.objectMode =  mode;
    msg.manipulationMode = ui->modeBox->currentIndex();
    mode_pub_.publish(msg);
}

void MainViewWidget::graspWidgetToggle()
{
    if(!graspContainer->isVisible())
    {
        graspContainer->show();
        graspFadeIn->start();        

        //reset graphic on toggle button
        QPixmap pixmap(icon_path_+"graspDown.png");
        QIcon ButtonIcon(pixmap);
        grasp_toggle_button_->setIcon(ButtonIcon);
        grasp_toggle_button_->setIconSize(pixmap.rect().size()/5);
    }
    else // visible
    {
        graspFadeOut->start();

        //reset graphic on button
        QPixmap pixmap(icon_path_+"graspUp.png");
        QIcon ButtonIcon(pixmap);
        grasp_toggle_button_->setIcon(ButtonIcon);
        grasp_toggle_button_->setIconSize(pixmap.rect().size()/5);
    }
}

MainViewWidget::~MainViewWidget()
{
    delete ui;
    //delete ghost_control_widget_;
}

void MainViewWidget::timerEvent(QTimerEvent *event)
{
    grasp_toggle_button_->setGeometry(ui->view_stack_->geometry().bottomRight().x() - 60,ui->view_stack_->geometry().bottom() + 22,60,22);
    sidebar_toggle_->setGeometry(ui->view_stack_->geometry().topRight().x() - 25 ,ui->view_stack_->geometry().top() + 43,25,25);

    //top right 3/4 of width just offset from top
    notification_container_->setGeometry(ui->view_stack_->geometry().topRight().x() - notification_container_->geometry().width()/2 - ui->view_stack_->geometry().width()/4,
                                         ui->view_stack_->geometry().topRight().y() + notification_container_->geometry().height() - 15,
                                         notification_container_->geometry().width(),notification_container_->geometry().height());

    //must be global, as it is treated as dialog window
    graspContainer->setGeometry(ui->view_stack_->mapToGlobal(ui->view_stack_->geometry().bottomRight()).x() - graspContainer->geometry().width()/2 - ui->view_stack_->geometry().width()/2,
                                ui->view_stack_->mapToGlobal(ui->view_stack_->geometry().bottomRight()).y() - graspContainer->geometry().height(),
                                graspContainer->geometry().width(),graspContainer->geometry().height());

}


bool MainViewWidget::eventFilter( QObject * o, QEvent * e )
{
    if ( e->type() == QEvent::Wheel &&
         (qobject_cast<QAbstractSpinBox*>( o ) || qobject_cast<QAbstractSlider*>( o ) || qobject_cast<QComboBox*>( o )))
    {
        e->ignore();
        return true;
    }
    if( qobject_cast<QComboBox*>( o ) && qobject_cast<QComboBox*>( o ) == ui->footstepParamSetBox)
    {
        e->ignore();
        return true;
    }
    return QWidget::eventFilter( o, e );
}

void MainViewWidget::setupToolbar()
{        
    //set menu to popup a config widget for footstep Params
    QWidgetAction *wa = new QWidgetAction(0);
    wa->setDefaultWidget(footstep_configure_widget_);
    footstep_menu_.addAction(wa);
    //associate button with menu
    ui->footstepConfigBtn->setMenu(&footstep_menu_);
    //need to install event filter for widget positioning
    footstep_menu_.installEventFilter(this);

    connect(ui->footstepConfigBtn,SIGNAL(clicked()),this,SLOT(toggleFootstepConfig()));

    //place graphic on joystick toggle
    loadButtonIconAndStyle(ui->joystickBtn, "controllerIcon.png");
    loadButtonIconAndStyle(ui->jointControlBtn, "jointIcon.png");
    loadButtonIconAndStyle(ui->pelvisControlBtn, "pelvis.png");
    loadButtonIconAndStyle(ui->ghostControlBtn, "ghostIcon.png");
    loadButtonIconAndStyle(ui->positionModeBtn, "positionIcon.png");
    //loadButtonIconAndStyle(ui->basicStepBtn, "footBasicIcon.png");
    loadButtonIconAndStyle(ui->stepBtn, "footAdvancedIcon.png");
    loadButtonIconAndStyle(ui->footstepParamBtn, "footParamIcon.png");
    loadButtonIconAndStyle(ui->footstepConfigBtn,"configIcon.png");

    //use signalmapper to avoid having one function for each one of the toggle buttons
    toggle_mapper_ = new QSignalMapper(this);
    connect(toggle_mapper_,SIGNAL(mapped(int)),this,SLOT(toggleWindow(int)));

    //map all toggles button to their identifiers
    toggle_mapper_->setMapping(this->ui->joystickBtn,WINDOW_JOYSTICK);
    toggle_mapper_->setMapping(this->ui->jointControlBtn,WINDOW_JOINT_CONTROL);
    toggle_mapper_->setMapping(this->ui->pelvisControlBtn,WINDOW_BDI_PELVIS_POSE);
    //toggle_mapper_->setMapping(this->ui->basicStepBtn,WINDOW_FOOTSTEP_BASIC);
    toggle_mapper_->setMapping(this->ui->stepBtn,WINDOW_FOOTSTEP_ADVANCED);
    toggle_mapper_->setMapping(this->ui->footstepParamBtn,WINDOW_FOOTSTEP_PARAMETER);
    toggle_mapper_->setMapping(this->ui->ghostControlBtn,WINDOW_GHOST_CONFIG);
    toggle_mapper_->setMapping(this->ui->positionModeBtn,WINDOW_POSITION_MODE);
    toggle_mapper_->setMapping(this->ui->plannerConfigBtn,WINDOW_PLANNER_CONFIG);

    //connect all buttons for mouse presses
    connect(ui->joystickBtn,SIGNAL(toggled(bool)),toggle_mapper_,SLOT(map()));
    connect(ui->jointControlBtn,SIGNAL(toggled(bool)),toggle_mapper_,SLOT(map()));
    connect(ui->pelvisControlBtn,SIGNAL(toggled(bool)),toggle_mapper_,SLOT(map()));
    //connect(ui->basicStepBtn,SIGNAL(toggled(bool)),toggle_mapper_,SLOT(map()));
    connect(ui->stepBtn,SIGNAL(toggled(bool)),toggle_mapper_,SLOT(map()));
    connect(ui->footstepParamBtn,SIGNAL(toggled(bool)),toggle_mapper_,SLOT(map()));
    connect(ui->ghostControlBtn,SIGNAL(toggled(bool)),toggle_mapper_,SLOT(map()));
    connect(ui->positionModeBtn,SIGNAL(toggled(bool)),toggle_mapper_,SLOT(map()));
    connect(ui->plannerConfigBtn,SIGNAL(toggled(bool)),toggle_mapper_,SLOT(map()));

    //combo box for manipulation modes
    //connect(ui->modeBox,SIGNAL(currentIndexChanged(int)),joystick,SLOT(setManipulationMode(int)));
    ui->modeBox->addItem(QString("Object"));
    ui->modeBox->addItem(QString("World"));
    ui->modeBox->addItem(QString("Camera"));

    // workaround to be able to use images from stylesheet without knowing the path in advance
    QString stylesheet = ui->modeBox->styleSheet() + "\n" +
            "QComboBox::down-arrow {\n" +
            " image: url(" + icon_path_ + "down_arrow.png" + ");\n" +
            "}";
    ui->modeBox->setStyleSheet(stylesheet);

    //set for footstep param box
    stylesheet = ui->footstepParamSetBox->styleSheet() + "\n" +
            "QComboBox::down-arrow {\n" +
            " image: url(" + icon_path_ + "down_arrow.png" + ");\n" +
            "}";
    ui->footstepParamSetBox->setStyleSheet(stylesheet);


    //allow combo boxes to send messages to joystick
    connect(ui->modeBox,SIGNAL(currentIndexChanged(int)),this,SLOT(setManipulationMode(int)));

    //publisher for joystick modes
    mode_pub_ = n_.advertise<flor_ocs_msgs::OCSControlMode>("/flor/ocs/control_modes",1,false);
    //need to subscribe to stay in sync with modes
    mode_sub_ = n_.subscribe<flor_ocs_msgs::OCSControlMode>("/flor/ocs/control_modes",1, &MainViewWidget::modeCB,this);

    connect(ui->footstepParamSetBox,SIGNAL(currentIndexChanged(QString)),((vigir_ocs::Base3DView*)views_list_["Top Left"])->getFootstepVisManager(),SLOT(setFootstepParameterSet(QString)));
    connect(((vigir_ocs::Base3DView*)views_list_["Top Left"])->getFootstepVisManager(),SIGNAL(populateFootstepParameterSetBox(std::vector<std::string>)),this,SLOT(populateFootstepParameterSetBox(std::vector<std::string>)));
    connect(((vigir_ocs::Base3DView*)views_list_["Top Left"])->getFootstepVisManager(),SIGNAL(setFootstepParameterSetBox(std::string)),this,SLOT(setFootstepParameterSetBox(std::string)));

}

void MainViewWidget::toggleFootstepConfig()
{
    ui->footstepConfigBtn->showMenu();
}

void MainViewWidget::loadButtonIconAndStyle(QPushButton* btn, QString image_name)
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

void MainViewWidget::toggleWindow(int window)
{
//    if(window == WINDOW_GHOST_CONFIG)
//    {
//        if(ghost_control_widget_->isVisible())
//            ghost_control_widget_->hide();
//        else
//            ghost_control_widget_->show();
//    }
//    else
//    {
        std_msgs::Int8 cmd;
        cmd.data = ((QPushButton*)toggle_mapper_->mapping(window))->isChecked() ? window : -window;
        window_control_pub_.publish(cmd);
   // }


}

void MainViewWidget::oneViewToggle()
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
            ((vigir_ocs::Base3DView*)iter->second)->updateRenderMask(true);
        else
            ((vigir_ocs::Base3DView*)iter->second)->updateRenderMask(false);
    }
}

void MainViewWidget::fourViewToggle()
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
        ((vigir_ocs::Base3DView*)iter->second)->updateRenderMask(true);
}

void MainViewWidget::ft_sensorToggled(bool toggled){
    ((vigir_ocs::PerspectiveView*)views_list_["Top Left"])->ft_sensorToggled(toggled);
}

void MainViewWidget::zero_leftPressed(){
    if(ft_zero_pub_){
        std_msgs::Int8 msg;
        msg.data = -1;
        ft_zero_pub_.publish(msg);
    }
    else{
        ROS_ERROR("No publisher for Zero F/T Wrench");
    }
}
void MainViewWidget::zero_rightPressed(){
    if(ft_zero_pub_){
        std_msgs::Int8 msg;
        msg.data = 1;
        ft_zero_pub_.publish(msg);
    }
    else{
        ROS_ERROR("No publisher for Zero F/T Wrench");
    }
}

//void MainViewWidget::processNewKeyEvent(const flor_ocs_msgs::OCSKeyEvent::ConstPtr &key_event)
//{
//    // store key state
//    if(key_event->state)
//        keys_pressed_list_.push_back(key_event->keycode);
//    else
//        keys_pressed_list_.erase(std::remove(keys_pressed_list_.begin(), keys_pressed_list_.end(), key_event->keycode), keys_pressed_list_.end());

//    // process hotkeys
//    std::vector<int>::iterator key_is_pressed;

//    // ctrl
//    //key_is_pressed = std::find(keys_pressed_list_.begin(), keys_pressed_list_.end(), 37);

//}

//ADD THIS FUNCTION TO THE INDIVIDUAL WINDOWS
//SETUP PUBLISHER FOR TOGGLE

void MainViewWidget::processWindowControl(const std_msgs::Int8::ConstPtr &visible)
{
    char visibility = visible->data;

    switch(abs(visibility))
    {
        case HIDE_ALL_WINDOWS:
            ui->joystickBtn->setChecked(false);
            ui->jointControlBtn->setChecked(false);
            ui->pelvisControlBtn->setChecked(false);
            //ui->basicStepBtn->setChecked(false);
            ui->stepBtn->setChecked(false);
            ui->footstepParamBtn->setChecked(false);
            ui->ghostControlBtn->setChecked(false);
            ui->positionModeBtn->setChecked(false);
            ui->plannerConfigBtn->setChecked(false);
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
        case WINDOW_BDI_PELVIS_POSE:
            ui->pelvisControlBtn->setChecked(visibility > 0 ? true : false);
            break;
        case WINDOW_GHOST_CONFIG:
            ui->ghostControlBtn->setChecked(visibility > 0 ? true : false);
            break;
        case WINDOW_PLANNER_CONFIG:
            ui->plannerConfigBtn->setChecked(visibility > 0 ? true : false);
            break;
        case WINDOW_JOINT_CONTROL:
            ui->jointControlBtn->setChecked(visibility > 0 ? true : false);
            break;
        case WINDOW_POSITION_MODE:
            ui->positionModeBtn->setChecked(visibility > 0 ? true : false);
            break;
        case WINDOW_CONTROL_MODE:
            //ui->basicStepBtn->setChecked(visibility > 0 ? true : false);
            break;
        case WINDOW_GLANCE_HUB:
            //ui->basicStepBtn->setChecked(visibility > 0 ? true : false);
            break;
        case WINDOW_STATUS_WINDOW:
            //ui->basicStepBtn->setChecked(visibility > 0 ? true : false);
            break;
        case WINDOW_SYSTEM_COMMAND:
            //ui->basicStepBtn->setChecked(visibility > 0 ? true : false);
            break;
        case WINDOW_JOYSTICK:
            ui->joystickBtn->setChecked(visibility > 0 ? true : false);
            break;
        case WINDOW_SYSTEM_LOG:
            //ui->basicStepBtn->setChecked(visibility > 0 ? true : false);
            break;
        case WINDOW_JOINT_STATUS:
            //ui->basicStepBtn->setChecked(visibility > 0 ? true : false);
            break;
        default:
            break;
    }
}

void MainViewWidget::populateFootstepParameterSetBox(std::vector<std::string> parameter_sets)
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

void MainViewWidget::setFootstepParameterSetBox(std::string parameter_set)
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
