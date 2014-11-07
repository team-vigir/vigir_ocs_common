#include "main_camera_view_widget.h"
#include "ui_main_camera_view_widget.h"
#include "ui/template_loader_widget.h"
#include "camera_view_widget.h"
#include "base_3d_view.h"
#include <ros/package.h>

#include <rviz/displays_panel.h>
#include "main_camera_context_menu.h"

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
            QObject::connect(ui->footstep_pose, SIGNAL(pressed()), ((CameraViewWidget*)iter->second)->getCameraView(), SLOT(defineFootstepGoal()));
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
            QObject::connect(ui->robot_joint_markers,SIGNAL(toggled(bool)), ((CameraViewWidget*)iter->second)->getCameraView(), SLOT(robotJointMarkerToggled(bool)));
            QObject::connect(ui->robot_occlusion_rendering,SIGNAL(toggled(bool)), ((CameraViewWidget*)iter->second)->getCameraView(), SLOT(robotOcclusionToggled(bool)));
            QObject::connect(ui->notification_system,SIGNAL(toggled(bool)), ((CameraViewWidget*)iter->second)->getCameraView(), SLOT(notificationSystemToggled(bool)));
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

    connect(ui->pitch, SIGNAL(sliderPressed()), this, SLOT(lockPitchUpdates()));
    connect(ui->pitch, SIGNAL(sliderReleased()), this, SLOT(sendPitch()));

    views_initialized_ = 0;

    fourViewToggle();

    key_event_sub_ = nh_.subscribe<flor_ocs_msgs::OCSKeyEvent>( "/flor/ocs/key_event", 5, &MainCameraViewWidget::processNewKeyEvent, this );
    neck_pos_sub_ = nh_.subscribe<std_msgs::Float32> ( "/flor/neck_controller/current_position" , 2, &MainCameraViewWidget::updatePitch, this );

    //send template list to views for context menu
    ((vigir_ocs::Base3DView*)((CameraViewWidget*)views_list_["Top Left"])->getCameraView())->getBaseContextMenu()->setTemplateTree(ui->template_widget->getTreeRoot());
    ((CameraViewWidget*)views_list_["Top Right"])->getCameraView()->getBaseContextMenu()->setTemplateTree(ui->template_widget->getTreeRoot());
    ((CameraViewWidget*)views_list_["Bottom Left"])->getCameraView()->getBaseContextMenu()->setTemplateTree(ui->template_widget->getTreeRoot());
    ((CameraViewWidget*)views_list_["Bottom Right"])->getCameraView()->getBaseContextMenu()->setTemplateTree(ui->template_widget->getTreeRoot());

    //hide sidebar elements that are no longer necessary
    ui->Tools->hide();
    ui->Template->hide();
    ui->Navigation->hide();

    //add status bar
    statusBar = new StatusBar(this);
    //connect for position text and fps
    connect(((vigir_ocs::Base3DView*) ((CameraViewWidget*)views_list_["Top Left"])->getCameraView()),SIGNAL(sendPositionText(QString)),statusBar,SLOT(receivePositionText(QString)));
    connect(((vigir_ocs::Base3DView*) ((CameraViewWidget*)views_list_["Top Left"])->getCameraView()),SIGNAL(sendFPS(int)),statusBar,SLOT(receiveFPS(int)));
    ui->statusLayout->addWidget(statusBar);

    // connect emergency stop button to glancehub
    stop_mapper_ = new QSignalMapper(this);
    connect(stop_mapper_,SIGNAL(mapped(int)),statusBar->getGlanceSbar(),SLOT(receiveModeChange(int)));

    //map all toggles button to their identifiers
    ROS_WARN(" Hard coding a STOP with index 1"); // @todo - verify this setup
    stop_mapper_->setMapping(((vigir_ocs::Base3DView*) ((CameraViewWidget*)views_list_["Top Left"])->getCameraView()),1); // flor_control_msgs::FlorControlModeCommand::FLOR_STOP);

    //connect all buttons for mouse presses
    connect(((vigir_ocs::Base3DView*) ((CameraViewWidget*)views_list_["Top Left"])->getCameraView()),SIGNAL(emergencyStop()),stop_mapper_,SLOT(map()));

    //Restore State
    QSettings settings("OCS", "camera_view");
    this->restoreGeometry(settings.value("mainWindowGeometry").toByteArray());
    // create docks, toolbars, etc...
    //this->restoreState(settings.value("mainWindowState").toByteArray());

    sidebar_toggle_ = new QPushButton(this);
    sidebar_toggle_->setStyleSheet("font: 9pt \"MS Shell Dlg 2\";background-color: rgb(0, 0, 0);color: rgb(108, 108, 108);border-color: rgb(0, 0, 0); ");
    sidebar_toggle_->setMaximumSize(25,25);

    QPixmap pix(icon_path_+"drawer.png");
    QIcon Btn(pix);
    sidebar_toggle_->setIcon(Btn);
    sidebar_toggle_->setIconSize(pix.rect().size() / 8);

    connect(sidebar_toggle_,SIGNAL(clicked()),this,SLOT(toggleSidebarVisibility()));

    //synchronize toggles
    ocs_sync_sub_ = nh_.subscribe<flor_ocs_msgs::OCSSynchronize>( "/flor/ocs/synchronize", 5, &MainCameraViewWidget::synchronizeToggleButtons, this );

    //disable joint markers by default
    ui->robot_joint_markers->setCheckState(Qt::Unchecked);

    //build and add Context menu to base3dview
    main_camera_context_menu_ = new MainCameraContextMenu(this);

    timer.start(100, this);
}

MainCameraViewWidget::~MainCameraViewWidget()
{
    delete ui;
}

void MainCameraViewWidget::timerEvent(QTimerEvent *event)
{
    sidebar_toggle_->setGeometry(ui->view_stack_->geometry().topRight().x() - 25 ,ui->view_stack_->geometry().top() + 40,25,25);
}

void MainCameraViewWidget::changeCheckBoxState(QCheckBox* checkBox, Qt::CheckState state)
{
    //set checkbox state without calling callbacks
    checkBox->blockSignals(true);
    checkBox->setCheckState(state);
    checkBox->blockSignals(false);
}

void MainCameraViewWidget::synchronizeToggleButtons(const flor_ocs_msgs::OCSSynchronize::ConstPtr &msg)
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

void MainCameraViewWidget::toggleSidebarVisibility()
{
    if(ui->scrollArea->isVisible())
        ui->scrollArea->hide();
    else
        ui->scrollArea->show();
}

void MainCameraViewWidget::closeEvent(QCloseEvent *event)
{
    QSettings settings("OCS", "camera_view");
    settings.setValue("mainWindowGeometry", this->saveGeometry());
}

void MainCameraViewWidget::resizeEvent(QResizeEvent * event)
{    
    QSettings settings("OCS", "camera_view");
    settings.setValue("mainWindowGeometry", this->saveGeometry());    
}

void MainCameraViewWidget::moveEvent(QMoveEvent * event)
{    
    QSettings settings("OCS", "camera_view");
    settings.setValue("mainWindowGeometry", this->saveGeometry());    
}

//void MainCameraViewWidget::addContextMenu()
//{
//    //can tell context menu to add a separator when this item is added
//    contextMenuItem * separator = new contextMenuItem();
//    separator->name = "Separator";

//    vigir_ocs::Base3DView::makeContextChild("Request Point Cloud",boost::bind(&vigir_ocs::Base3DView::publishPointCloudWorldRequest,((vigir_ocs::Base3DView*) ((CameraViewWidget*)views_list_["Top Left"])->getCameraView())), NULL, contextMenuElements);

//    contextMenuElements.push_back(separator);

//    contextMenuItem * systemCommands = vigir_ocs::Base3DView::makeContextParent("System Commands", contextMenuElements);

//    vigir_ocs::Base3DView::makeContextChild("Reset World Model",boost::bind(&MainCameraViewWidget::systemCommandContext,this, "reset"), systemCommands, contextMenuElements);
//    vigir_ocs::Base3DView::makeContextChild("Save Octomap",boost::bind(&MainCameraViewWidget::systemCommandContext,this,"save_octomap"), systemCommands, contextMenuElements);
//    vigir_ocs::Base3DView::makeContextChild("Save Pointcloud",boost::bind(&MainCameraViewWidget::systemCommandContext,this,"save_pointcloud"), systemCommands, contextMenuElements);
//    vigir_ocs::Base3DView::makeContextChild("Save Image Head",boost::bind(&MainCameraViewWidget::systemCommandContext,this,"save_image_left_eye"), systemCommands, contextMenuElements);
//    vigir_ocs::Base3DView::makeContextChild("Save Left Hand Image",boost::bind(&MainCameraViewWidget::systemCommandContext,this,"save_image_left_hand"), systemCommands, contextMenuElements);
//    vigir_ocs::Base3DView::makeContextChild("Save Right Hand Image",boost::bind(&MainCameraViewWidget::systemCommandContext,this,"save_image_right_hand"), systemCommands, contextMenuElements);

//    //add all context menu items to each view
//    for(int i=0;i<contextMenuElements.size();i++)
//    {
//        ((vigir_ocs::Base3DView*) ((CameraViewWidget*)views_list_["Top Left"])->getCameraView())->addToContextVector(contextMenuElements[i]);
//        ((vigir_ocs::Base3DView*) ((CameraViewWidget*)views_list_["Top Right"])->getCameraView())->addToContextVector(contextMenuElements[i]);
//        ((vigir_ocs::Base3DView*) ((CameraViewWidget*)views_list_["Bottom Left"])->getCameraView())->addToContextVector(contextMenuElements[i]);
//        ((vigir_ocs::Base3DView*) ((CameraViewWidget*)views_list_["Bottom Right"])->getCameraView())->addToContextVector(contextMenuElements[i]);
//    }
//}

//callback function for context menu
//void MainCameraViewWidget::systemCommandContext(std::string command)
//{
//    sysCmdMsg.data = command;
//    sys_command_pub_.publish(sysCmdMsg);
//}

void MainCameraViewWidget::lockPitchUpdates()
{
    lock_pitch_slider_ = true;
}

void MainCameraViewWidget::sendPitch()
{
    lock_pitch_slider_ = false;

    int value = ui->pitch->sliderPosition();

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

void MainCameraViewWidget::updatePitch( const std_msgs::Float32::ConstPtr &pitch)
{
    if(!lock_pitch_slider_)
    {
        ((CameraViewWidget*)views_list_["Top Left"])->updateCurrentPitch((int)(pitch->data/0.0174532925));
        ui->pitch->setValue((int)(pitch->data/0.0174532925));
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
    if(key_event->key == 13 && key_event->state && ctrl_is_pressed) // '4' - get single image in the main view
        ((CameraViewWidget*)views_list_["Top Left"])->getCameraView()->requestSingleFeedImage();
    else if(key_event->key == 14 && key_event->state && ctrl_is_pressed) // '5' - set main view to 5 fps
    {
        ((CameraViewWidget*)views_list_["Top Left"])->imageFeedSliderChanged(5);
        ((CameraViewWidget*)views_list_["Top Left"])->imageFeedSliderReleased();
    }
    else if(key_event->key == 15 && key_event->state && ctrl_is_pressed) // '6' - close selected
    {
        ((CameraViewWidget*)views_list_["Top Left"])->getCameraView()->closeSelectedArea();
    }
}
