#include "main_view_widget.h"
#include "ui_main_view_widget.h"


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
    key_event_sub_ = n_.subscribe<flor_ocs_msgs::OCSKeyEvent>( "/flor/ocs/key_event", 5, &MainViewWidget::processNewKeyEvent, this );

    QHBoxLayout* aux_layout;

    // setup default views
    views_list["Top Left"] = new vigir_ocs::PerspectiveView(NULL, "/world", "MainView");
    views_list["Top Right"] = new vigir_ocs::OrthoView(((vigir_ocs::PerspectiveView*)views_list["Top Left"]),"/pelvis", "MainView"); //views_list["Top Right"] = new vigir_ocs::OrthoView();
    views_list["Bottom Left"] = new vigir_ocs::OrthoView(((vigir_ocs::PerspectiveView*)views_list["Top Left"]),"/pelvis", "MainView"); //views_list["Bottom Left"] = new vigir_ocs::OrthoView();
    views_list["Bottom Right"] = new vigir_ocs::OrthoView(((vigir_ocs::PerspectiveView*)views_list["Top Left"]),"/pelvis", "MainView"); //views_list["Bottom Right"] = new vigir_ocs::OrthoView();

    aux_layout = new QHBoxLayout();
    aux_layout->setMargin(0);
    aux_layout->addWidget(views_list["Top Left"]);
    ui->center_parent_->setLayout(aux_layout);

    aux_layout = new QHBoxLayout();
    aux_layout->setMargin(0);
    aux_layout->addWidget(views_list["Top Right"]);
    ui->top_right_parent_->setLayout(aux_layout);

    aux_layout = new QHBoxLayout();
    aux_layout->setMargin(0);
    aux_layout->addWidget(views_list["Bottom Left"]);
    ui->bottom_left_parent_->setLayout(aux_layout);

    aux_layout = new QHBoxLayout();
    aux_layout->setMargin(0);
    aux_layout->addWidget(views_list["Bottom Right"]);
    ui->bottom_right_parent_->setLayout(aux_layout);

    ((vigir_ocs::OrthoView*)views_list["Top Right"])->setViewPlane("XY");
    ((vigir_ocs::OrthoView*)views_list["Bottom Left"])->setViewPlane("XZ");
    ((vigir_ocs::OrthoView*)views_list["Bottom Right"])->setViewPlane("YZ");

    std::map<std::string, QWidget*>::iterator iter;

    for (iter = views_list.begin(); iter != views_list.end(); ++iter)
    {
        // only need to connect to the main rviz instance (maybe not all of them?
        if(iter->second == views_list["Top Left"])
        {
            ((vigir_ocs::Base3DView*)iter->second)->updateRenderMask(true);

            ((vigir_ocs::Base3DView*)iter->second)->simulationRobotToggled(true);
            ((vigir_ocs::Base3DView*)iter->second)->simulationRobotToggled(false);

            // connect UI to perspective functions
            QObject::connect(ui->camera_tool, SIGNAL(toggled(bool)), iter->second, SLOT(cameraToggled(bool)));
            QObject::connect(ui->footstep_planning, SIGNAL(toggled(bool)), iter->second, SLOT(footstepPlanningToggled(bool)));
            QObject::connect(ui->footstep_pose_walk, SIGNAL(pressed()), iter->second, SLOT(defineWalkPosePressed()));
            QObject::connect(ui->footstep_pose_step, SIGNAL(pressed()), iter->second, SLOT(defineStepPosePressed()));
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
        }
        else
        {
            ((vigir_ocs::Base3DView*)iter->second)->updateRenderMask(false);
        }
    }

    std::string ip = ros::package::getPath("vigir_ocs_main_view")+"/icons/";
    icon_path_ = QString(ip.c_str());

    //contains both view buttons for alignment
    position_widget_ = new QWidget(views_list["Top Left"]);
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
    displays_panel->initialize( ((vigir_ocs::PerspectiveView*)views_list["Top Left"])->getVisualizationManager());

    //rviz::ViewsPanel* views_panel = new rviz::ViewsPanel(this);
    //views_panel->setViewManager(((vigir_ocs::PerspectiveView*)views_list["Top Left"])->getVisualizationManager()->getViewManager());

    QVBoxLayout* displays_layout = new QVBoxLayout();
    displays_layout->setMargin(0);
    displays_layout->addWidget(displays_panel);
    //displays_layout->addWidget(views_panel);
    ui->rviz_options->setLayout(displays_layout);

    QObject::connect(ui->ft_sensor, SIGNAL(toggled(bool)), this, SLOT(ft_sensorToggled(bool)));
    QObject::connect(ui->zero_left, SIGNAL(pressed()), this, SLOT(zero_leftPressed()));
    QObject::connect(ui->zero_right, SIGNAL(pressed()), this, SLOT(zero_rightPressed()));

    //reset for the hand f/t sensors
    ft_zero_pub_ = n_.advertise<std_msgs::Int8>("/flor/controller/zero_hand_wrench",1,false);

    //publisher and subscriber for window visibility control
    window_control_sub_ = n_.subscribe<std_msgs::Int8>( "/flor/ocs/window_control", 5, &MainViewWidget::processWindowControl, this );
    window_control_pub_ = n_.advertise<std_msgs::Int8>( "/flor/ocs/window_control", 1, false);

    // setup all buttons/icons in the toolbar
    setupToolbar();

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


    //allow combo boxes to send messages to joystick
    connect(ui->modeBox,SIGNAL(currentIndexChanged(int)),this,SLOT(setManipulationMode(int)));

    //publisher for joystick modes
    joystick_pub_ = n_.advertise<flor_ocs_msgs::OCSJoystick>("/flor/ocs/joystick",1,false);
    //publisher for the interactive marker mode
    interactive_marker_mode_pub_ = n_.advertise<std_msgs::Int8>("/flor/ocs/interactive_marker_server/set_mode",1,false);

    statusBar = new StatusBar();

    //connect view to update position data
    connect(views_list["Top Left"],SIGNAL(sendPositionText(QString)),statusBar,SLOT(receivePositionText(QString)));

    ui->statusLayout->addWidget(statusBar);
    grasp_toggle_button_ = new QPushButton("Grasp",this);

    grasp_toggle_button_->setStyleSheet("font: 9pt \"MS Shell Dlg 2\";background-color: rgb(0, 0, 0);color: rgb(108, 108, 108);border-color: rgb(0, 0, 0); ");
    grasp_toggle_button_->setMaximumSize(68,22);
    grasp_toggle_button_->adjustSize();

    QPixmap pixmap(icon_path_+"up_arrow_white.png");
    QIcon ButtonIcon(pixmap);
    grasp_toggle_button_->setIcon(ButtonIcon);
    grasp_toggle_button_->setIconSize(pixmap.rect().size()/10);

    connect(grasp_toggle_button_, SIGNAL(clicked()), this, SLOT(graspWidgetToggle()));

    //put two grasp widgets into container to have same focus
    graspContainer = new QWidget(this);
    QHBoxLayout * graspLayout = new QHBoxLayout();

    graspLayout->setSpacing(10);
    graspLayout->setContentsMargins(0,0,0,0);
    graspContainer->setLayout(graspLayout);
    graspContainer->hide();

    // try to load grasp environment variables
    std::string env_left_hand = getenv("FLOR_LEFT_HAND_TYPE");
    if(env_left_hand.find("irobot") != std::string::npos || env_left_hand.find("sandia") != std::string::npos)
    {
        QWidget * leftGrasp = new QWidget(graspContainer);
        QVBoxLayout * leftLayout = new QVBoxLayout();
        QLabel * leftLabel = new QLabel("Left Hand");
        leftLabel->setAlignment(Qt::AlignCenter);
        leftLayout->addWidget(leftLabel);

        if(env_left_hand.find("irobot") != std::string::npos)
            leftGraspWidget = new graspWidget(graspContainer, "left", "irobot");
        else if(env_left_hand.find("sandia") != std::string::npos)
            leftGraspWidget = new graspWidget(graspContainer, "left", "sandia");
        leftGraspWidget->show();
        leftLayout->addWidget(leftGraspWidget);
        leftGrasp->setLayout(leftLayout);

        Qt::WindowFlags flags = leftGraspWidget->windowFlags();
        //flags |= Qt::WindowStaysOnTopHint;
        flags |= Qt::FramelessWindowHint;
        flags |= Qt::Dialog;
        graspContainer->setWindowFlags(flags);

        //set border color of left grasp widget
        QFrame* leftFrame = new QFrame(graspContainer);
        leftFrame->setLayout(leftLayout);
        leftFrame->setFrameStyle(QFrame::Panel| QFrame::Plain);
        leftFrame->setLineWidth(2);
        leftFrame->setObjectName("leftFrame");
        leftFrame->setStyleSheet("#leftFrame {color: yellow;}");
        graspContainer->layout()->addWidget(leftFrame); //adds graspwidgets as well
    }

    std::string env_right_hand = getenv("FLOR_RIGHT_HAND_TYPE");
    if(env_right_hand.find("irobot") != std::string::npos || env_right_hand.find("sandia") != std::string::npos)
    {
        QWidget * rightGrasp = new QWidget(graspContainer);
        QVBoxLayout * rightLayout = new QVBoxLayout();
        QLabel * rightLabel = new QLabel("Right Hand");
        rightLabel->setAlignment(Qt::AlignCenter);
        rightLayout->addWidget(rightLabel);
        if(env_right_hand.find("irobot") != std::string::npos)
            rightGraspWidget = new graspWidget(graspContainer, "right", "irobot");
        else if(env_right_hand.find("sandia") != std::string::npos)
            rightGraspWidget = new graspWidget(graspContainer, "right", "sandia");
        rightGraspWidget->show();
        rightLayout->addWidget(rightGraspWidget);
        rightGrasp->setLayout(rightLayout);

        Qt::WindowFlags flags = rightGraspWidget->windowFlags();
        //flags |= Qt::WindowStaysOnTopHint;
        flags |= Qt::FramelessWindowHint;
        flags |= Qt::Dialog;
        graspContainer->setWindowFlags(flags);

        //set border color of right grasp widget
        QFrame* rightFrame = new QFrame(graspContainer);
        rightFrame->setLayout(rightLayout);
        rightFrame->setFrameStyle(QFrame::WinPanel | QFrame::Plain);
        rightFrame->setLineWidth(2);
        rightFrame->setObjectName("rightFrame");
        rightFrame->setStyleSheet("#rightFrame {color: cyan;}");
        graspContainer->layout()->addWidget(rightFrame);
    }

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

    sys_command_pub_ = n_.advertise<std_msgs::String>("/syscommand",1,false);

    //send template tree to base3dview
    ((vigir_ocs::Base3DView*)views_list["Top Left"])->setTemplateTree(ui->template_widget->getTreeRoot());
    ((vigir_ocs::Base3DView*)views_list["Top Right"])->setTemplateTree(ui->template_widget->getTreeRoot());
    ((vigir_ocs::Base3DView*)views_list["Bottom Left"])->setTemplateTree(ui->template_widget->getTreeRoot());
    ((vigir_ocs::Base3DView*)views_list["Bottom Right"])->setTemplateTree(ui->template_widget->getTreeRoot());

    addContextMenu();

    connect(((vigir_ocs::Base3DView*) views_list["Top Left"]),SIGNAL(updateMainViewItems()),this,SLOT(updateContextMenu()));

    timer.start(100, this);

    //hide items in toolbar that are no longer needed
    ui->Tools->hide();
    ui->Navigation->hide();
    ui->Template->hide();

    // create a publisher to add templates
    template_add_pub_   = n_.advertise<flor_ocs_msgs::OCSTemplateAdd>( "/template/add", 1, false );
    startingPosition = new QVector3D(0,0,0);
    startingRotation = new QQuaternion(0,0,0,0);

    elapsedTimer = new QElapsedTimer();
    //accounts for current challenge in vector
    challengeCount = 0;
    defineTransforms();

    time_t rawtime;
    struct tm * timeinfo;
    char buffer[80];

    time (&rawtime);
    timeinfo = localtime(&rawtime);

    strftime(buffer,80,"%d-%m-%Y %I:%M:%S",timeinfo);
    //std::string str(buffer);
    QString str(buffer);

    //make file with current time as name
    results = new QFile("ManipulationResults" + str);
    results->open(QIODevice::WriteOnly | QIODevice::Text);

    template_list_sub = n_.subscribe<flor_ocs_msgs::OCSTemplateList>( "list", 1, &MainViewWidget::templateListCb ,this );
}

void MainViewWidget::templateListCb(const flor_ocs_msgs::OCSTemplateList::ConstPtr& msg)
{
    temList = *msg;
}

//initializing 10 predefined transforms for templates to be placed. also ten different templates by name
void MainViewWidget::defineTransforms()
{
    //stash all into corresponding vector
    QVector3D * position1 = new QVector3D(5,6,1);
    QQuaternion * rotation1 = new QQuaternion(.5,20,50,70);
    templatePositions.push_back(position1);
    templateRotations.push_back(rotation1);
    templateNames.push_back("debris/2x4x36.mesh");

    QVector3D * position2 = new QVector3D(4,7,2);
    QQuaternion * rotation2 = new QQuaternion(.5,10,110,170);
    templatePositions.push_back(position2);
    templateRotations.push_back(rotation2);
    templateNames.push_back("door/door.mesh");

    QVector3D * position3 = new QVector3D(4,7,2);
    QQuaternion * rotation3 = new QQuaternion(.5,10,110,170);
    templatePositions.push_back(position3);
    templateRotations.push_back(rotation3);
    templateNames.push_back("valve/valve.mesh");

    QVector3D * position4 = new QVector3D(4,7,2);
    QQuaternion * rotation4 = new QQuaternion(.5,10,110,170);
    templatePositions.push_back(position4);
    templateRotations.push_back(rotation4);
    templateNames.push_back("vehicle/brake_pedal_mm.mesh");

    QVector3D * position5 = new QVector3D(4,7,2);
    QQuaternion * rotation5 = new QQuaternion(.5,10,110,170);
    templatePositions.push_back(position5);
    templateRotations.push_back(rotation5);
    templateNames.push_back("tools/DRC_drill.mesh");

    QVector3D * position6 = new QVector3D(4,7,2);
    QQuaternion * rotation6 = new QQuaternion(.5,10,110,170);
    templatePositions.push_back(position6);
    templateRotations.push_back(rotation6);
    templateNames.push_back("valve/QUAL_valve_holes.mesh");

    QVector3D * position7 = new QVector3D(4,7,2);
    QQuaternion * rotation7 = new QQuaternion(.5,10,110,170);
    templatePositions.push_back(position7);
    templateRotations.push_back(rotation7);
    templateNames.push_back("vehicle/throttle_mm.mesh");

    QVector3D * position8 = new QVector3D(4,7,2);
    QQuaternion * rotation8 = new QQuaternion(.5,10,110,170);
    templatePositions.push_back(position8);
    templateRotations.push_back(rotation8);
    templateNames.push_back("door/doorhandle.mesh");

    QVector3D * position9 = new QVector3D(4,7,2);
    QQuaternion * rotation9 = new QQuaternion(.5,10,110,170);
    templatePositions.push_back(position9);
    templateRotations.push_back(rotation9);
    templateNames.push_back("tools/ladder.mesh");

    QVector3D * position10 = new QVector3D(4,7,2);
    QQuaternion * rotation10 = new QQuaternion(.5,10,110,170);
    templatePositions.push_back(position10);
    templateRotations.push_back(rotation10);
    templateNames.push_back("hose/coupling_hexagon.mesh");

}

void MainViewWidget::keyPressEvent(QKeyEvent *event)
{
    if(event->key() == Qt::Key_Space)
    {
        if(challengeCount < templatePositions.size())
        {
            newChallenge();
            challengeCount++;
        }
    }
}

void MainViewWidget::newChallenge()
{
    //write info about template locations, restart timer

    qint64 challengeTime = elapsedTimer->restart();

    if(temList.template_id_list.size() != 0)
    {
        QString timeStr = QString::number(challengeTime);
        //grab string form of goal position
        QString goalPosXStr = QString::number(temList.pose[0].pose.position.x );
        QString goalPosYStr = QString::number(temList.pose[0].pose.position.y );
        QString goalPosZStr = QString::number(temList.pose[0].pose.position.z );
        QString goalRotXStr = QString::number(temList.pose[0].pose.orientation.x );
        QString goalRotYStr = QString::number(temList.pose[0].pose.orientation.y );
        QString goalRotZStr = QString::number(temList.pose[0].pose.orientation.z );
        QString goalRotWStr = QString::number(temList.pose[0].pose.orientation.w );
        //grab strings of actual position
        QString actualPosXStr = QString::number(temList.pose[1].pose.position.x );
        QString actualPosYStr = QString::number(temList.pose[1].pose.position.y );
        QString actualPosZStr = QString::number(temList.pose[1].pose.position.z );
        QString actualRotXStr = QString::number(temList.pose[1].pose.orientation.x );
        QString actualRotYStr = QString::number(temList.pose[1].pose.orientation.y );
        QString actualRotZStr = QString::number(temList.pose[1].pose.orientation.z );
        QString actualRotWStr = QString::number(temList.pose[1].pose.orientation.w );

        QTextStream out(results);        
        out << "Challenge :"+ QString::number(challengeCount) +  " time: " + timeStr  +"Position Goal: "+ goalPosXStr + " " +
               goalPosYStr + " " + goalPosZStr + " Rotation Goal: " + goalRotXStr + " " +
               goalRotYStr + " " + goalRotZStr + " " + goalRotWStr + " Actual Position: " +  actualPosXStr + " " +
               actualPosYStr + " " + actualPosZStr + " Actual Rotation: " + actualRotXStr + " " + actualRotYStr + " " + actualRotZStr + " " + actualRotWStr ;
    }


    //remove templates if present
    if(challengeCount > 0)
    {
        flor_ocs_msgs::OCSTemplateRemove template1;
        template1.template_id = 0;

        flor_ocs_msgs::OCSTemplateRemove template2;
        template2.template_id = 1;

        // publish template to be removed
        template_remove_pub_.publish( template1 );
        template_remove_pub_.publish( template2 );
    }


    //insert two templates, one goal, one to be manipulated
    insertTemplate(startingPosition,startingRotation, templateNames[challengeCount]);
    insertTemplate(templatePositions[challengeCount], templateRotations[challengeCount],templateNames[challengeCount]);
}

void MainViewWidget::insertTemplate(QVector3D * position, QQuaternion * rotation, QString path)
{
    flor_ocs_msgs::OCSTemplateAdd cmd;
    geometry_msgs::PoseStamped pose;

    cmd.template_path = path.toStdString();

    pose.pose.position.x = position->x();
    pose.pose.position.y = position->y();
    pose.pose.position.z = position->z();
    pose.pose.orientation.x = rotation->x();
    pose.pose.orientation.y = rotation->y();
    pose.pose.orientation.z = rotation->z();
    pose.pose.orientation.w = rotation->scalar();

    pose.header.frame_id = "/world";

    cmd.pose = pose;

    // publish complete list of templates and poses
    template_add_pub_.publish( cmd );
}

void MainViewWidget::updateContextMenu()
{
    //change default checkable values of context items
    joystickContext->action->setCheckable(true);
    positionContext->action->setCheckable(true);
    graspContext->action->setCheckable(true);
    jointControlContext->action->setCheckable(true);
    pelvisContext->action->setCheckable(true);
    ghostContext->action->setCheckable(true);
    plannerContext->action->setCheckable(true);
    footBasicContext->action->setCheckable(true);
    footAdvancedContext->action->setCheckable(true);
    footParameterContext->action->setCheckable(true);

    //update context menu elements with checks
    if(!ui->joystickBtn->isChecked())
        joystickContext->action->setChecked(false);
    else
        joystickContext->action->setChecked(true);

    if(!ui->jointControlBtn->isChecked())
        jointControlContext->action->setChecked(false);
    else
        jointControlContext->action->setChecked(true);

    if(!ui->pelvisControlBtn->isChecked())
        pelvisContext->action->setChecked(false);
    else
        pelvisContext->action->setChecked(true);

    if(!ui->basicStepBtn->isChecked())
        footBasicContext->action->setChecked(false);
    else
        footBasicContext->action->setChecked(true);

    if(!ui->stepBtn->isChecked())
        footAdvancedContext->action->setChecked(false);
    else
        footAdvancedContext->action->setChecked(true);

    if(!ui->footstepParamBtn->isChecked())
        footParameterContext->action->setChecked(false);
    else
        footParameterContext->action->setChecked(true);

    if(!ui->ghostControlBtn->isChecked())
        ghostContext->action->setChecked(false);
    else
        ghostContext->action->setChecked(true);

    if(!ui->positionModeBtn->isChecked())
         positionContext->action->setChecked(false);
    else
         positionContext->action->setChecked(true);

    if(!ui->plannerConfigBtn->isChecked())
       plannerContext->action->setChecked(false);
    else
       plannerContext->action->setChecked(true);
}

void MainViewWidget::hideGraspWidgets()
{
    graspContainer->hide();
}

//define all context menu items to be displayed for main view and add them to base 3d view
void MainViewWidget::addContextMenu()
{
    //can tell context menu to add a separator when this item is added
    contextMenuItem * separator = new contextMenuItem();
    separator->name = "Separator";

    //create Menu items,
    //the order in which they are created matters
    //must do parent objects before children
    //and in the order you want them to show up in the context menu
    vigir_ocs::Base3DView::makeContextChild("Define Target Pose-Walk",boost::bind(&vigir_ocs::Base3DView::defineWalkPosePressed,(vigir_ocs::Base3DView*)views_list["Top Left"]), NULL, contextMenuElements);
    vigir_ocs::Base3DView::makeContextChild("Define Target Pose-Step",boost::bind(&vigir_ocs::Base3DView::defineStepPosePressed,(vigir_ocs::Base3DView*)views_list["Top Left"]), NULL, contextMenuElements);

    contextMenuElements.push_back(separator);

    vigir_ocs::Base3DView::makeContextChild("Request Point Cloud",boost::bind(&vigir_ocs::Base3DView::publishPointCloudWorldRequest,(vigir_ocs::Base3DView*)views_list["Top Left"]), NULL, contextMenuElements);

    contextMenuElements.push_back(separator);

    //manage windows-------------
    contextMenuItem * windowVisibility = vigir_ocs::Base3DView::makeContextParent("Window Visibility", contextMenuElements);

    joystickContext =  vigir_ocs::Base3DView::makeContextChild("Joystick",boost::bind(&MainViewWidget::contextToggleWindow,this, WINDOW_JOYSTICK), windowVisibility, contextMenuElements);

    contextMenuItem * jointControlMenu = vigir_ocs::Base3DView::makeContextParent("Joint Windows", contextMenuElements);
    jointControlMenu->parent = windowVisibility;

    //elements from joint control toolbar
    jointControlContext = vigir_ocs::Base3DView::makeContextChild("Joint Control",boost::bind(&MainViewWidget::contextToggleWindow,this,WINDOW_JOINT_CONTROL), jointControlMenu, contextMenuElements);
    pelvisContext = vigir_ocs::Base3DView::makeContextChild("Pelvis Pose",boost::bind(&MainViewWidget::contextToggleWindow,this,WINDOW_BDI_PELVIS_POSE), jointControlMenu, contextMenuElements);
    ghostContext = vigir_ocs::Base3DView::makeContextChild("Ghost Control",boost::bind(&MainViewWidget::contextToggleWindow,this,WINDOW_GHOST_CONFIG), jointControlMenu, contextMenuElements);
    plannerContext = vigir_ocs::Base3DView::makeContextChild("Planner Configuration",boost::bind(&MainViewWidget::contextToggleWindow,this,WINDOW_PLANNER_CONFIG), jointControlMenu, contextMenuElements);

    contextMenuItem * footstepControl = vigir_ocs::Base3DView::makeContextParent("Footstep Control", contextMenuElements);
    footstepControl->parent = windowVisibility;

    //elements from footstep control toolbar
    footBasicContext = vigir_ocs::Base3DView::makeContextChild("Basic Footstep Interface",boost::bind(&MainViewWidget::contextToggleWindow,this,WINDOW_FOOTSTEP_BASIC), footstepControl, contextMenuElements);
    footAdvancedContext = vigir_ocs::Base3DView::makeContextChild("Advanced Footstep Interface",boost::bind(&MainViewWidget::contextToggleWindow,this,WINDOW_FOOTSTEP_ADVANCED), footstepControl, contextMenuElements);
    footParameterContext = vigir_ocs::Base3DView::makeContextChild("Footstep Parameter Control",boost::bind(&MainViewWidget::contextToggleWindow,this,WINDOW_FOOTSTEP_PARAMETER), footstepControl, contextMenuElements);

    graspContext = vigir_ocs::Base3DView::makeContextChild("Grasp",boost::bind(&MainViewWidget::graspWidgetToggle,this), windowVisibility , contextMenuElements);

    positionContext = vigir_ocs::Base3DView::makeContextChild("Position Mode",boost::bind(&MainViewWidget::contextToggleWindow,this,WINDOW_POSITION_MODE), windowVisibility, contextMenuElements);

    //------------------------
    contextMenuElements.push_back(separator);

    contextMenuItem * manipulationModes = vigir_ocs::Base3DView::makeContextParent("Manipulation Mode", contextMenuElements);

    vigir_ocs::Base3DView::makeContextChild("Camera",boost::bind(&MainViewWidget::setCameraMode,this), manipulationModes, contextMenuElements);
    vigir_ocs::Base3DView::makeContextChild("World",boost::bind(&MainViewWidget::setWorldMode,this), manipulationModes, contextMenuElements);
    vigir_ocs::Base3DView::makeContextChild("Object",boost::bind(&MainViewWidget::setObjectMode,this), manipulationModes, contextMenuElements);

    contextMenuItem * objectModes = vigir_ocs::Base3DView::makeContextParent("Object Mode", contextMenuElements);

    vigir_ocs::Base3DView::makeContextChild("Template",boost::bind(&MainViewWidget::setTemplateMode,this), objectModes, contextMenuElements);
    vigir_ocs::Base3DView::makeContextChild("Left Arm",boost::bind(&MainViewWidget::setLeftArmMode,this), objectModes, contextMenuElements);
    vigir_ocs::Base3DView::makeContextChild("Right Arm",boost::bind(&MainViewWidget::setRightArmMode,this), objectModes, contextMenuElements);         

    contextMenuElements.push_back(separator);

    contextMenuItem * systemCommands = vigir_ocs::Base3DView::makeContextParent("System Commands", contextMenuElements);

    vigir_ocs::Base3DView::makeContextChild("Reset World Model",boost::bind(&MainViewWidget::systemCommandContext,this, "reset"), systemCommands, contextMenuElements);
    vigir_ocs::Base3DView::makeContextChild("Save Octomap",boost::bind(&MainViewWidget::systemCommandContext,this,"save_octomap"), systemCommands, contextMenuElements);
    vigir_ocs::Base3DView::makeContextChild("Save Pointcloud",boost::bind(&MainViewWidget::systemCommandContext,this,"save_pointcloud"), systemCommands, contextMenuElements);

    //add all context menu items to each view
    for(int i=0;i<contextMenuElements.size();i++)
    {
        ((vigir_ocs::Base3DView*) views_list["Top Left"])->addToContextVector(contextMenuElements[i]);
        ((vigir_ocs::Base3DView*) views_list["Top Right"])->addToContextVector(contextMenuElements[i]);
        ((vigir_ocs::Base3DView*) views_list["Bottom Left"])->addToContextVector(contextMenuElements[i]);
        ((vigir_ocs::Base3DView*) views_list["Bottom Right"])->addToContextVector(contextMenuElements[i]);
    }
}

//callback functions for context menu
void MainViewWidget::systemCommandContext(std::string command)
{
    sysCmdMsg.data = command;
    sys_command_pub_.publish(sysCmdMsg);
}
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
    case WINDOW_FOOTSTEP_BASIC:
        if(!ui->basicStepBtn->isChecked())
            ui->basicStepBtn->setChecked(true);
        else
            ui->basicStepBtn->setChecked(false);
        break;
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
void MainViewWidget::setTemplateMode()
{
    setObjectMode(0);
}
void MainViewWidget::setLeftArmMode()
{
    setObjectMode(1);
}
void MainViewWidget::setRightArmMode()
{
    setObjectMode(2);
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
void MainViewWidget::setObjectMode()
{
    ui->modeBox->setCurrentIndex(0);
    setManipulationMode(0);
}

void MainViewWidget::setManipulationMode(int mode)
{
    // update template joystick
    flor_ocs_msgs::OCSJoystick msg;
    msg.manipulationMode =  mode;
    joystick_pub_.publish(msg);
    // update interactive markers
    std_msgs::Int8 m;
    m.data = mode;
    interactive_marker_mode_pub_.publish(m);
}

void MainViewWidget::setObjectMode(int mode)
{
    flor_ocs_msgs::OCSJoystick msg;
    msg.objectMode =  mode;
    msg.manipulationMode = ui->modeBox->currentIndex();
    joystick_pub_.publish(msg);
}

void MainViewWidget::graspWidgetToggle()
{
    if(!graspContainer->isVisible())
    {

        graspContainer->show();
        graspFadeIn->start();
        //graspContainer->setGeometry(ui->view_stack_->geometry().bottomRight().x()/2 - 600,ui->view_stack_->geometry().bottomRight().y()- 242, 500,300);

        //reset graphic on toggle button
        QPixmap pixmap(icon_path_+"down_arrow_white.png");
        QIcon ButtonIcon(pixmap);
        grasp_toggle_button_->setIcon(ButtonIcon);
        grasp_toggle_button_->setIconSize(pixmap.rect().size()/10);
    }
    else // visible
    {
        graspFadeOut->start();

        //reset graphic on button
        QPixmap pixmap(icon_path_+"up_arrow_white.png");
        QIcon ButtonIcon(pixmap);
        grasp_toggle_button_->setIcon(ButtonIcon);
        grasp_toggle_button_->setIconSize(pixmap.rect().size()/10);
    }
}

MainViewWidget::~MainViewWidget()
{
    delete ui;
}

void MainViewWidget::timerEvent(QTimerEvent *event)
{
    grasp_toggle_button_->setGeometry(ui->view_stack_->geometry().bottomRight().x()-68,ui->view_stack_->geometry().bottom()+ 22,68,30);    
    //ROS_ERROR("grasp button pos x:%d y:%d ",ui->view_stack_->mapToGlobal(ui->view_stack_->geometry().bottomRight()).x()/2 - 68,ui->view_stack_->mapToGlobal(ui->view_stack_->geometry().bottomRight()).y()+22 );
    //must be global, as it is treated as dialog window
    graspContainer->setGeometry(ui->view_stack_->mapToGlobal(ui->view_stack_->geometry().bottomRight()).x() - graspContainer->geometry().width()/2 - ui->view_stack_->geometry().width()/2,ui->view_stack_->mapToGlobal(ui->view_stack_->geometry().bottomRight()).y() - graspContainer->geometry().height(), graspContainer->geometry().width(),graspContainer->geometry().height());

}


bool MainViewWidget::eventFilter( QObject * o, QEvent * e )
{
    if ( e->type() == QEvent::Wheel &&
         (qobject_cast<QAbstractSpinBox*>( o ) || qobject_cast<QAbstractSlider*>( o ) || qobject_cast<QComboBox*>( o )))
    {
        e->ignore();
        return true;
    }
    return QWidget::eventFilter( o, e );
}

void MainViewWidget::setupToolbar()
{
    //place graphic on joystick toggle
    loadButtonIcon(ui->joystickBtn, "controllerIcon.png");
    loadButtonIcon(ui->jointControlBtn, "jointIcon.png");
    loadButtonIcon(ui->pelvisControlBtn, "pelvis.png");
    loadButtonIcon(ui->ghostControlBtn, "ghostIcon.png");
    loadButtonIcon(ui->positionModeBtn, "positionIcon.png");
    loadButtonIcon(ui->basicStepBtn, "footBasicIcon.png");
    loadButtonIcon(ui->stepBtn, "footAdvancedIcon.png");
    loadButtonIcon(ui->footstepParamBtn, "footParamIcon.png");

    //set button style
    QString btnStyle = QString("QPushButton  { ") +
                               " background-color: qlineargradient(spread:pad, x1:0, y1:0, x2:0, y2:1, stop:0 rgba(240, 240, 240, 255), stop:1 rgba(222, 222, 222, 255));" +
                               " border-style: solid;" +
                               " border-width: 1px;" +
                               " border-radius: 1px;" +
                               " border-color: gray;" +
                               " padding: 0px;" +
                               " image-position: top left"
                               "}" +
                               "QPushButton:checked  {" +
                               " padding-top:1px; padding-left:1px;" +
                               " background-color: rgb(180,180,180);" +
                               " border-style: inset;" +
                               "}";
    ui->joystickBtn->setStyleSheet(btnStyle);
    ui->jointControlBtn->setStyleSheet(btnStyle);
    ui->pelvisControlBtn->setStyleSheet(btnStyle);
    ui->basicStepBtn->setStyleSheet(btnStyle);
    ui->stepBtn->setStyleSheet(btnStyle);
    ui->footstepParamBtn->setStyleSheet(btnStyle);
    ui->ghostControlBtn->setStyleSheet(btnStyle);
    ui->positionModeBtn->setStyleSheet(btnStyle);
    ui->plannerConfigBtn->setStyleSheet(btnStyle);

    //use signalmapper to avoid having one function for each one of the toggle buttons
    toggle_mapper_ = new QSignalMapper(this);
    connect(toggle_mapper_,SIGNAL(mapped(int)),this,SLOT(toggleWindow(int)));

    //map all toggles button to their identifiers
    toggle_mapper_->setMapping(this->ui->joystickBtn,WINDOW_JOYSTICK);
    toggle_mapper_->setMapping(this->ui->jointControlBtn,WINDOW_JOINT_CONTROL);
    toggle_mapper_->setMapping(this->ui->pelvisControlBtn,WINDOW_BDI_PELVIS_POSE);
    toggle_mapper_->setMapping(this->ui->basicStepBtn,WINDOW_FOOTSTEP_BASIC);
    toggle_mapper_->setMapping(this->ui->stepBtn,WINDOW_FOOTSTEP_ADVANCED);
    toggle_mapper_->setMapping(this->ui->footstepParamBtn,WINDOW_FOOTSTEP_PARAMETER);
    toggle_mapper_->setMapping(this->ui->ghostControlBtn,WINDOW_GHOST_CONFIG);
    toggle_mapper_->setMapping(this->ui->positionModeBtn,WINDOW_POSITION_MODE);
    toggle_mapper_->setMapping(this->ui->plannerConfigBtn,WINDOW_PLANNER_CONFIG);

    //connect all buttons for mouse presses
    connect(ui->joystickBtn,SIGNAL(toggled(bool)),toggle_mapper_,SLOT(map()));
    connect(ui->jointControlBtn,SIGNAL(toggled(bool)),toggle_mapper_,SLOT(map()));
    connect(ui->pelvisControlBtn,SIGNAL(toggled(bool)),toggle_mapper_,SLOT(map()));
    connect(ui->basicStepBtn,SIGNAL(toggled(bool)),toggle_mapper_,SLOT(map()));
    connect(ui->stepBtn,SIGNAL(toggled(bool)),toggle_mapper_,SLOT(map()));
    connect(ui->footstepParamBtn,SIGNAL(toggled(bool)),toggle_mapper_,SLOT(map()));
    connect(ui->ghostControlBtn,SIGNAL(toggled(bool)),toggle_mapper_,SLOT(map()));
    connect(ui->positionModeBtn,SIGNAL(toggled(bool)),toggle_mapper_,SLOT(map()));
    connect(ui->plannerConfigBtn,SIGNAL(toggled(bool)),toggle_mapper_,SLOT(map()));
}

void MainViewWidget::loadButtonIcon(QPushButton* btn, QString image_name)
{
    QPixmap pixmap( icon_path_+ image_name );
    QIcon icon(pixmap);
    btn->setIcon(icon);
    QSize size(btn->size());
    btn->setIconSize(size);
}

void MainViewWidget::toggleWindow(int window)
{
    std_msgs::Int8 cmd;
    cmd.data = ((QPushButton*)toggle_mapper_->mapping(window))->isChecked() ? window : -window;
    window_control_pub_.publish(cmd);
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

    for (iter = views_list.begin(); iter != views_list.end(); ++iter)
    {
        if(iter->second == views_list["Top Left"])
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

    for (iter = views_list.begin(); iter != views_list.end(); ++iter)
        ((vigir_ocs::Base3DView*)iter->second)->updateRenderMask(true);
}

void MainViewWidget::ft_sensorToggled(bool toggled){
    ((vigir_ocs::PerspectiveView*)views_list["Top Left"])->ft_sensorToggled(toggled);
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

void MainViewWidget::processNewKeyEvent(const flor_ocs_msgs::OCSKeyEvent::ConstPtr &key_event)
{
    // store key state
    if(key_event->state)
        keys_pressed_list_.push_back(key_event->key);
    else
        keys_pressed_list_.erase(std::remove(keys_pressed_list_.begin(), keys_pressed_list_.end(), key_event->key), keys_pressed_list_.end());

    // process hotkeys
    std::vector<int>::iterator key_is_pressed;

    // ctrl
    //key_is_pressed = std::find(keys_pressed_list_.begin(), keys_pressed_list_.end(), 37);

}

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
            ui->basicStepBtn->setChecked(false);
            ui->stepBtn->setChecked(false);
            ui->footstepParamBtn->setChecked(false);
            ui->ghostControlBtn->setChecked(false);
            ui->positionModeBtn->setChecked(false);
            ui->plannerConfigBtn->setChecked(false);
            break;
        case WINDOW_FOOTSTEP_BASIC:
            ui->basicStepBtn->setChecked(visibility > 0 ? true : false);
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
