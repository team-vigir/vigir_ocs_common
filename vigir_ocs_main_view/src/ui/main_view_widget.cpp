#include "main_view_widget.h"
#include "ui_main_view_widget.h"
#include "ui/template_loader_widget.h"
#include "perspective_view.h"
#include "ortho_view.h"
#include <ros/package.h>
#include <rviz/visualization_manager.h>
#include <rviz/displays_panel.h>
#include <rviz/views_panel.h>
#include <QPropertyAnimation>
#include <flor_ocs_msgs/WindowCodes.h>

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
    ui->modeBox->addItem(QString("Camera"));
    ui->modeBox->addItem(QString("World"));
    ui->modeBox->addItem(QString("Object"));

    // workaround to be able to use images from stylesheet without knowing the path in advance
    QString stylesheet = ui->modeBox->styleSheet() + "\n" +
            "QComboBox::down-arrow {\n" +
            " image: url(" + icon_path_ + "down_arrow.png" + ");\n" +
            "}";
    ui->modeBox->setStyleSheet(stylesheet);

    ui->objectBox->addItem(QString("Template"));
    ui->objectBox->addItem(QString("Left Arm"));
    ui->objectBox->addItem(QString("Right Arm"));


    stylesheet = ui->objectBox->styleSheet() + "\n" +
            "QComboBox::down-arrow {\n" +
            " image: url(" + icon_path_ + "down_arrow.png" + ");\n" +
            "}";
    ui->objectBox->setStyleSheet(stylesheet);

    //allow combo boxes to send messages to joystick
    connect(ui->modeBox,SIGNAL(currentIndexChanged(int)),this,SLOT(setManipulationMode(int)));
    connect(ui->objectBox,SIGNAL(currentIndexChanged(int)),this,SLOT(setObjectMode(int)));

    //publisher for joystick modes
    joystick_pub_ = n_.advertise<flor_ocs_msgs::OCSJoystick>("/flor/ocs/joystick",1,false);

    statusBar = new StatusBar();

    //connect view to update position data
    connect(views_list["Top Left"],SIGNAL(sendPositionText(QString)),statusBar,SLOT(receivePositionText(QString)));

    ui->statusLayout->addWidget(statusBar);

    grasp_toggle_button_ = new QPushButton("Grasp",this);
    grasp_toggle_button_->setStyleSheet("font: 8pt \"MS Shell Dlg 2\";background-color: rgb(0, 0, 0);color: rgb(108, 108, 108);border-color: rgb(0, 0, 0); ");
    grasp_toggle_button_->setMaximumSize(68,22);
    grasp_toggle_button_->adjustSize();
//    loadButtonIcon(grasp_toggle_button_,"down_arrow_white.png");

    QPixmap pixmap(icon_path_+"up_arrow_white.png");
    QIcon ButtonIcon(pixmap);
    grasp_toggle_button_->setIcon(ButtonIcon);
    grasp_toggle_button_->setIconSize(pixmap.rect().size()/10);

    connect(grasp_toggle_button_, SIGNAL(clicked()), this, SLOT(graspWidgetToggle()));

    leftGraspWidget = new graspWidget(this);
    rightGraspWidget = new graspWidget(this);

    //leftGraspWidget->setStyleSheet(".QWidget {border-style: solid;  border-radius: 1px;border-width: 1px; border-color: yellow;}");
    //rightGraspWidget->setStyleSheet(".QWidget {border-style: solid;  border-radius: 1px;border-width: 1px; border-color: cyan;}");

    Qt::WindowFlags flags = leftGraspWidget->windowFlags();
    //flags |= Qt::WindowStaysOnTopHint;
    flags |= Qt::FramelessWindowHint;
    flags |= Qt::Dialog;
    leftGraspWidget->setWindowFlags(flags);
    rightGraspWidget->setWindowFlags(flags);

    //set border color of left grasp widget
    QFrame* leftFrame = new QFrame(leftGraspWidget);
    leftFrame->setFrameStyle(QFrame::Panel| QFrame::Plain);
    //leftFrame->setFrameRect(leftGraspWidget->rect());
    leftFrame->setFixedSize(633,290);
    leftFrame->setLineWidth(2);


    QPalette* palette = new QPalette();
    palette->setColor(QPalette::Foreground,Qt::yellow);
    leftFrame->setPalette(*palette);

    //set border color of right grasp widget
    QFrame* rightFrame = new QFrame(rightGraspWidget);
    rightFrame->setFrameStyle(QFrame::WinPanel | QFrame::Plain);
    //rightFrame->setFrameRect(rightGraspWidget->rect());
    rightFrame->setFixedSize(633,290);
    rightFrame->setLineWidth(2);


    palette->setColor(QPalette::Foreground,Qt::cyan);
    rightFrame->setPalette(*palette);
    delete(palette);

    rightGraspFadeIn = new QPropertyAnimation(rightGraspWidget, "windowOpacity");
    rightGraspFadeIn->setEasingCurve(QEasingCurve::InOutQuad);
    rightGraspFadeIn->setDuration(500);
    rightGraspFadeIn->setStartValue(0.0);
    rightGraspFadeIn->setEndValue(.8);

    rightGraspFadeOut = new QPropertyAnimation(rightGraspWidget, "windowOpacity");
    rightGraspFadeOut->setEasingCurve(QEasingCurve::InOutQuad);
    rightGraspFadeOut->setDuration(300);
    rightGraspFadeOut->setStartValue(0.8);
    rightGraspFadeOut->setEndValue(0.0);

    leftGraspFadeIn = new QPropertyAnimation(leftGraspWidget, "windowOpacity");
    leftGraspFadeIn->setEasingCurve(QEasingCurve::InOutQuad);
    leftGraspFadeIn->setDuration(500);
    leftGraspFadeIn->setStartValue(0.0);
    leftGraspFadeIn->setEndValue(.8);

    leftGraspFadeOut = new QPropertyAnimation(leftGraspWidget, "windowOpacity");
    leftGraspFadeOut->setEasingCurve(QEasingCurve::InOutQuad);
    leftGraspFadeOut->setDuration(300);
    leftGraspFadeOut->setStartValue(0.8);
    leftGraspFadeOut->setEndValue(0.0);


//     QSpacerItem* spacer = new QSpacerItem(300,0,QSizePolicy::Expanding,QSizePolicy::Expanding);
//     QSpacerItem* spacer2 = new QSpacerItem(20,0,QSizePolicy::Expanding,QSizePolicy::Expanding);
//     QSpacerItem* spacer3 = new QSpacerItem(300,0,QSizePolicy::Expanding,QSizePolicy::Expanding);
//    ui->graspLayout->addSpacerItem(spacer);
//    ui->graspLayout->addWidget(leftGraspWidget);
//    ui->graspLayout->addSpacerItem(spacer3);
//    ui->graspLayout->addWidget(rightGraspWidget);
//    ui->graspLayout->addSpacerItem(spacer2);
    leftGraspWidget->hide();
    rightGraspWidget->hide();

    timer.start(100, this);
}

void MainViewWidget::setManipulationMode(int mode)
{
    flor_ocs_msgs::OCSJoystick msg;
    msg.manipulationMode =  mode;
    msg.objectMode = ui->objectBox->currentIndex();
    joystick_pub_.publish(msg);
    ROS_ERROR("publish joystick mode %d %d",msg.manipulationMode, msg.objectMode);
}

void MainViewWidget::setObjectMode(int mode)
{    
    flor_ocs_msgs::OCSJoystick msg;
    msg.objectMode =  mode;
    msg.manipulationMode = ui->modeBox->currentIndex();
    joystick_pub_.publish(msg);
     ROS_ERROR("publish joystick mode %d %d",msg.manipulationMode, msg.objectMode);
}

void MainViewWidget::graspWidgetToggle()
{
    if(!leftGraspWidget->isVisible())
    {
        //leftGraspWidget->setWindowOpacity(75);
        //rightGraspWidget->setWindowOpacity(75);
        leftGraspWidget->show();
        rightGraspWidget->show();
        rightGraspFadeIn->start();
        leftGraspFadeIn->start();

        QPixmap pixmap(icon_path_+"down_arrow_white.png");
        QIcon ButtonIcon(pixmap);
        grasp_toggle_button_->setIcon(ButtonIcon);
        grasp_toggle_button_->setIconSize(pixmap.rect().size()/10);

    }
    else // visible
    {
        leftGraspWidget->hide();
        rightGraspWidget->hide();

        QPixmap pixmap(icon_path_+"up_arrow_white.png");
        QIcon ButtonIcon(pixmap);
        grasp_toggle_button_->setIcon(ButtonIcon);
        grasp_toggle_button_->setIconSize(pixmap.rect().size()/10);
    }
}

MainViewWidget::~MainViewWidget()
{
    //delete(joystick);
    delete ui;    
}


void MainViewWidget::timerEvent(QTimerEvent *event)
{
    //reposition grasp button
    grasp_toggle_button_->setGeometry(ui->view_stack_->geometry().bottomRight().x()-68,ui->view_stack_->geometry().bottom()+ 24,68,18);

    leftGraspWidget->setGeometry(ui->view_stack_->geometry().bottomLeft().x() + 200,ui->view_stack_->geometry().bottomRight().y()- 197, 300,200);
    rightGraspWidget->setGeometry(ui->view_stack_->geometry().bottomRight().x()- 740,ui->view_stack_->geometry().bottomRight().y()- 197, 300,200);


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
    connect(ui->stepBtn,SIGNAL(pressed()),toggle_mapper_,SLOT(map()));
    connect(ui->footstepParamBtn,SIGNAL(pressed()),toggle_mapper_,SLOT(map()));
    connect(ui->ghostControlBtn,SIGNAL(pressed()),toggle_mapper_,SLOT(map()));
    connect(ui->positionModeBtn,SIGNAL(pressed()),toggle_mapper_,SLOT(map()));
    connect(ui->plannerConfigBtn,SIGNAL(pressed()),toggle_mapper_,SLOT(map()));
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
