#include "graspWidget.h"
#include "ui_graspWidget.h"

#include <algorithm>

#include <boost/asio/ip/host_name.hpp>

#include <QColor>
#include <QProgressBar>
#include <QSlider>

#include <ros/package.h>

//grasp_testing grasp_testing_simple.launch

graspWidget::graspWidget(QWidget *parent, std::string hand, std::string hand_name)
    : QWidget(parent)
    , ui(new Ui::graspWidget)
    , selected_template_id_(-1)
    , selected_grasp_id_(-1)
    , selected_affordance_id_(-1)
    , selected_usability_id_(-1)
    , show_grasp_(false)
    , follow_ban_(false)
    , hand_side_(hand)
    , hand_name_(hand_name)
    , wrist_link_("l_hand")
    , palm_link_("l_palm")
    , hand_group_("l_hand_group")
{

    std::string ip = ros::package::getPath("vigir_ocs_grasp_widget")+"/icons/";
    icon_path_ = QString(ip.c_str());

    // setup UI
    ui->setupUi(this);
    ui->templateBox->setDisabled(true);
    ui->graspBox->setDisabled(true);
    ui->affordanceBox->setDisabled(true);
    ui->usabilityBox->setDisabled(true);
    ui->attachButton->setDisabled(true);
    ui->detachButton->setDisabled(true);
    ui->show_grasp->setDisabled(true);
    ui->graspPose->setDisabled(true);
    setUpButtons();

    this->stitch_template_pose_.setIdentity();

    this->hand_T_palm_.setIdentity();

    // initialize variables
    currentGraspMode = 0;
    templateMatchDone = false;

    std::string grasp_control_prefix = "/grasp_control/" + hand_name_;
    // initialize template subscribers and publishers
    template_list_sub_           = nh_.subscribe(    "/template/list",                    5, &graspWidget::processTemplateList, this );
    template_match_feedback_sub_ = nh_.subscribe("/grasp_control/template_selection", 1, &graspWidget::templateMatchFeedback, this );
    grasp_state_sub_             = nh_.subscribe(      grasp_control_prefix+"/active_state", 1, &graspWidget::graspStateReceived,  this );
    grasp_sync_sub_              = nh_.subscribe("/flor/ocs/grasp_sync", 1, &graspWidget::processGraspSyncCB,  this );

    grasp_selection_pub_         = nh_.advertise<vigir_grasp_msgs::GraspSelection>(    grasp_control_prefix+"/grasp_selection",                            1, false);
    grasp_release_pub_           = nh_.advertise<vigir_grasp_msgs::GraspSelection>(    grasp_control_prefix+"/release_grasp" ,                             1, false);
    grasp_command_pub_           = nh_.advertise<vigir_grasp_msgs::GraspState>(        grasp_control_prefix+"/grasp_command",                              1, false);
    affordance_selection_pub_    = nh_.advertise<vigir_object_template_msgs::Affordance>( "/manipulation_control/" + hand_name_ + "/affordance_command",  1, false);
    snap_template_pub_           = nh_.advertise<vigir_grasp_msgs::TemplateSelection>( "/template/snap",                                                   1, false);
    hand_marker_pub_             = nh_.advertise<std_msgs::Int8>(                     "/manipulation_control/" + hand_name_ + "/hand_marker",             1, false);
    grasp_sync_pub_              = nh_.advertise<vigir_ocs_msgs::OCSGraspSync>(                     "/flor/ocs/grasp_sync",             1, false);

    grasp_info_client_           = nh_.serviceClient<vigir_object_template_msgs::GetGraspInfo>("/grasp_info");
    template_info_client_        = nh_.serviceClient<vigir_object_template_msgs::GetInstantiatedTemplateStateAndTypeInfo>("/instantiated_template_info");

    //Get hand parameters from server
    left_wrist_link_ = "l_hand";
    left_palm_link_  = "l_palm";
    left_hand_group_ = "l_hand_group";
    right_wrist_link_ = "r_hand";
    right_palm_link_  = "r_palm";
    right_hand_group_ = "r_hand_group";

    if(!nh_.getParam("/left_wrist_link", left_wrist_link_))
        ROS_WARN("No left wrist link defined, using l_hand as default");
    if(!nh_.getParam("/left_palm_link",  left_palm_link_))
        ROS_WARN("No left palm link defined, using l_palm as default");
    if(!nh_.getParam("/left_hand_group", left_hand_group_))
        ROS_WARN("No left hand group defined, using l_hand_group as default");

    if(!nh_.getParam("/right_wrist_link", right_wrist_link_))
        ROS_WARN("No right wrist link defined, using r_hand as default");
    if(!nh_.getParam("/right_palm_link",  right_palm_link_))
        ROS_WARN("No right palm link defined, using r_palm as default");
    if(!nh_.getParam("/right_hand_group", right_hand_group_))
        ROS_WARN("No right hand group defined, using r_hand_group as default");

    ui->stackedWidget->setVisible(true);

    float color_r, color_g, color_b;

    if(hand_side_ == "left")
    {
        color_r = 1.0f;
        color_g = 1.0f;
        color_b = 0.0f;

        palm_link_  = left_palm_link_;
        wrist_link_ = left_wrist_link_;
        hand_group_ = left_hand_group_;
    }else{
        color_r = 0.0f;
        color_g = 1.0f;
        color_b = 1.0f;

        palm_link_  = right_palm_link_;
        wrist_link_ = right_wrist_link_;
        hand_group_ = right_hand_group_;
    }

    this->setWindowTitle(QString::fromStdString(hand_side_ + " Hand Grasp Widget"));

    //Publisher for template match rewuest for
    template_match_request_pub_ = nh_.advertise<vigir_grasp_msgs::TemplateSelection>( "/template/" + hand_name_ + "_template_match_request", 1, false );

    move_request_pub_  = nh_.advertise<vigir_grasp_msgs::GraspSelection>(    "/manipulation_control/" + hand_name_ + "/move_to_pose",  1, false );
    detach_object_pub_ = nh_.advertise<vigir_grasp_msgs::TemplateSelection>( "/manipulation_control/" + hand_name_ + "/detach_object", 1, false );

    robot_status_sub_           = nh_.subscribe( "/grasp_control/" + hand_name_ + "/grasp_status",1, &graspWidget::robotStatusCB,  this );
    ghost_hand_pub_             = nh_.advertise<geometry_msgs::PoseStamped>(     "/ghost_" + hand_side_ + "_hand_pose",             1, false);
    //ghost_hand_joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>(        "/ghost_" + hand_ + "_hand/joint_states",     1, false); // /ghost_" + hand_ + "_hand/joint_states


    //Start hand model operations
    hand_model_loader_.reset(new robot_model_loader::RobotModelLoader(hand_side_ + "_hand_robot_description"));
    hand_robot_model_ = hand_model_loader_->getModel();

    if (!hand_robot_model_.get()){
      ROS_ERROR("Error loading hand robot model in GraspWidget, will not work correctly!");
      return;
    }

    hand_robot_state_.reset(new robot_state::RobotState(hand_robot_model_));
    //Finish hand model operations

    //Start robot model operations
    robot_model_loader_.reset(new robot_model_loader::RobotModelLoader("robot_description"));
    robot_model_ = robot_model_loader_->getModel();

    if(robot_model_->hasJointModelGroup(hand_group_))
    {
        hand_joint_names_.clear();
        hand_joint_names_ = robot_model_->getJointModelGroup(hand_group_)->getJointModelNames();//->getActiveJointModelNames();

        for(int i = 0; i < hand_joint_names_.size(); i++)
            ROS_INFO("Grasp widget loading joint %d: %s",i,hand_joint_names_[i].c_str());

        if(!robot_model_->hasLinkModel(palm_link_)){
            ROS_WARN("Hand model does not contain %s",palm_link_.c_str());
        }else{
            robot_model::LinkTransformMap hand_palm_tf_map = robot_model_->getLinkModel(palm_link_)->getAssociatedFixedTransforms();
            ROS_INFO("Requested linktransform for %s",palm_link_.c_str());

            Eigen::Affine3d hand_palm_aff;
            bool found = false;

            for(robot_model::LinkTransformMap::iterator it = hand_palm_tf_map.begin(); it != hand_palm_tf_map.end(); ++it){
                ROS_INFO("Getting links in map: %s and comparing to: %s", it->first->getName().c_str(), wrist_link_.c_str());
                if(it->first->getName() == wrist_link_){
                    ROS_INFO("Wrist %s found!!!",wrist_link_.c_str());
                    hand_palm_aff = it->second;
                    found = true;
                    break;
                }
            }

            if(!found){
                ROS_WARN("Wrist %c_hand NOT found!!!, setting to identity",hand_side_[0]);
            }else{
                tf::transformEigenToTF( hand_palm_aff,hand_T_palm_);
                hand_T_palm_ = hand_T_palm_.inverse();
            }
        }
        // change color of the ghost template hands
        std::vector<std::string> link_names = hand_robot_model_->getJointModelGroup(hand_group_)->getLinkModelNames();
        link_names.push_back(palm_link_);

        for (size_t i = 0; i < link_names.size(); ++i)
        {
            moveit_msgs::ObjectColor tmp;
            tmp.id = link_names[i];
            tmp.color.a = 0.5f;
            tmp.color.r = color_r;
            tmp.color.g = color_g;
            tmp.color.b = color_b;
            display_state_msg_.highlight_links.push_back(tmp);
        }
    }else{
        ROS_WARN("NO JOINTS FOUND FOR %s HAND",hand_side_.c_str());
    }

    //Finish robot model operations


    // Publisher for hand position/state
    robot_state_vis_pub_ = nh_.advertise<moveit_msgs::DisplayRobotState>("/flor/ghost/template_" + hand_side_ + "_hand",1, true);

    // We first subscribe to the JointState messages
    link_states_sub_ = nh_.subscribe( "/grasp_control/" + hand_name_ + "/tactile_feedback", 2, &graspWidget::linkStatesCB, this );

    template_stitch_pose_sub_    = nh_.subscribe( "/manipulation_control/" + hand_name_ + "/template_stitch_pose",1, &graspWidget::templateStitchPoseCallback,  this );
    template_stitch_request_pub_ = nh_.advertise<vigir_grasp_msgs::GraspSelection>( "/manipulation_control/" + hand_name_ + "/template_stitch_request", 1, false );

    XmlRpc::XmlRpcValue   gp_T_palm;

    if(nh_.getParam("/" + hand_name_ + "_tf/gp_T_palm", gp_T_palm))
    {
        gp_T_palm_.setOrigin(tf::Vector3(static_cast<double>(gp_T_palm[0]),static_cast<double>(gp_T_palm[1]),static_cast<double>(gp_T_palm[2])));
        gp_T_palm_.setRotation(tf::Quaternion(static_cast<double>(gp_T_palm[3]),static_cast<double>(gp_T_palm[4]),static_cast<double>(gp_T_palm[5]),static_cast<double>(gp_T_palm[6])));
    }
    else
    {
        gp_T_palm_.setOrigin(tf::Vector3(0,0,0));
        gp_T_palm_.setRotation(tf::Quaternion(0,0,0,1));
    }




    planning_hand_target_pub_   = nh_.advertise<geometry_msgs::PoseStamped>( "/grasp_control/" + hand_name_ + "/planning_target_pose", 1, false );

    // this is for publishing the hand position in world coordinates for moveit
    virtual_link_joint_states_.name.push_back("world_virtual_joint/trans_x");
    virtual_link_joint_states_.name.push_back("world_virtual_joint/trans_y");
    virtual_link_joint_states_.name.push_back("world_virtual_joint/trans_z");
    virtual_link_joint_states_.name.push_back("world_virtual_joint/rot_x");
    virtual_link_joint_states_.name.push_back("world_virtual_joint/rot_y");
    virtual_link_joint_states_.name.push_back("world_virtual_joint/rot_z");
    virtual_link_joint_states_.name.push_back("world_virtual_joint/rot_w");
    virtual_link_joint_states_.position.resize(7);

    // publisher to color the hand links
    hand_link_color_pub_        = nh_.advertise<vigir_ocs_msgs::OCSLinkColor>("/link_color", 1, false);

    // find robot status message code csv file
    std::string code_path_ = (ros::package::getPath("vigir_ocs_msgs"))+"/include/vigir_ocs_msgs/messages.csv";
    std::cout << code_path_ << std::endl;
    robot_status_codes_.loadErrorMessages(code_path_);

    //accessing ros param to tell if we're main operator or not
    ros::NodeHandle nh("~");
    if(nh.hasParam("operator_type"))
        nh.getParam("operator_type",operator_type_);

    // create publisher and subscriber for object selection
    // PUBLISHER WILL BE USED BY THE RIGHT/DOUBLE CLICK TO INFORM WHICH TEMPLATE/HAND/OBJECT HAS BEEN selected
    // SUBSCRIBER WILL BE USED TO CHANGE VISIBILITY OF THE OBJECT THAT IS BEING USED (E.G., TALK TO TEMPLATE DISPLAY AND SET VISIBILITY OF MARKERS)
    select_object_pub_ = nh_.advertise<vigir_ocs_msgs::OCSObjectSelection>( "/flor/ocs/object_selection", 1, false );
    select_object_sub_ = nh_.subscribe( "/flor/ocs/object_selection", 5, &graspWidget::processObjectSelection, this );

    //key_event_sub_ = nh_.subscribe( "/flor/ocs/key_event", 5, &graspWidget::processNewKeyEvent, this );
    timer.start(33, this);


}
//SetStylesheet to change on the fly


graspWidget::~graspWidget()
{
    ui2->close();
    delete ui;
}

void graspWidget::timerEvent(QTimerEvent *event)
{
	// check if ros is still running; if not, just kill the application
    if(!ros::ok())
        qApp->quit();
    
    // make sure that we don't show the grasp hands if the user doesn't want to see them
    if(!ui->graspBox->isEnabled() || !show_grasp_)
        hideHand();

    //Spin at beginning of Qt timer callback, so current ROS time is retrieved
    ros::spinOnce();
}

void graspWidget::setUpButtons()
{
    //set button style
    QString btnStyle = QString("QPushButton  { ") +
                               " background-color: qlineargradient(spread:pad, x1:0, y1:0, x2:0, y2:1, stop:0 rgba(240, 240, 240, 255), stop:1 rgba(222, 222, 222, 255));" +
                               " border-style: solid;" +
                               " border-width: 1px;" +
                               " border-radius: 1px;" +
                               " border-color: gray;" +
                               " padding: 2px;" +
                               " image-position: top left"
                               "}" +

                               "QPushButton:checked  {" +
                               " padding-top:1px; padding-left:1px;" +
                               " background-color: rgb(180,180,180);" +
                               " border-style: inset;" +
                               "}";
    ui->releaseButton->setStyleSheet(btnStyle);
    ui->releaseButton->setFont(QFont ("Ubuntu", 10));
    ui->performButton->setStyleSheet(btnStyle);
    ui->performButton->setFont(QFont ("Ubuntu", 10));
    ui->moveToPoseButton->setStyleSheet(btnStyle);
    ui->moveToPoseButton->setFont(QFont ("Ubuntu", 10));
    ui->attachButton->setStyleSheet(btnStyle);
    ui->attachButton->setFont(QFont ("Ubuntu", 10));
    ui->detachButton->setStyleSheet(btnStyle);
    ui->detachButton->setFont(QFont ("Ubuntu", 10));
    ui->snapTemplateButton->setStyleSheet(btnStyle);
    ui->snapTemplateButton->setFont(QFont ("Ubuntu", 10));
    ui->affordanceButton->setStyleSheet(btnStyle);
    ui->affordanceButton->setFont(QFont ("Ubuntu", 10));
    ui->graspPose->setStyleSheet(btnStyle);
    ui->graspPose->setFont(QFont ("Ubuntu", 10));
    //put arrows on comboboxes
    QString styleSheet = ui->templateBox->styleSheet() + "\n" +
            "QComboBox::down-arrow {\n" +
            " image: url(" + icon_path_ + "down_arrow.png" + ");\n" +
            "}";
    ui->templateBox->setStyleSheet(styleSheet);
    ui->graspBox->setStyleSheet(styleSheet);
    ui->affordanceBox->setStyleSheet(styleSheet);
    ui->usabilityBox->setStyleSheet(styleSheet);
}

void graspWidget::templateMatchFeedback (const vigir_grasp_msgs::TemplateSelection::ConstPtr feedback)
{
    // provide feedback about template grasp confidence by changing the color of the move to template button
    templateMatchDone = true; // is this still being used?
}

void graspWidget::graspStateReceived (const vigir_grasp_msgs::GraspState::ConstPtr graspState)
{
    ui->userSlider->setValue(graspState->grip.data);

}


void graspWidget::processTemplateList( const vigir_ocs_msgs::OCSTemplateList::ConstPtr list)
{
    //std::cout << "Template list received containing " << list->template_id_list.size() << " elements" << std::cout;
    // save last template list
    last_template_list_ = *list;

    // enable boxes and buttons
    if(list->template_list.size() > 0 && selected_template_id_ != -1)
    {
        ui->graspBox->setDisabled(false);
    }

    bool was_empty = ui->templateBox->count() == 0 ? true : false;

    QString currentItem = ui->templateBox->currentText();
    //ui->templateBox->clear();

    // populate template combobox
    for(int i = 0; i < list->template_list.size(); i++)
    {
        // remove the .mesh string from the template name
        std::string templateName = list->template_list[i];
        if(templateName.size() > 5 && templateName.substr(templateName.size()-5,5) == ".mesh")
            templateName = templateName.substr(0,templateName.size()-5);
        // add the template
        templateName = boost::to_string((int)list->template_id_list[i])+std::string(": ")+templateName;

        //std::cout << "template item " << (int)list->template_id_list[i] << " has name " << templateName << std::endl;

        // add the template to the box if it doesn't exist
        if(ui->templateBox->count() < i+1)
        {
            ui->templateBox->addItem(QString::fromStdString(templateName));
        } // update existing templates
        else if( ui->templateBox->itemText(i).toStdString() != templateName)
        {
            ui->templateBox->setItemText(i,QString::fromStdString(templateName));
        }
    }

    for(int i = list->template_list.size(); i < ui->templateBox->count(); i++)
        ui->templateBox->removeItem(i);

    if(selected_template_id_ != -1 && ui->templateBox->findText(currentItem) == -1)
    {
        hideHand();
        ui->graspBox->clear();
        ui->affordanceBox->clear();
        ui->usabilityBox->clear();
        selected_template_id_   = -1;
        selected_grasp_id_      = -1;
        selected_affordance_id_ = -1;
        selected_usability_id_  = -1;
        ui->graspBox->setEnabled(false);
        ui->affordanceBox->setEnabled(false);
        ui->usabilityBox->setEnabled(false);
        ui->show_grasp->setChecked(false);
        ui->show_grasp->setEnabled(false);
        ui->graspPose->setEnabled(false);
        ui->attachButton->setEnabled(false);
        ui->snapTemplateButton->setEnabled(false);
        ui->detachButton->setEnabled(false);
    }
    else
    {
        if(was_empty && ui->templateBox->count() > 0)
        {
            //ROS_ERROR("Seleting template 0");
            ui->templateBox->setCurrentIndex(0);
            on_templateBox_activated(ui->templateBox->itemText(0));
            selected_template_id_ = 0;
        }

        if(selected_grasp_id_ != -1 && show_grasp_)
            publishHandPose(selected_grasp_id_);
    }
}

void graspWidget::on_moveToPoseButton_clicked()
{
    if(ui->templateBox->count() < 1)
    {
        ROS_ERROR("Tried to move when no templates exsist");
        return;
    }
    hideHand();
    ROS_INFO("Hand move requested...");

    vigir_grasp_msgs::GraspSelection msg;
    msg.template_type.data = last_template_list_.template_type_list[ui->templateBox->currentIndex()];
    msg.template_id.data   = last_template_list_.template_id_list[ui->templateBox->currentIndex()];
    msg.grasp_id.data      = ui->graspBox->currentText().toInt();
    msg.final_pose         = ui->show_grasp_radio->isChecked();

    move_request_pub_.publish(msg);
}

void graspWidget::on_performButton_clicked()
{
    ROS_INFO("Closing hand");
    vigir_grasp_msgs::GraspState cmd;
    cmd.grip.data             = 200;
    cmd.finger_effort.resize(FINGER_EFFORTS);
    cmd.finger_effort[0].data = 0;
    cmd.finger_effort[1].data = 0;   //index for sandia
    cmd.finger_effort[2].data = 0;  //middle for sandia
    cmd.finger_effort[3].data = 0; //Spread iRobot, Pinky for sandia

    ui->userSlider->setValue(100);

    cmd.grasp_state.data = cmd.CLOSE;

    grasp_command_pub_.publish(cmd);
}

void graspWidget::on_releaseButton_clicked()
{
    vigir_grasp_msgs::GraspState cmd;
    cmd.grip.data             = 0;
    cmd.finger_effort.resize(FINGER_EFFORTS);
    cmd.finger_effort[0].data = 0;
    cmd.finger_effort[1].data = 0;   //index for sandia
    cmd.finger_effort[2].data = 0;  //middle for sandia
    cmd.finger_effort[3].data = 0; //Spread iRobot, Pinky for sandia

    ui->userSlider->setValue(0);

    cmd.grasp_state.data = cmd.OPEN;

    grasp_command_pub_.publish(cmd);
}

void graspWidget::on_userSlider_sliderReleased()
{
        sendManualMsg(ui->userSlider->value());
}


void graspWidget::sendManualMsg(uint8_t level)
{
    vigir_grasp_msgs::GraspState cmd;
    cmd.grip.data             = level;
    cmd.grasp_state.data = cmd.PERCENTAGE;
    grasp_command_pub_.publish(cmd);
}



void graspWidget::on_templateBox_activated(const QString &arg1)
{
    // update the selected template id
    QString template_id = ui->templateBox->currentText();
    template_id.remove(template_id.indexOf(": "),template_id.length()-template_id.indexOf(": "));
    selected_template_id_ = template_id.toInt();

    std::cout << "updating the grasp widget grasp selection box contents" << std::endl;
    // clean grasp box
    ui->graspBox->clear();
    ui->affordanceBox->clear();
    ui->usabilityBox->clear();
    ui->usabilityBox->addItem("Palm");
    selected_grasp_id_      = -1;
    selected_affordance_id_ = -1;
    selected_usability_id_  = -1;

    int index;

    for(index = 0; index < last_grasp_srv_.response.grasp_information.grasps.size(); index++)
    {
        if(hand_side_ == "left" && std::atoi(last_grasp_srv_.response.grasp_information.grasps[index].id.c_str()) >=1000)
            ui->graspBox->addItem(QString(last_grasp_srv_.response.grasp_information.grasps[index].id.c_str()));
        if(hand_side_ == "right" && std::atoi(last_grasp_srv_.response.grasp_information.grasps[index].id.c_str()) <1000)
            ui->graspBox->addItem(QString(last_grasp_srv_.response.grasp_information.grasps[index].id.c_str()));
    }

    if(ui->graspBox->count() > 0){
        ui->show_grasp->setEnabled(true);
        ui->graspPose->setEnabled(true);
    }

    for(index = 0; index < last_template_srv_.response.template_type_information.affordances.size(); index++)
    {
        ui->affordanceBox->addItem(QString(last_template_srv_.response.template_type_information.affordances[index].name.c_str()));
    }

    for(index = 0; index < last_template_srv_.response.template_type_information.usabilities.size(); index++)
    {
        ui->usabilityBox->addItem(QString(last_template_srv_.response.template_type_information.usabilities[index].name.c_str()));
        //ui->usabilityBox->addItem(QString((boost::to_string(int(last_template_srv_.response.template_type_information.usabilities[index].id))).c_str()));
    }

    if(ui->affordanceBox->count() > 0){
        ui->affordanceButton->setEnabled(true);
        ui->keepOrientationBox->setEnabled(true);
        ui->displacementBox->setEnabled(true);
        ui->pitchSpinBox->setEnabled(true);
        ui->usabilityBox->setEnabled(true);
    }

    if(index >0){
        ui->affordanceBox->setEnabled(true);
        current_affordance_ = last_template_srv_.response.template_type_information.affordances[0];
        ui->keepOrientationBox->setChecked(current_affordance_.keep_orientation);
        if(current_affordance_.type == "circular")
            ui->displacementBox->setValue(current_affordance_.displacement / 0.0174532925); // UI in deg, msg in rad);
        else
            ui->displacementBox->setValue(current_affordance_.displacement);
        ui->pitchSpinBox->setValue(current_affordance_.pitch);
    }

    if(ui->templateBox->count() > 0){
        selected_grasp_id_      = ui->graspBox->itemText(0).toInt();
        selected_affordance_id_ = ui->affordanceBox->itemText(0).toInt();
    }
}

void graspWidget::on_graspBox_activated(const QString &arg1)
{
    //publish to update grasp id for all operators, provided that were the main operator
    if(operator_type_ != "main")
        return;

    vigir_ocs_msgs::OCSGraspSync msg;
    msg.grasp_id = arg1.toInt();
    msg.sync_mode = vigir_ocs_msgs::OCSGraspSync::GRASP_ID;
    grasp_sync_pub_.publish(msg);

}

void graspWidget::on_affordanceBox_activated(const int &arg1)
{
    std::cout << " affordance selection = " << arg1 << std::endl;
    selected_affordance_id_ = arg1;

    current_affordance_ = last_template_srv_.response.template_type_information.affordances[selected_affordance_id_];

    ui->keepOrientationBox->setChecked(current_affordance_.keep_orientation);
    if(current_affordance_.type == "circular")
        ui->displacementBox->setValue(current_affordance_.displacement / 0.0174532925); // UI in deg, msg in rad);
    else
        ui->displacementBox->setValue(current_affordance_.displacement);
    ui->pitchSpinBox->setValue(current_affordance_.pitch);
}

void graspWidget::on_usabilityBox_activated(const int &arg1)
{
    selected_usability_id_ = arg1-1;
    ROS_INFO(" usability selection = %d",selected_usability_id_);
    std_msgs::Int8 usability;
    usability.data = selected_usability_id_;

    hand_marker_pub_.publish(usability);
}

void graspWidget::robotStatusCB(const vigir_ocs_msgs::OCSRobotStatus::ConstPtr msg)
{
    uint16_t code;
    uint8_t  severity;
    RobotStatusCodes::codes(msg->status,code,severity);
    ui->robot_status_->setText(robot_status_codes_.str(code).c_str());
}

void graspWidget::templateStitchPoseCallback(const geometry_msgs::PoseStamped::ConstPtr msg)
{
    this->stitch_template_pose_.setRotation(tf::Quaternion(msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z,msg->pose.orientation.w));
    this->stitch_template_pose_.setOrigin(tf::Vector3(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z) );
}

void graspWidget::linkStatesCB( const vigir_grasp_msgs::LinkState::ConstPtr link_states )
{
    double min_feedback = 0, max_feedback = 1.0;
    for(int i = 0; i < link_states->name.size(); i++)
    {
        // get the joint name to figure out the color of the links
        std::string link_name = link_states->name[i].c_str();

        // velocity represents tactile feedback
        double feedback = link_states->tactile_array[i].pressure[0];

        //ROS_ERROR("Applying color to %s",link_name.c_str());
        // calculate color intensity based on min/max feedback
        unsigned char color_intensity = (unsigned char)((feedback - min_feedback)/(max_feedback-min_feedback) * 255.0);
        publishLinkColor(link_name,color_intensity,255-color_intensity,0);
    }
}

void graspWidget::publishLinkColor(std::string link_name, unsigned char r, unsigned char g, unsigned char b)
{
    vigir_ocs_msgs::OCSLinkColor cmd;

    cmd.link = link_name;
    cmd.r = r;
    cmd.g = g;
    cmd.b = b;

    hand_link_color_pub_.publish(cmd);
}

void graspWidget::publishHandPose(unsigned int id)
{
    //std::cout << "publishing hand pose for grasp id " << id << std::endl;
    //ROS_ERROR("publishing hand pose for grasp id %d",id);

    geometry_msgs::PoseStamped grasp_pose;
    moveit_msgs::GripperTranslation trans;

    std::vector<float> joints;
    std::vector<std::string> joint_names;

    size_t size = last_grasp_srv_.response.grasp_information.grasps.size();

    if(size == 0)
        ROS_ERROR_STREAM_THROTTLE(5,"No grasps found for this template, this message is throttled");
    else{
        size_t index = 0;
        for(; index < size; ++index)
        {
            if(std::atoi(last_grasp_srv_.response.grasp_information.grasps[index].id.c_str()) == id){
                grasp_pose = last_grasp_srv_.response.grasp_information.grasps[index].grasp_pose;
                joints.resize(last_grasp_srv_.response.grasp_information.grasps[index].grasp_posture.points[0].positions.size());
                joint_names.resize(joints.size());
                trans = last_grasp_srv_.response.grasp_information.grasps[index].pre_grasp_approach;
                for(int j=0; j<joints.size();j++){
                    joints[j]      = last_grasp_srv_.response.grasp_information.grasps[index].grasp_posture.points[0].positions[j];
                    joint_names[j] = last_grasp_srv_.response.grasp_information.grasps[index].grasp_posture.joint_names[j];
                }
                break;
            }
        }

        if(index > size)
            ROS_ERROR_STREAM("Template server response id: " << last_grasp_srv_.response.grasp_information.grasps[index].id << " while searching for id: " << id);
        else{
            // get the selected grasp pose
            geometry_msgs::Pose template_T_palm;//geometry_msgs::PoseStamped grasp_transform;
            template_T_palm = grasp_pose.pose;//grasp_transform.pose = grasp_db_[grasp_index].final_pose;

            staticTransform(template_T_palm);

            if(!ui->show_grasp_radio->isChecked())  //Pre-Grasp pose
                gripperTranslationToPreGraspPose(template_T_palm,trans);            

            unsigned int template_index;
            for(template_index = 0; template_index < last_template_list_.template_id_list.size(); template_index++)
                if(last_template_list_.template_id_list[template_index] == selected_template_id_)
                    break;

            if(template_index == last_template_list_.template_id_list.size()) return;

            //ROS_ERROR("Template transform:     p=(%f, %f, %f) q=(%f, %f, %f, %f)",template_transform.pose.position.x,template_transform.pose.position.y,template_transform.pose.position.z,template_transform.pose.orientation.w,template_transform.pose.orientation.x,template_transform.pose.orientation.y,template_transform.pose.orientation.z);

            geometry_msgs::PoseStamped hand_transform;
            if(last_template_list_.pose[template_index].header.frame_id == "/world"){
                follow_ban_ = false;
                frameid_T_template_.pose = last_template_list_.pose[template_index].pose;
                calcWristTarget(template_T_palm, frameid_T_template_, hand_transform);
            }else{
                if(follow_ban_){
                    hand_transform.pose.position.x = 0;
                    hand_transform.pose.position.y = 0;
                    hand_transform.pose.position.z = 10000;
                    hand_transform.pose.orientation.x = 0;
                    hand_transform.pose.orientation.y = 0;
                    hand_transform.pose.orientation.z = 0;
                    hand_transform.pose.orientation.w = 1;
                }else
                    calcWristTarget(template_T_palm, frameid_T_template_, hand_transform);
            }

            hand_transform.header.stamp = ros::Time::now();
            hand_transform.header.frame_id = last_template_list_.pose[template_index].header.frame_id;

            // publish
            ghost_hand_pub_.publish(hand_transform);

            virtual_link_joint_states_.position[0] = hand_transform.pose.position.x;
            virtual_link_joint_states_.position[1] = hand_transform.pose.position.y;
            virtual_link_joint_states_.position[2] = hand_transform.pose.position.z;
            virtual_link_joint_states_.position[3] = hand_transform.pose.orientation.x;
            virtual_link_joint_states_.position[4] = hand_transform.pose.orientation.y;
            virtual_link_joint_states_.position[5] = hand_transform.pose.orientation.z;
            virtual_link_joint_states_.position[6] = hand_transform.pose.orientation.w;

            moveit::core::jointStateToRobotState(virtual_link_joint_states_, *hand_robot_state_);

            publishHandJointStates(joints, joint_names);

            geometry_msgs::PoseStamped planning_hand_target;
            calcPlanningTarget(template_T_palm, frameid_T_template_, planning_hand_target);

            planning_hand_target_pub_.publish(planning_hand_target);
        }

    }
    
}

void graspWidget::publishHandJointStates(std::vector<float>& finger_joints, std::vector<std::string>& finger_joint_names)
{
    sensor_msgs::JointState joint_states;

    joint_states.header.stamp = ros::Time::now();
    joint_states.header.frame_id = std::string("/")+hand_side_+std::string("_hand_model/")+hand_side_+"_palm";

    for(int i = 0; i < hand_joint_names_.size(); i++)
        joint_states.name.push_back(hand_joint_names_[i]);

    joint_states.position.resize(joint_states.name.size());
    joint_states.effort.resize(joint_states.name.size());
    joint_states.velocity.resize(joint_states.name.size());

    for(unsigned int i = 0; i < joint_states.position.size(); ++i)
    {
        joint_states.effort[i] = 0;
        joint_states.velocity[i] = 0;
        if(finger_joints.size() == 0)
            joint_states.position[i] = 0;
        else{
            int index = 0;
            while (index < finger_joint_names.size() && joint_states.name[i] != finger_joint_names[index]){
                index++;
            }
            if(index < finger_joint_names.size()){
                joint_states.position[i] = finger_joints[index];
            }else
                joint_states.position[i] = 0;
        //ROS_ERROR("Setting Finger Joint %s to %f",joint_states.name[i].c_str(),joint_states.position[i]);
        }
    }

    //ghost_hand_joint_state_pub_.publish(joint_states);
    moveit::core::jointStateToRobotState(joint_states, *hand_robot_state_);
    robot_state::robotStateToRobotStateMsg(*hand_robot_state_, display_state_msg_.state);
    robot_state_vis_pub_.publish(display_state_msg_);
}

// assume this function is called within mutex block
int graspWidget::calcWristTarget(const geometry_msgs::Pose& palm_pose, const geometry_msgs::PoseStamped& template_pose, geometry_msgs::PoseStamped& final_pose)
{
    // Transform wrist_pose into the template pose frame
    tf::Transform template_T_palm;
    tf::Transform world_T_template;
    tf::Transform target_pose;

    template_T_palm.setRotation(tf::Quaternion(palm_pose.orientation.x,palm_pose.orientation.y,palm_pose.orientation.z,palm_pose.orientation.w));
    template_T_palm.setOrigin(tf::Vector3(palm_pose.position.x,palm_pose.position.y,palm_pose.position.z) );
    world_T_template.setRotation(tf::Quaternion(template_pose.pose.orientation.x,template_pose.pose.orientation.y,template_pose.pose.orientation.z,template_pose.pose.orientation.w));
    world_T_template.setOrigin(tf::Vector3(template_pose.pose.position.x,template_pose.pose.position.y,template_pose.pose.position.z) );

    target_pose = world_T_template * template_T_palm * hand_T_palm_.inverse() * this->stitch_template_pose_ * hand_T_palm_;

    tf::Quaternion tg_quat;
    tf::Vector3    tg_vector;
    tg_quat   = target_pose.getRotation();
    tg_vector = target_pose.getOrigin();

    final_pose.pose.orientation.w = tg_quat.getW();
    final_pose.pose.orientation.x = tg_quat.getX();
    final_pose.pose.orientation.y = tg_quat.getY();
    final_pose.pose.orientation.z = tg_quat.getZ();

    final_pose.pose.position.x = tg_vector.getX();
    final_pose.pose.position.y = tg_vector.getY();
    final_pose.pose.position.z = tg_vector.getZ();
    return 0;
}

void graspWidget::calcPlanningTarget(const geometry_msgs::Pose& palm_pose, const geometry_msgs::PoseStamped& template_pose, geometry_msgs::PoseStamped& planning_hand_pose)
{
  tf::Transform wt_pose;
  tf::Transform tp_pose;

  wt_pose.setRotation(tf::Quaternion(palm_pose.orientation.x,palm_pose.orientation.y,palm_pose.orientation.z,palm_pose.orientation.w));
  wt_pose.setOrigin(tf::Vector3(palm_pose.position.x,palm_pose.position.y,palm_pose.position.z) );
  tp_pose.setRotation(tf::Quaternion(template_pose.pose.orientation.x,template_pose.pose.orientation.y,template_pose.pose.orientation.z,template_pose.pose.orientation.w));
  tp_pose.setOrigin(tf::Vector3(template_pose.pose.position.x,template_pose.pose.position.y,template_pose.pose.position.z) );

  tf::Transform target_pose = tp_pose * wt_pose * hand_T_palm_.inverse();

  tf::Quaternion tg_quat = target_pose.getRotation();
  tf::Vector3    tg_vector = target_pose.getOrigin();

  planning_hand_pose.header.frame_id = "world";
  planning_hand_pose.header.stamp = ros::Time::now();

  planning_hand_pose.pose.orientation.w = tg_quat.getW();
  planning_hand_pose.pose.orientation.x = tg_quat.getX();
  planning_hand_pose.pose.orientation.y = tg_quat.getY();
  planning_hand_pose.pose.orientation.z = tg_quat.getZ();

  planning_hand_pose.pose.position.x = tg_vector.getX();
  planning_hand_pose.pose.position.y = tg_vector.getY();
  planning_hand_pose.pose.position.z = tg_vector.getZ();

}

int graspWidget::staticTransform(geometry_msgs::Pose& palm_pose)
{
    tf::Transform o_T_palm;    //describes palm in object's frame
    tf::Transform o_T_pg;       //describes palm_from_graspit in object's frame

    o_T_pg.setRotation(tf::Quaternion(palm_pose.orientation.x,palm_pose.orientation.y,palm_pose.orientation.z,palm_pose.orientation.w));
    o_T_pg.setOrigin(tf::Vector3(palm_pose.position.x,palm_pose.position.y,palm_pose.position.z) );

    o_T_palm = o_T_pg * gp_T_palm_;

    tf::Quaternion hand_quat;
    tf::Vector3    hand_vector;
    hand_quat   = o_T_palm.getRotation();
    hand_vector = o_T_palm.getOrigin();

    palm_pose.position.x = hand_vector.getX();
    palm_pose.position.y = hand_vector.getY();
    palm_pose.position.z = hand_vector.getZ();
    palm_pose.orientation.x = hand_quat.getX();
    palm_pose.orientation.y = hand_quat.getY();
    palm_pose.orientation.z = hand_quat.getZ();
    palm_pose.orientation.w = hand_quat.getW();

    return 0;
}

// transform endeffort to palm pose used by GraspIt
int graspWidget::poseTransform(geometry_msgs::Pose& input_pose, tf::Transform transform)
{
    tf::Transform output_transform;    //describes hand in object's frame
    tf::Transform input_transform;       //describes palm_from_graspit in object's frame

    input_transform.setRotation(tf::Quaternion(input_pose.orientation.x,input_pose.orientation.y,input_pose.orientation.z,input_pose.orientation.w));
    input_transform.setOrigin(tf::Vector3(input_pose.position.x,input_pose.position.y,input_pose.position.z) );

    output_transform = input_transform * transform;

    tf::Quaternion output_quat;
    tf::Vector3    output_vector;
    output_quat   = output_transform.getRotation();
    output_vector = output_transform.getOrigin();

    input_pose.position.x    = output_vector.getX();
    input_pose.position.y    = output_vector.getY();
    input_pose.position.z    = output_vector.getZ();
    input_pose.orientation.x = output_quat.getX();
    input_pose.orientation.y = output_quat.getY();
    input_pose.orientation.z = output_quat.getZ();
    input_pose.orientation.w = output_quat.getW();
    return 0;
}

// transform endeffort to palm pose used by GraspIt
int graspWidget::poseTransform(geometry_msgs::Pose& first_pose, geometry_msgs::Pose& second_pose)
{
    tf::Transform output_transform;
    tf::Transform first_transform;
    tf::Transform second_transform;

    first_transform.setRotation(tf::Quaternion(first_pose.orientation.x,first_pose.orientation.y,first_pose.orientation.z,first_pose.orientation.w));
    first_transform.setOrigin(tf::Vector3(first_pose.position.x,first_pose.position.y,first_pose.position.z) );

    second_transform.setRotation(tf::Quaternion(second_pose.orientation.x,second_pose.orientation.y,second_pose.orientation.z,second_pose.orientation.w));
    second_transform.setOrigin(tf::Vector3(second_pose.position.x,second_pose.position.y,second_pose.position.z) );

    output_transform = first_transform * second_transform;

    tf::Quaternion output_quat;
    tf::Vector3    output_vector;
    output_quat   = output_transform.getRotation();
    output_vector = output_transform.getOrigin();

    first_pose.position.x    = output_vector.getX();
    first_pose.position.y    = output_vector.getY();
    first_pose.position.z    = output_vector.getZ();
    first_pose.orientation.x = output_quat.getX();
    first_pose.orientation.y = output_quat.getY();
    first_pose.orientation.z = output_quat.getZ();
    first_pose.orientation.w = output_quat.getW();
    return 0;
}

void graspWidget::gripperTranslationToPreGraspPose(geometry_msgs::Pose& pose, moveit_msgs::GripperTranslation& trans){
    geometry_msgs::Vector3Stamped direction = trans.direction;
    tf::Transform template_T_hand, vec_in, vec_out;
    ROS_INFO("receiving trans distance: %f; dx: %f, dy: %f, dz: %f", trans.desired_distance, direction.vector.x, direction.vector.y, direction.vector.z);
    float norm = sqrt((direction.vector.x * direction.vector.x) +(direction.vector.y * direction.vector.y) +(direction.vector.z * direction.vector.z));
    if(norm != 0){
        direction.vector.x /= norm;
        direction.vector.y /= norm;
        direction.vector.z /= norm;
    }else{
        ROS_INFO("Norm is ZERO!");
        direction.vector.x = 0 ;
        direction.vector.y = -1;
        direction.vector.z = 0 ;
    }

    direction.vector.x *= -trans.desired_distance; //hand_side_ == "left" ? -trans.desired_distance : trans.desired_distance;  //Change due to Atlas specifics, hand axis are reflected
    direction.vector.y *= -trans.desired_distance; //hand_side_ == "left" ? -trans.desired_distance : trans.desired_distance;  //Change due to Atlas specifics, hand axis are reflected
    direction.vector.z *= -trans.desired_distance;

    ROS_INFO("setting trans; dx: %f, dy: %f, dz: %f", direction.vector.x, direction.vector.y, direction.vector.z);

    template_T_hand.setRotation(tf::Quaternion(pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w));
    template_T_hand.setOrigin(tf::Vector3(0,0,0));

    vec_in.setOrigin(tf::Vector3(direction.vector.x,direction.vector.y,direction.vector.z));
    vec_in.setRotation(tf::Quaternion(0,0,0,1));

    vec_out = template_T_hand * vec_in;

    ROS_INFO("setting result; dx: %f, dy: %f, dz: %f", vec_out.getOrigin().getX(), vec_out.getOrigin().getY(), vec_out.getOrigin().getZ());

    pose.position.x += vec_out.getOrigin().getX();
    pose.position.y += vec_out.getOrigin().getY();
    pose.position.z += vec_out.getOrigin().getZ();
}

int graspWidget::hideHand()
{
    geometry_msgs::PoseStamped hand_transform;
    hand_transform.pose.position.z = 10000;
    hand_transform.pose.orientation.w = 1;
    hand_transform.header.stamp = ros::Time::now();
    hand_transform.header.frame_id = "/world";
    ghost_hand_pub_.publish(hand_transform);

    virtual_link_joint_states_.position[0] = hand_transform.pose.position.x;
    virtual_link_joint_states_.position[1] = hand_transform.pose.position.y;
    virtual_link_joint_states_.position[2] = hand_transform.pose.position.z;
    virtual_link_joint_states_.position[3] = hand_transform.pose.orientation.x;
    virtual_link_joint_states_.position[4] = hand_transform.pose.orientation.y;
    virtual_link_joint_states_.position[5] = hand_transform.pose.orientation.z;
    virtual_link_joint_states_.position[6] = hand_transform.pose.orientation.w;

    moveit::core::jointStateToRobotState(virtual_link_joint_states_, *hand_robot_state_);

    std::vector<float> zero_joints;
    std::vector<std::string> zero_joints_names;
    zero_joints.clear();
    zero_joints_names.clear();
    publishHandJointStates(zero_joints,zero_joints_names);
}

void graspWidget::on_show_grasp_toggled(bool checked)
{
    //publish to sync with other operators
    //hand would jump between 2 positions otherwise if one operator is on show and other is not
    vigir_ocs_msgs::OCSGraspSync msg;
    msg.show_grasp = checked;
    msg.hand = hand_side_;
    msg.host = boost::asio::ip::host_name();
    msg.sync_mode = vigir_ocs_msgs::OCSGraspSync::SHOW_GRASP;
    grasp_sync_pub_.publish(msg);
}

void graspWidget::processGraspSyncCB(const vigir_ocs_msgs::OCSGraspSync::ConstPtr msg)
{
    //accept changes from all operators
    if(msg->sync_mode == vigir_ocs_msgs::OCSGraspSync::SHOW_GRASP)
    {
        //only affect this hand
        if(msg->hand == hand_side_)
        {
            //show/hide the grasp
            show_grasp_ = msg->show_grasp;
            ui->show_grasp_radio->setEnabled(show_grasp_);
            ui->show_pre_grasp_radio->setEnabled(show_grasp_);
            ui->moveToPoseButton->setEnabled(show_grasp_);

            robot_state::robotStateToRobotStateMsg(*hand_robot_state_, display_state_msg_.state);
            robot_state_vis_pub_.publish(display_state_msg_);
        }
    }

    if(msg->sync_mode == vigir_ocs_msgs::OCSGraspSync::GRASP_ID)
    {
        //set new grasp with new id
        selected_grasp_id_ = msg->grasp_id;
        publishHandPose(msg->grasp_id);
    }
}

void graspWidget::processObjectSelection(const vigir_ocs_msgs::OCSObjectSelection::ConstPtr msg)
{
    // only process object selection if I'm the sender
    //if(msg->host != boost::asio::ip::host_name())
    //    return;

    //only update template selection if the main operator has selected
    if(operator_type_ != "main")
        return;

    switch(msg->type)
    {
        case vigir_ocs_msgs::OCSObjectSelection::TEMPLATE:
            {
            // enable template marker
            //ui->templateBox->setDisabled(false);
            ui->graspBox->setDisabled(false);
            std::vector<unsigned char>::iterator it;
            it = std::find(last_template_list_.template_id_list.begin(), last_template_list_.template_id_list.end(), msg->id);
            if(it != last_template_list_.template_id_list.end())
            {
                int tmp = std::distance(last_template_list_.template_id_list.begin(),it);
                ui->templateBox->setCurrentIndex(tmp);
                on_templateBox_activated(ui->templateBox->itemText(tmp));
                selected_template_id_ = tmp;

                if(last_template_list_.template_status_list[ui->templateBox->currentIndex()] == 1) {//is attached
                    ui->attachButton->setEnabled(false);
                    ui->snapTemplateButton->setEnabled(true);
                    ui->detachButton->setEnabled(true);
                }else{
                    ui->attachButton->setEnabled(true);
                    ui->detachButton->setEnabled(false);
                    ui->snapTemplateButton->setEnabled(false);
                }


                //CALL TEMPLATE SERVER ONCE, INSTEAD OF CALLING ON EACH CASE
                last_grasp_srv_.request.template_type = last_template_list_.template_type_list[ui->templateBox->currentIndex()];
                if (!grasp_info_client_.call(last_grasp_srv_))
                {
                    ROS_ERROR("Failed to call service request grasp info");
                }
                last_template_srv_.request.template_id = last_template_list_.template_id_list[ui->templateBox->currentIndex()];
                last_template_srv_.request.hand_side = last_template_srv_.request.BOTH_HANDS;
                if (!template_info_client_.call(last_template_srv_))
                {
                    ROS_ERROR("Failed to call service request template info");
                }
                on_templateBox_activated(ui->templateBox->itemText(tmp));

                if(selected_grasp_id_ != -1 && show_grasp_)
                    publishHandPose(selected_grasp_id_);
            }
            }
            break;
        // not a template
        default:
            {
            selected_template_id_ = -1;
            ui->templateBox->setCurrentIndex(-1);
            ui->graspBox->setCurrentIndex(-1);
            ui->affordanceBox->setCurrentIndex(-1);            
            ui->usabilityBox->setCurrentIndex(-1);
            ui->graspBox->setDisabled(true);
            ui->affordanceBox->setDisabled(true);
            ui->usabilityBox->setDisabled(true);
            ui->moveToPoseButton->setDisabled(true);
            ui->attachButton->setDisabled(true);
            ui->detachButton->setDisabled(true);
            ui->show_grasp->setDisabled(true);
            ui->graspPose->setDisabled(true);
            ui->show_grasp->setChecked(false);
            ui->show_grasp_radio->setEnabled(false);
            ui->show_pre_grasp_radio->setEnabled(false);
            ui->affordanceButton->setEnabled(false);
            ui->keepOrientationBox->setEnabled(false);
            ui->displacementBox->setEnabled(false);
            ui->pitchSpinBox->setEnabled(false);
            ui->snapTemplateButton->setEnabled(false);
            }
            break;
    }
}


Ui::graspWidget * graspWidget::getUi()
{
    return ui;
}
QLayout* graspWidget::getMainLayout()
{
    return ui->mainLayout;
}

void graspWidget::on_attachButton_clicked()
{
    ui->detachButton->setEnabled(true);
    ui->snapTemplateButton->setEnabled(true);
    vigir_grasp_msgs::GraspSelection msg;
    msg.template_type.data = last_template_list_.template_type_list[ui->templateBox->currentIndex()];
    msg.template_id.data   = last_template_list_.template_id_list[ui->templateBox->currentIndex()];
    msg.grasp_id.data      = ui->graspBox->currentText().toInt();
    msg.final_pose         = true;  //using final_pose as is_stitched?
    template_stitch_request_pub_.publish(msg);
}

void graspWidget::on_detachButton_clicked()
{
    ui->attachButton->setEnabled(true);
    ui->detachButton->setEnabled(false);
    ui->snapTemplateButton->setEnabled(false);
    this->stitch_template_pose_.setIdentity();
    vigir_grasp_msgs::TemplateSelection msg;
    msg.template_id.data = selected_template_id_;
    if(detach_object_pub_)
        detach_object_pub_.publish(msg);
}

void graspWidget::on_displacementBox_valueChanged(double value){
    //Set displacement value on afordance msg
    if(current_affordance_.type == "circular")
        current_affordance_.displacement = value * 0.0174532925; // UI in deg, msg in rad;
    else
        current_affordance_.displacement = value;
}

void graspWidget::on_pitchSpinBox_valueChanged(double value){
    //Set displacement value on afordance msg
    current_affordance_.pitch = value;
}

void graspWidget::on_keepOrientationBox_toggled(bool checked){
    //set keep orientation on affordance msg
    current_affordance_.keep_orientation = checked;
}

void graspWidget::on_affordanceButton_clicked()
{
    //Create message for manipulation controller
    last_template_srv_.request.template_id = last_template_list_.template_id_list[ui->templateBox->currentIndex()];
    last_template_srv_.request.hand_side   = last_template_srv_.request.BOTH_HANDS;
    if (!template_info_client_.call(last_template_srv_))
    {
        ROS_ERROR("Failed to call service request template info");
    }else{
        current_affordance_.waypoints = last_template_srv_.response.template_type_information.affordances[current_affordance_.id].waypoints;

        current_affordance_.speed        = ui->speedBox->value();

        if(affordance_selection_pub_)
            affordance_selection_pub_.publish(current_affordance_);
    }
}

void graspWidget::on_snapTemplateButton_clicked()
{
    vigir_grasp_msgs::TemplateSelection msg;
    msg.template_id.data = last_template_list_.template_id_list[ui->templateBox->currentIndex()];
    if(snap_template_pub_)
        snap_template_pub_.publish(msg);
    follow_ban_ = true;
}

void graspWidget::on_graspPose_clicked()
{
    vigir_grasp_msgs::GraspState grasp;
    size_t size = last_grasp_srv_.response.grasp_information.grasps.size();
    if(size == 0)
        ROS_ERROR_STREAM("No grasps found for this template");
    else{
        size_t index = 0;
        for(; index < size; ++index)
        {
            if(std::atoi(last_grasp_srv_.response.grasp_information.grasps[index].id.c_str()) == selected_grasp_id_){
                grasp.grasp.grasp_posture = last_grasp_srv_.response.grasp_information.grasps[index].grasp_posture;
            }
        }

        if(index > size)
            ROS_ERROR_STREAM("Template server response id: " << last_grasp_srv_.response.grasp_information.grasps[index].id << " while searching for id: " << selected_grasp_id_);
        else{
            grasp.grasp_state.data = grasp.GRASP_ID;
            grasp_command_pub_.publish(grasp);
        }

    }
}
