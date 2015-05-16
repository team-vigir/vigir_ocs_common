#include <ros/package.h>

#include "ghost_control_widget.h"
#include "ui_ghost_control_widget.h"
#include "stdio.h"
#include <iostream>
#include <QPainter>
#include <QtGui>
#include <QSignalMapper>

#include <boost/exception/to_string.hpp>

#include <flor_grasp_msgs/InverseReachabilityForGraspRequest.h>
#include <flor_ocs_msgs/OCSInverseReachability.h>
#include <flor_ocs_msgs/WindowCodes.h>

std::vector<unsigned char> GhostControlWidget::saved_state_planning_group_;
unsigned char GhostControlWidget::saved_state_use_torso_;
unsigned char GhostControlWidget::saved_state_lock_pelvis_;
unsigned char GhostControlWidget::saved_state_position_only_ik_;
unsigned char GhostControlWidget::saved_state_use_drake_ik_;

GhostControlWidget::GhostControlWidget(QWidget *parent) :
    QWidget(parent)
    , selected_template_id_(-1)
    , selected_pose_id_(-1),
    ui(new Ui::GhostControlWidget)
{    
    ui->setupUi(this);

    saved_state_use_torso_ = 0;
    saved_state_lock_pelvis_ = 0;
    saved_state_position_only_ik_ = 0;
    saved_state_use_drake_ik_ = 0;

    //publish to normal state to be read by outside
    state_use_torso_pub_ = nh_.advertise<std_msgs::Bool>( "/flor/ocs/ghost/state_use_torso", 1, false );
    state_lock_pelvis_pub_ = nh_.advertise<std_msgs::Bool>( "/flor/ocs/ghost/state_lock_pelvis", 1, false );
    state_position_only_ik_pub_ = nh_.advertise<std_msgs::Bool>( "/flor/ocs/ghost/state_position_only_ik", 1, false );
    state_use_drake_ik_pub_ = nh_.advertise<std_msgs::Bool>( "/flor/ocs/ghost/state_use_drake_ik", 1, false );
    state_snap_ghost_to_robot_pub_ = nh_.advertise<std_msgs::Bool>( "/flor/ocs/ghost/state_snap_ghost_to_robot", 1, false );

    //subscribe to template list
    template_list_sub_ = nh_.subscribe<flor_ocs_msgs::OCSTemplateList>(    "/template/list",5, &GhostControlWidget::processTemplateList, this );

    // advertise the topic to publish state configurations    
    reset_pelvis_pub_ = nh_.advertise<std_msgs::Bool>( "/flor/ocs/ghost/reset_pelvis", 1, false );
    send_pelvis_pub_ = nh_.advertise<std_msgs::Bool>( "/flor/ocs/ghost/send_pelvis_to_footstep", 1, false );
    send_ghost_cartesian_pub_ = nh_.advertise<std_msgs::Bool>( "/flor/ocs/ghost/send_cartesian", 1, false );

    // advertise set pose buttons
    set_to_target_pose_pub_   = nh_.advertise<std_msgs::String>( "/flor/ocs/planning/plan_to_pose_state", 1, false );
    set_to_target_config_pub_ = nh_.advertise<std_msgs::String>( "/flor/ocs/planning/plan_to_joint_state", 1, false );

    send_inverse_rechability_req_pub_ = nh_.advertise<flor_ocs_msgs::OCSInverseReachability>( "/flor/ocs/ghost/inverse_rechability", 1, false );

    send_ghost_to_template_pub_ = nh_.advertise<geometry_msgs::PoseStamped>( "/flor/ocs/ghost/set_pose", 1, false );
    send_template_to_behavior_pub_ = nh_.advertise<geometry_msgs::PoseStamped>( "/flor/ocs/ghost/behavior_pose", 1, false );;   

    timer.start(33, this);

    //ui->position_only_ik_->hide();

    window_control_sub = nh_.subscribe<std_msgs::Int8>( "/flor/ocs/window_control", 5, &GhostControlWidget::processWindowControl, this );
    window_control_pub = nh_.advertise<std_msgs::Int8>("/flor/ocs/window_control", 1, false);

    //TEMPLATE SERVER STUFF
    grasp_info_client_    = nh_.serviceClient<vigir_object_template_msgs::GetGraspInfo>("/grasp_info");
    template_info_client_ = nh_.serviceClient<vigir_object_template_msgs::GetTemplateStateAndTypeInfo>("/template_info");


    //Context menu fix
    snap_ghost_sub_ = nh_.subscribe<std_msgs::Bool>("/flor/ocs/ghost/snap_ghost_context",5, &GhostControlWidget::snapGhostContextMenu, this );
    use_torso_sub_ = nh_.subscribe<std_msgs::Bool>("/flor/ocs/ghost/use_torso_context",5, &GhostControlWidget::useTorsoContextMenu, this );


    //Restore State
    //this->show();
    QSettings settings("OCS", "joint_limit");
    this->restoreGeometry(settings.value("mainWindowGeometry").toByteArray());
    this->geometry_ = this->geometry();
    // create docks, toolbars, etc...
    //this->restoreState(settings.value("mainWindowState").toByteArray());

    addHotkeys();
}

GhostControlWidget::~GhostControlWidget()
{
    delete ui;
}

void GhostControlWidget::closeEvent(QCloseEvent * event)
{
    QSettings settings("OCS", "joint_limit");
    settings.setValue("mainWindowGeometry", this->saveGeometry());    
    std_msgs::Int8 msg;
    msg.data = -WINDOW_GHOST_CONFIG;
    window_control_pub.publish(msg);
    event->ignore();
}

void GhostControlWidget::resizeEvent(QResizeEvent * event)
{
    QSettings settings("OCS", "joint_limit");
    settings.setValue("mainWindowGeometry", this->saveGeometry());    
}

void GhostControlWidget::moveEvent(QMoveEvent * event)
{
    QSettings settings("OCS", "joint_limit");
    settings.setValue("mainWindowGeometry", this->saveGeometry());    
}

void GhostControlWidget::timerEvent(QTimerEvent *event)
{
	// check if ros is still running; if not, just kill the application
    if(!ros::ok())
        qApp->quit();
    
    //Spin at beginning of Qt timer callback, so current ROS time is retrieved
    ros::spinOnce();
}

void GhostControlWidget::processTemplateList( const flor_ocs_msgs::OCSTemplateList::ConstPtr& list)
{
    //std::cout << "Template list received containing " << list->template_id_list.size() << " elements" << std::cout;
    // save last template list
    last_template_list_ = *list;

    // enable boxes and buttons
    if(list->template_list.size() > 0)
    {
        ui->templateBox->setDisabled(false);
        ui->graspBox->setDisabled(false);
        ui->send_ghost_to_template_button->setDisabled(false);
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
        ui->graspBox->clear();
        selected_template_id_ = -1;
        selected_pose_id_ = -1;
        ui->graspBox->setEnabled(false);
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
    }
}

void GhostControlWidget::on_templateBox_activated(const QString &arg1)
{
    // update the selected template id
    QString template_id = ui->templateBox->currentText();
    template_id.remove(template_id.indexOf(": "),template_id.length()-template_id.indexOf(": "));
    selected_template_id_ = template_id.toInt();

    //std::cout << "updating the ghost widget pose selection box contents" << std::endl;
    // clean grasp box
    ui->graspBox->clear();
    selected_pose_id_ = -1;

    //CALLING THE TEMPLATE SERVER
    vigir_object_template_msgs::GetTemplateStateAndTypeInfo srv;
    srv.request.template_id = selected_template_id_;
    srv.request.hand_side   = srv.request.BOTH_HANDS;
    if (!template_info_client_.call(srv))
    {
        ROS_ERROR("Failed to call service request grasp info");
    }else{
        for(int index = 0; index < srv.response.template_type_information.stand_poses.size(); index++)
        {
            ui->graspBox->addItem(QString::number(srv.response.template_type_information.stand_poses[index].id));
        }
        if(ui->templateBox->count() > 0)
            selected_pose_id_ = ui->graspBox->itemText(0).toInt();
    }
}

void GhostControlWidget::on_graspBox_activated(const QString &arg1)
{
    selected_pose_id_ = arg1.toInt();
}

void GhostControlWidget::snapClicked()
{    
    std_msgs::Bool msg;
    msg.data = true;
    state_snap_ghost_to_robot_pub_.publish(msg);
}

void GhostControlWidget::sendTargetPoseClicked()
{
    std_msgs::String cmd;

    cmd.data = this->getGroupNameForSettings(saved_state_planning_group_);

    set_to_target_pose_pub_.publish(cmd);
}

void GhostControlWidget::sendTargetConfigClicked()
{
    std_msgs::String cmd;

    cmd.data = this->getGroupNameForSettings(saved_state_planning_group_);

    set_to_target_config_pub_.publish(cmd);
}

void GhostControlWidget::resetPelvisClicked()
{
    //ROS_ERROR("RESET PELVIS!");
    std_msgs::Bool cmd;
    cmd.data = true;
    reset_pelvis_pub_.publish(cmd);
}


void GhostControlWidget::on_planning_torso__clicked()
{
    saved_state_planning_group_.clear();

    saved_state_planning_group_.push_back(false);
    saved_state_planning_group_.push_back(false);
    saved_state_planning_group_.push_back(ui->planning_torso_->isChecked());

    //only send use torso
    saved_state_use_torso_ = ui->planning_torso_->isChecked();
    std_msgs::Bool cmd;
    cmd.data = saved_state_use_torso_;
    state_use_torso_pub_.publish(cmd);
}

//temp fix before making ghost manager, ignoring message, just using these callbacks as a way to signal
// this reference from main view without a local reference, ignore msg just a placeholder
//public wrapper for context menu callback
void GhostControlWidget::snapGhostContextMenu(const std_msgs::BoolConstPtr &msg)
{
    snapClicked();
}

void GhostControlWidget::useTorsoContextMenu(const std_msgs::BoolConstPtr &msg)
{
    ui->planning_torso_->toggle();
}
//----End temp fix-----//

void GhostControlWidget::on_position_only_ik__clicked()
{
    saved_state_position_only_ik_ = ui->position_only_ik_->isChecked();

    std_msgs::Bool msg;
    msg.data = saved_state_position_only_ik_;
    state_position_only_ik_pub_.publish(msg);
}


void GhostControlWidget::on_lock_pelvis__clicked()
{
    saved_state_lock_pelvis_ = ui->lock_pelvis_->isChecked();

    std_msgs::Bool msg;
    msg.data = saved_state_lock_pelvis_;
    state_lock_pelvis_pub_.publish(msg);
}

void GhostControlWidget::on_use_drake_ik__clicked()
{
    saved_state_use_drake_ik_ = ui->use_drake_ik_->isChecked();

    std_msgs::Bool msg;
    msg.data = saved_state_use_drake_ik_;
    state_use_drake_ik_pub_.publish(msg);
}

void GhostControlWidget::on_send_left_pose_button__clicked()
{
    std_msgs::String cmd;

    if(ui->planning_torso_->isChecked())
        cmd.data = "l_arm_with_torso_group";
    else
        cmd.data = "l_arm_group";

    set_to_target_pose_pub_.publish(cmd);
}


void GhostControlWidget::on_send_right_pose_button__clicked()
{
    std_msgs::String cmd;

    if(ui->planning_torso_->isChecked())
        cmd.data = "r_arm_with_torso_group";
    else
        cmd.data = "r_arm_group";

    set_to_target_pose_pub_.publish(cmd);
}



void GhostControlWidget::on_send_left_configuration_button__clicked()
{
    std_msgs::String cmd;

    if(ui->planning_torso_->isChecked())
        cmd.data = "l_arm_with_torso_group";
    else
        cmd.data = "l_arm_group";

    set_to_target_config_pub_.publish(cmd);
}


void GhostControlWidget::on_send_right_configuration_button__clicked()
{
    std_msgs::String cmd;

    if(ui->planning_torso_->isChecked())
        cmd.data = "r_arm_with_torso_group";
    else
        cmd.data = "r_arm_group";

    set_to_target_config_pub_.publish(cmd);
}


void GhostControlWidget::on_send_upper_body_button__clicked()
{
    std_msgs::String cmd;
    cmd.data = "both_arms_with_torso_group";

    set_to_target_config_pub_.publish(cmd);
}

void GhostControlWidget::on_send_whole_body_button__clicked()
{
    std_msgs::String cmd;

    cmd.data = "whole_body_group";

    set_to_target_config_pub_.publish(cmd);
}

void GhostControlWidget::on_send_left_ghost_hand_button__clicked()
{
    flor_grasp_msgs::InverseReachabilityForGraspRequest inv_grasp_req;
    inv_grasp_req.hand_side = flor_grasp_msgs::InverseReachabilityForGraspRequest::HAND_LEFT;
    flor_ocs_msgs::OCSInverseReachability cmd;
    cmd.request = inv_grasp_req;
    cmd.use_pose = flor_ocs_msgs::OCSInverseReachability::GHOST_HAND;
    send_inverse_rechability_req_pub_.publish(cmd);
}

void GhostControlWidget::on_send_right_ghost_hand_button__clicked()
{
    flor_grasp_msgs::InverseReachabilityForGraspRequest inv_grasp_req;
    inv_grasp_req.hand_side = flor_grasp_msgs::InverseReachabilityForGraspRequest::HAND_RIGHT;
    flor_ocs_msgs::OCSInverseReachability cmd;
    cmd.request = inv_grasp_req;
    cmd.use_pose = flor_ocs_msgs::OCSInverseReachability::GHOST_HAND;
    send_inverse_rechability_req_pub_.publish(cmd);
}


void GhostControlWidget::addHotkeys()
{
    HotkeyManager::Instance()->addHotkeyFunction("ctrl+shift+e",boost::bind(&GhostControlWidget::on_send_left_cartesian_button__clicked,this));
    HotkeyManager::Instance()->addHotkeyFunction("ctrl+shift+r",boost::bind(&GhostControlWidget::on_send_right_cartesian_button__clicked,this));
    HotkeyManager::Instance()->addHotkeyFunction("ctrl+e",boost::bind(&GhostControlWidget::on_send_left_configuration_button__clicked,this));
    HotkeyManager::Instance()->addHotkeyFunction("ctrl+r",boost::bind(&GhostControlWidget::on_send_right_configuration_button__clicked,this));
    HotkeyManager::Instance()->addHotkeyFunction("ctrl+f",boost::bind(&GhostControlWidget::on_send_upper_body_button__clicked,this));
    HotkeyManager::Instance()->addHotkeyFunction("ctrl+s",boost::bind(&GhostControlWidget::snapClicked,this));
}

void GhostControlWidget::on_send_left_cartesian_button__clicked()
{
    std_msgs::Bool cmd;
    cmd.data = false;
    send_ghost_cartesian_pub_.publish(cmd);
}

void GhostControlWidget::on_send_right_cartesian_button__clicked()
{
    std_msgs::Bool cmd;
    cmd.data = true;
    send_ghost_cartesian_pub_.publish(cmd);
}

void GhostControlWidget::on_pushButton_clicked()
{
    std_msgs::Bool cmd;
    cmd.data = true;
    send_pelvis_pub_.publish(cmd);
}

std::string GhostControlWidget::getGroupNameForSettings(const std::vector<unsigned char>& settings)
{
  bool left = settings[0];
  bool right = settings[1];
  bool torso = settings[2];

  if(left && !right && !torso)
      return "l_arm_group";
  else if(left && !right && torso)
      return "l_arm_with_torso_group";
  else if(!left && right && !torso)
      return "r_arm_group";
  else if(!left && right && torso)
      return "r_arm_with_torso_group";
  else if(left && right && !torso)
      return "both_arms_group";
  else if(left && right && torso)
      return "both_arms_with_torso_group";

  return "INVALID_GROUP";
}

void GhostControlWidget::on_send_ghost_to_template_button_clicked()
{

    //TO USE CERTAIN ARM GROUP
//    std_msgs::String cmd;

//    cmd.data = "l_arm_group";

//    set_to_target_pose_pub_.publish(cmd);

    geometry_msgs::PoseStamped stand_pose;

    //CALLING THE TEMPLATE SERVER
    vigir_object_template_msgs::GetTemplateStateAndTypeInfo srv;
    srv.request.template_id = last_template_list_.template_id_list[ui->templateBox->currentIndex()];
    srv.request.hand_side   = srv.request.BOTH_HANDS;
    if (!template_info_client_.call(srv))
    {
        ROS_ERROR("Failed to call service request grasp info");
    }
    else
    {
        //cant send ghost robot if no pose
        if(srv.response.template_type_information.stand_poses.size() == 0)
            return;
        for(int index = 0; index < srv.response.template_type_information.stand_poses.size(); index++)
        {
            if(srv.response.template_type_information.stand_poses[index].id == selected_pose_id_){
                stand_pose = srv.response.template_type_information.stand_poses[index].pose;
                break;
            }
        }
        if(send_ghost_to_template_pub_)
        {
//            geometry_msgs::PoseStamped pose;
//            pose.header.frame_id = "/world";
//            pose.header.stamp = ros::Time::now();
//            calcTargetPose(last_template_list_.pose[ui->templateBox->currentIndex()].pose,
//                           stand_pose.pose,
//                           pose.pose);
            send_ghost_to_template_pub_.publish(stand_pose);
        }
        else{
            ROS_ERROR("No Publisher for ghost to template pose");
        }
    }
}

int GhostControlWidget::calcTargetPose(const geometry_msgs::Pose& pose_1, const geometry_msgs::Pose& pose_2, geometry_msgs::Pose& pose_result)
{
    // Transform wrist_pose into the template pose frame
    //   @TODO        "wrist_target_pose.pose   = T(template_pose)*wrist_pose";
    tf::Transform tf_pose_1;
    tf::Transform tf_pose_2;
    tf::Transform tf_pose_result;

    tf_pose_1.setRotation(tf::Quaternion(pose_1.orientation.x,pose_1.orientation.y,pose_1.orientation.z,pose_1.orientation.w));
    tf_pose_1.setOrigin(tf::Vector3(pose_1.position.x,pose_1.position.y,pose_1.position.z) );
    tf_pose_2.setRotation(tf::Quaternion(pose_2.orientation.x,pose_2.orientation.y,pose_2.orientation.z,pose_2.orientation.w));
    tf_pose_2.setOrigin(tf::Vector3(pose_2.position.x,pose_2.position.y,pose_2.position.z) );

    tf_pose_result = tf_pose_1 * tf_pose_2;

    tf::Quaternion pose_result_quat;
    tf::Vector3    pose_result_vector;
    pose_result_quat   = tf_pose_result.getRotation();
    pose_result_vector = tf_pose_result.getOrigin();

    pose_result.orientation.w = pose_result_quat.getW();
    pose_result.orientation.x = pose_result_quat.getX();
    pose_result.orientation.y = pose_result_quat.getY();
    pose_result.orientation.z = pose_result_quat.getZ();

    pose_result.position.x = pose_result_vector.getX();
    pose_result.position.y = pose_result_vector.getY();
    pose_result.position.z = pose_result_vector.getZ();
    return 0;
}

void GhostControlWidget::processWindowControl(const std_msgs::Int8::ConstPtr& msg)
{
    if(!isVisible() && msg->data == WINDOW_GHOST_CONFIG)
    {
        this->show();
        this->setGeometry(geometry_);
    }
    else if(isVisible() && (!msg->data || msg->data == -WINDOW_GHOST_CONFIG))
    {
        geometry_ = this->geometry();
        this->hide();
    }
}









