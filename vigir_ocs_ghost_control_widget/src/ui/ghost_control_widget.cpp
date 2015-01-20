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
std::vector<unsigned char> GhostControlWidget::saved_state_pose_source_;
std::vector<unsigned char> GhostControlWidget::saved_state_world_lock_;
unsigned char GhostControlWidget::saved_state_collision_avoidance_;
unsigned char GhostControlWidget::saved_state_lock_pelvis_;
unsigned char GhostControlWidget::saved_state_position_only_ik_;

GhostControlWidget::GhostControlWidget(QWidget *parent) :
    QWidget(parent)
    , selected_template_id_(-1)
    , selected_grasp_id_(-1),
    ui(new Ui::GhostControlWidget)
{    
    ui->setupUi(this);

    saveState();

    // subscribe to the topic to load state configurations
    state_sub_ = nh_.subscribe<flor_ocs_msgs::OCSGhostControl>( "/flor/ocs/ghost_ui_state", 5, &GhostControlWidget::processState, this );
    //subscribe to template list
    template_list_sub_ = nh_.subscribe<flor_ocs_msgs::OCSTemplateList>(    "/template/list",5, &GhostControlWidget::processTemplateList, this );

    // advertise the topic to publish state configurations
    state_pub_ = nh_.advertise<flor_ocs_msgs::OCSGhostControl>( "/flor/ocs/ghost_ui_state", 1, false );
    reset_pelvis_pub_ = nh_.advertise<std_msgs::Bool>( "/flor/ocs/reset_pelvis", 1, false );
    send_pelvis_pub_ = nh_.advertise<std_msgs::Bool>( "/flor/ocs/send_pelvis_to_footstep", 1, false );
    send_ghost_cartesian_pub_ = nh_.advertise<std_msgs::Bool>( "/flor/ocs/send_cartesian", 1, false );

    // advertise set pose buttons
    set_to_target_pose_pub_   = nh_.advertise<std_msgs::String>( "/flor/ocs/planning/plan_to_pose_state", 1, false );
    set_to_target_config_pub_ = nh_.advertise<std_msgs::String>( "/flor/ocs/planning/plan_to_joint_state", 1, false );

    send_inverse_rechability_req_pub_ = nh_.advertise<flor_ocs_msgs::OCSInverseReachability>( "/flor/ocs/ghost/inverse_rechability", 1, false );

    send_ghost_to_template_pub_ = nh_.advertise<geometry_msgs::PoseStamped>( "/flor/ocs/ghost/set_pose", 1, false );
    send_template_to_behavior_pub_ = nh_.advertise<geometry_msgs::PoseStamped>( "/flor/ocs/ghost/behavior_pose", 1, false );;

    key_event_sub_ = nh_.subscribe<flor_ocs_msgs::OCSKeyEvent>( "/flor/ocs/key_event", 5, &GhostControlWidget::processNewKeyEvent, this );

    timer.start(33, this);

    //ui->position_only_ik_->hide();

    window_control_sub = nh_.subscribe<std_msgs::Int8>( "/flor/ocs/window_control", 5, &GhostControlWidget::processWindowControl, this );
    window_control_pub = nh_.advertise<std_msgs::Int8>("/flor/ocs/window_control", 1, false);

    //TEMPLATE SERVER STUFF
    grasp_info_client_    = nh_.serviceClient<vigir_object_template_msgs::GetGraspInfo>("/grasp_info");
    template_info_client_ = nh_.serviceClient<vigir_object_template_msgs::GetTemplateStateAndTypeInfo>("/template_info");


    //Restore State
    //this->show();
    QSettings settings("OCS", "joint_limit");
    this->restoreGeometry(settings.value("mainWindowGeometry").toByteArray());
    this->geometry_ = this->geometry();
    // create docks, toolbars, etc...
    //this->restoreState(settings.value("mainWindowState").toByteArray());
}

GhostControlWidget::~GhostControlWidget()
{
    delete ui;
}

void GhostControlWidget::closeEvent(QCloseEvent * event)
{
    QSettings settings("OCS", "joint_limit");
    settings.setValue("mainWindowGeometry", this->saveGeometry());
    //settings.setValue("mainWindowState", this->saveState());
    std_msgs::Int8 msg;
    msg.data = -WINDOW_GHOST_CONFIG;
    window_control_pub.publish(msg);
    event->ignore();
}

void GhostControlWidget::resizeEvent(QResizeEvent * event)
{
    QSettings settings("OCS", "joint_limit");
    settings.setValue("mainWindowGeometry", this->saveGeometry());
    //settings.setValue("mainWindowState", this->saveState());

}

void GhostControlWidget::moveEvent(QMoveEvent * event)
{
    QSettings settings("OCS", "joint_limit");
    settings.setValue("mainWindowGeometry", this->saveGeometry());
    //settings.setValue("mainWindowState", this->saveState());

}

void GhostControlWidget::timerEvent(QTimerEvent *event)
{
	// check if ros is still running; if not, just kill the application
    if(!ros::ok())
        qApp->quit();
    
    //Spin at beginning of Qt timer callback, so current ROS time is retrieved
    ros::spinOnce();
}

void GhostControlWidget::processState(const flor_ocs_msgs::OCSGhostControl::ConstPtr &msg)
{
    // apply state coming from message
    loadState(msg->planning_group,msg->pose_source,msg->world_lock,msg->collision_avoidance,msg->lock_pelvis);

    // save as the last used state
    saveState();
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
        selected_grasp_id_ = -1;
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

    std::cout << "updating the ghost widget pose selection box contents" << std::endl;
    // clean grasp box
    ui->graspBox->clear();
    selected_grasp_id_ = -1;

    //CHANGE TO CALL FOR GRASP INFO FROM TEMPLATE SERVER

//    // add grasps to the grasp combo box
//    for(int index = 0; index < pose_db_.size(); index++)
//    {
//        QString tmp = arg1;
//        tmp.remove(0,tmp.indexOf(": ")+2);
//        std::cout << "comparing db " << pose_db_[index].template_name << " to " << tmp.toStdString() << std::endl;

//        if(pose_db_[index].template_name == tmp.toStdString())
//        {
//            std::cout << "Found pose for template" << std::endl;
//            ui->graspBox->addItem(QString::number(pose_db_[index].pose_id));
//        }
//    }

    if(ui->templateBox->count() > 0)
        selected_grasp_id_ = ui->graspBox->itemText(0).toInt();
}

void GhostControlWidget::on_graspBox_activated(const QString &arg1)
{
    std::cout << " pose selection = " << arg1.toStdString() << std::endl;
    selected_grasp_id_ = arg1.toInt();
}

void GhostControlWidget::publishState( bool snap )
{
    flor_ocs_msgs::OCSGhostControl cmd;
    cmd.planning_group = saved_state_planning_group_;
    cmd.pose_source = saved_state_pose_source_;
    cmd.world_lock = saved_state_world_lock_;
    //cmd.collision_avoidance = saved_state_collision_avoidance_;
    cmd.lock_pelvis = saved_state_lock_pelvis_;
    cmd.snap = snap;
    cmd.position_only_ik = saved_state_position_only_ik_;

    state_pub_.publish(cmd);
}

void GhostControlWidget::saveState()
{
    saved_state_planning_group_.clear();
    saved_state_pose_source_.clear();
    saved_state_world_lock_.clear();

    saved_state_planning_group_.push_back(false);
    saved_state_planning_group_.push_back(false);
    saved_state_planning_group_.push_back(ui->planning_torso_->isChecked());

    //if(ui->left_template_lock->isChecked())
    //{
    //    saved_state_pose_source_.push_back(1);
    //    saved_state_world_lock_.push_back(1);
    //}
    //else
    {
        saved_state_pose_source_.push_back(0);
        //saved_state_world_lock_.push_back(ui->left_marker_lock->isChecked());
    }

//    if(ui->right_template_lock->isChecked())
//    {
//        saved_state_pose_source_.push_back(1);
//        saved_state_world_lock_.push_back(1);
//    }
//    else
    {
        saved_state_pose_source_.push_back(0);
        //saved_state_world_lock_.push_back(ui->right_marker_lock->isChecked());
    }

    saved_state_lock_pelvis_ = ui->lock_pelvis_->isChecked();
    saved_state_position_only_ik_ = ui->position_only_ik_->isChecked();
}

// default arguments are class members for saved state
void GhostControlWidget::loadState(std::vector<unsigned char> planning_group, std::vector<unsigned char> pose_source,
                                   std::vector<unsigned char> world_lock, unsigned char collision_avoidance,
                                   unsigned char lock_pelvis)
{
    /*ui->planning_left_->setChecked(planning_group[0]);
    ui->planning_right_->setChecked(planning_group[1]);
    ui->planning_torso_->setChecked(planning_group[2]);

    ui->pose_left_->setCurrentIndex(pose_source[0]);
    ui->pose_right_->setCurrentIndex(pose_source[1]);
    //ui->pose_torso_->setCurrentIndex(pose_source[2]);

    //ui->lock_left_->setChecked(world_lock[0]);
    //ui->lock_right_->setChecked(world_lock[1]);
    //ui->lock_torso_->setChecked(world_lock[2]);

    //ui->collision_->setChecked(collision_avoidance);

    ui->lock_pelvis_->setChecked(lock_pelvis);*/
}

void GhostControlWidget::applyClicked()
{
    saveState();
    publishState();
}

void GhostControlWidget::cancelClicked()
{
    loadState();
}

void GhostControlWidget::snapClicked()
{
    std::cout << "Snap Clicked" << std::endl;
    publishState(true);
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

void GhostControlWidget::on_planning_left__clicked()
{
    saveState();
    publishState();
}

void GhostControlWidget::on_planning_torso__clicked()
{
    saveState();
    publishState();
}

void GhostControlWidget::on_position_only_ik__clicked()
{
    saveState();
    publishState();
}

void GhostControlWidget::on_planning_right__clicked()
{
    saveState();
    publishState();
}

void GhostControlWidget::on_lock_left__clicked()
{
    saveState();
    publishState();
}

void GhostControlWidget::on_lock_torso__clicked()
{
    saveState();
    publishState();
}

void GhostControlWidget::on_lock_right__clicked()
{
    saveState();
    publishState();
}

void GhostControlWidget::on_pose_left__currentIndexChanged(int index)
{
    saveState();
    publishState();
}

void GhostControlWidget::on_pose_torso__currentIndexChanged(int index)
{
    saveState();
    publishState();
}

void GhostControlWidget::on_pose_right__currentIndexChanged(int index)
{
    saveState();
    publishState();
}

void GhostControlWidget::on_lock_pelvis__clicked()
{
    saveState();
    publishState();
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

void GhostControlWidget::on_send_left_torso_pose_button__clicked()
{
    std_msgs::String cmd;

    cmd.data = "l_arm_with_torso_group";

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

void GhostControlWidget::on_send_right_torso_pose_button__clicked()
{
    std_msgs::String cmd;

    cmd.data = "r_arm_with_torso_group";

    set_to_target_pose_pub_.publish(cmd);
}

//void GhostControlWidget::on_left_no_lock_toggled(bool checked)
//{
//    saveState();
//    publishState();
//}

//void GhostControlWidget::on_left_marker_lock_toggled(bool checked)
//{
//    saveState();
//    publishState();
//}

//void GhostControlWidget::on_left_template_lock_toggled(bool checked)
//{
//    saveState();
//    publishState();
//}

//void GhostControlWidget::on_right_no_lock_toggled(bool checked)
//{
//    saveState();
//    publishState();
//}

//void GhostControlWidget::on_right_marker_lock_toggled(bool checked)
//{
//    saveState();
//    publishState();
//}

//void GhostControlWidget::on_right_template_lock_toggled(bool checked)
//{
//    saveState();
//    publishState();
//}

void GhostControlWidget::on_send_left_configuration_button__clicked()
{
    std_msgs::String cmd;

    if(ui->planning_torso_->isChecked())
        cmd.data = "l_arm_with_torso_group";
    else
        cmd.data = "l_arm_group";

    set_to_target_config_pub_.publish(cmd);
}

void GhostControlWidget::on_send_left_torso_configuration_button__clicked()
{
    std_msgs::String cmd;

    cmd.data = "l_arm_with_torso_group";

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

void GhostControlWidget::on_send_right_torso_configuration_button__clicked()
{
    std_msgs::String cmd;

    cmd.data = "r_arm_with_torso_group";

    set_to_target_config_pub_.publish(cmd);
}

void GhostControlWidget::on_send_upper_body_button__clicked()
{
    std_msgs::String cmd;

    cmd.data = "both_arms_with_torso_group";

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

void GhostControlWidget::processNewKeyEvent(const flor_ocs_msgs::OCSKeyEvent::ConstPtr &key_event)
{
    // store key state
    if(key_event->state)
        keys_pressed_list_.push_back(key_event->keycode);
    else
        keys_pressed_list_.erase(std::remove(keys_pressed_list_.begin(), keys_pressed_list_.end(), key_event->keycode), keys_pressed_list_.end());

    // process hotkeys
    bool ctrl_is_pressed = (std::find(keys_pressed_list_.begin(), keys_pressed_list_.end(), 37) != keys_pressed_list_.end());
    bool shift_is_pressed = (std::find(keys_pressed_list_.begin(), keys_pressed_list_.end(), 50) != keys_pressed_list_.end());
    bool alt_is_pressed = (std::find(keys_pressed_list_.begin(), keys_pressed_list_.end(), 64) != keys_pressed_list_.end());

    /*if(key_event->keycode == 12 && key_event->state && key_is_pressed != keys_pressed_list_.end()) // ctrl+3
    {
        if(this->isVisible())
        {
            this->hide();
        }
        else
        {
            this->move(QPoint(key_event->cursor_x+5, key_event->cursor_y+5));
            this->show();
        }
    }*/

    if(key_event->keycode == 26 && key_event->state && ctrl_is_pressed && shift_is_pressed)
            on_send_left_cartesian_button__clicked();
    else if(key_event->keycode == 27 && key_event->state && ctrl_is_pressed && shift_is_pressed)
        on_send_right_cartesian_button__clicked();
    else if(key_event->keycode == 26 && key_event->state && ctrl_is_pressed)
        on_send_left_configuration_button__clicked();
    else if(key_event->keycode == 27 && key_event->state && ctrl_is_pressed)
        on_send_right_configuration_button__clicked();
    else if(key_event->keycode == 41 && key_event->state && ctrl_is_pressed)
        on_send_upper_body_button__clicked();
    else if(key_event->keycode == 39 && key_event->state && ctrl_is_pressed)
        snapClicked();

    std::cout << "key code:" << key_event->keycode << std::endl;
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
    cmd.data = false;
    send_pelvis_pub_.publish(cmd);
}

void GhostControlWidget::on_pushButton_2_clicked()
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

void GhostControlWidget::on_send_template_to_behavior_button_clicked()
{
    //CHANGE TO CALL FOR STAND POSE INFO FROM TEMPLATE SERVER
//    unsigned int pose_index;
//    for(pose_index = 0; pose_index < pose_db_.size(); pose_index++)
//        if(pose_db_[pose_index].pose_id == selected_grasp_id_)
//            break;

//    if(pose_index == pose_db_.size()){
//        ROS_ERROR("Pose not found in database");
//    }
//    else
//    {
//        if(send_template_to_behavior_pub_)
//        {
//        geometry_msgs::PoseStamped pose;
//        pose.header.frame_id = "/world";
//        pose.header.stamp = ros::Time::now();
//        pose.pose = last_template_list_.pose[ui->templateBox->currentIndex()].pose;
//        send_template_to_behavior_pub_.publish(pose);
//        }
//        else{
//            ROS_ERROR("No Publisher for ghost to template pose");
//        }
//    }

}


void GhostControlWidget::on_send_ghost_to_template_button_clicked()
{

    //TO USE CERTAIN ARM GROUP
//    std_msgs::String cmd;

//    cmd.data = "l_arm_group";

//    set_to_target_pose_pub_.publish(cmd);

    //CHANGE TO CALL FOR STAND POSE INFO FROM TEMPLATE SERVER
//    unsigned int pose_index;
//    for(pose_index = 0; pose_index < pose_db_.size(); pose_index++)
//        if(pose_db_[pose_index].pose_id == selected_grasp_id_)
//            break;

//    if(pose_index == pose_db_.size()){
//        ROS_ERROR("Pose not found in database");
//    }
//    else
//    {
//        if(send_ghost_to_template_pub_)
//        {
//        geometry_msgs::PoseStamped pose;
//        pose.header.frame_id = "/world";
//        pose.header.stamp = ros::Time::now();
//        calcTargetPose(last_template_list_.pose[ui->templateBox->currentIndex()].pose,
//                       pose_db_[pose_index].ghost_pose,
//                       pose.pose);
//        send_ghost_to_template_pub_.publish(pose);
//        }
//        else{
//            ROS_ERROR("No Publisher for ghost to template pose");
//        }
//    }
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
