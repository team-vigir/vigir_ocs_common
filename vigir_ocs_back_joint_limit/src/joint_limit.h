#ifndef joint_limit_H
#define joint_limit_H

#include <QWidget>
#include <QObject>
#include <QApplication>
#include <QBasicTimer>
#include <QSettings>
#include <QCloseEvent>
#include <QVBoxLayout>
#include <QSlider>
#include <QLabel>
#include <QCheckBox>

#include <vector>
#include <algorithm>

#include <ros/ros.h>
#include <ros/publisher.h>

#include <std_msgs/Int8.h>

#include <vigir_ocs_msgs/OCSKeyEvent.h>

#include "hotkey_manager.h"

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>

#define JOINT_SLIDER_UI_FACTOR 10000000.0

namespace Ui {
class joint_limit;
}


class JointSettingControls : public QObject
{
public:
  Q_OBJECT

public:

  JointSettingControls(QHBoxLayout* parent_layout,
                       std::string joint_name,
                       std::string joint_description,
                       const robot_model::RobotModel& robot_model)
  {
    const robot_model::JointModel* jm = robot_model.getJointModel(joint_name);

    if (!jm){
      ROS_ERROR("Joint %s not part of robot, cannot generate controls for it!", joint_name.c_str());
      return;
    }

    this->joint_name_ = joint_name;
    this->joint_description_ = joint_description;
    this->joint_index_ = jm->getJointIndex();

    main_layout_ = new QVBoxLayout();
    slider_h_box_layout_ = new QHBoxLayout();
    vert_layout_min_ = new QVBoxLayout();
    vert_layout_max_ = new QVBoxLayout();

    description_label_ = new QLabel(QString(joint_description.c_str()));// + " (" + joint_name + ")"));
    min_label_= new QLabel("min");
    max_label_= new QLabel("max");
    min_val_label_= new QLabel("N/A");
    max_val_label_= new QLabel("N/A");

    lock_checkbox_ = new QCheckBox("Lock");
    lock_checkbox_->setCheckState(Qt::Unchecked);

    const robot_model::VariableBounds& bounds = jm->getVariableBounds(joint_name);

    min_val_slider_ = new QSlider(Qt::Vertical);
    max_val_slider_ = new QSlider(Qt::Vertical);

    min_val_slider_->setMaximum(bounds.max_position_ * JOINT_SLIDER_UI_FACTOR);
    min_val_slider_->setMinimum(bounds.min_position_ * JOINT_SLIDER_UI_FACTOR);

    max_val_slider_->setMaximum(bounds.max_position_ * JOINT_SLIDER_UI_FACTOR);
    max_val_slider_->setMinimum(bounds.min_position_ * JOINT_SLIDER_UI_FACTOR);



    //QLabel* min_val_label_QLabel* min_label_= new QLabel("min");
    //QLabel* max_val_label_;

    //main_layout_->addWidget();
    QObject::connect(min_val_slider_, SIGNAL(sliderReleased()),
                     this, SLOT(on_min_slider_released()));
    QObject::connect(max_val_slider_, SIGNAL(sliderReleased()),
                     this, SLOT(on_max_slider_released()));

    QObject::connect(lock_checkbox_, SIGNAL(toggled(bool)),
                     this, SLOT(on_lock_checkbox_toggled(bool)));

    parent_layout->addLayout(main_layout_);
    main_layout_->addWidget(description_label_);
    main_layout_->addWidget(lock_checkbox_);
    main_layout_->addLayout(slider_h_box_layout_);

    slider_h_box_layout_->addLayout(vert_layout_min_);
    slider_h_box_layout_->addLayout(vert_layout_max_);

    vert_layout_min_->addWidget(min_label_);
    vert_layout_min_->addWidget(min_val_slider_);
    vert_layout_min_->addWidget(min_val_label_);

    vert_layout_max_->addWidget(max_label_);
    vert_layout_max_->addWidget(max_val_slider_);
    vert_layout_max_->addWidget(max_val_label_);

    max_limit_ = bounds.max_position_;
    min_limit_ = bounds.min_position_;

    //Set UI to initial values
    max_val_ = bounds.max_position_;
    min_val_ = bounds.min_position_;
    max_val_slider_->setValue(max_val_*JOINT_SLIDER_UI_FACTOR);
    min_val_slider_->setValue(min_val_*JOINT_SLIDER_UI_FACTOR);
    max_val_label_->setText(QString::number(max_val_,'g',6));
    min_val_label_->setText(QString::number(min_val_,'g',6));

  };

  bool isLocked() const
  {
    return lock_checkbox_->isChecked();
  }

  int getJointIndex() const
  {
    return joint_index_;
  }

  //@TODO Looks a bit hacky but ok for now
  bool constraintsDifferFromModelBounds()
  {
    return ( std::fabs(max_limit_ - max_val_) > 0.001) || (std::fabs (min_limit_ - min_val_) > 0.001);
  }

  double getMax() const
  {
    return max_val_;
  }

  double getMin() const
  {
    return min_val_;
  }

public Q_SLOTS:

  void on_max_slider_released()
  {
      if(max_val_slider_->value() <= min_val_*JOINT_SLIDER_UI_FACTOR)
          max_val_slider_->setValue(max_val_*JOINT_SLIDER_UI_FACTOR);
      else
      {
          max_val_ = (double)max_val_slider_->value()/JOINT_SLIDER_UI_FACTOR;
          max_val_label_->setText(QString::number(max_val_,'g',6));
      }
  };

  void on_min_slider_released()
  {
      if(min_val_slider_->value() >= max_val_*JOINT_SLIDER_UI_FACTOR)
          min_val_slider_->setValue(min_val_*JOINT_SLIDER_UI_FACTOR);
      else
      {
          min_val_ = (double)min_val_slider_->value()/JOINT_SLIDER_UI_FACTOR;
          min_val_label_->setText(QString::number(min_val_,'g',6));
      }
  }

  void on_lock_checkbox_toggled(bool checked)
  {
    min_val_slider_->setEnabled(!checked);
    max_val_slider_->setEnabled(!checked);
  }

private:
  QVBoxLayout* main_layout_;
  QHBoxLayout* slider_h_box_layout_;
  QVBoxLayout* vert_layout_min_;
  QVBoxLayout* vert_layout_max_;

  QLabel* min_label_;
  QLabel* max_label_;
  QLabel* description_label_;
  QLabel* min_val_label_;
  QLabel* max_val_label_;

  QCheckBox* lock_checkbox_;
  QSlider* min_val_slider_;
  QSlider* max_val_slider_;

  std::string joint_name_;
  std::string joint_description_;
  int joint_index_;

  double max_val_;
  double min_val_;

  double max_limit_;
  double min_limit_;
};

class joint_limit : public QWidget
{
    Q_OBJECT

public:
    explicit joint_limit(QWidget *parent = 0);
    ~joint_limit();

    void processWindowControl(const std_msgs::Int8::ConstPtr& msg);

    //void processNewKeyEvent(const vigir_ocs_msgs::OCSKeyEvent::ConstPtr& pose);

private:
    ros::Subscriber window_control_sub;
    ros::Publisher window_control_pub;
    QRect geometry_;

    Ui::joint_limit *ui;
    ros::NodeHandle nh_;
    ros::Publisher constraints_pub_;

    QBasicTimer timer;

    std::vector<int> keys_pressed_list_;

    ros::Subscriber key_event_sub_;

    robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
    robot_model::RobotModelPtr robot_model_;

    std::vector <boost::shared_ptr<JointSettingControls> > joint_controls_;

public Q_SLOTS:
    void on_apply_clicked();

    //void on_Presets_comboBox_currentIndexChanged(int index);

protected Q_SLOTS:
    void timerEvent(QTimerEvent *event);
    void closeEvent(QCloseEvent *event);
    void resizeEvent(QResizeEvent * event);
    void moveEvent(QMoveEvent * event);
};

#endif // joint_limit_H
