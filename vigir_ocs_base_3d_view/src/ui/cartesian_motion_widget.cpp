#include "cartesian_motion_widget.h"

#include <QDoubleSpinBox>
#include <vigir_planning_msgs/ExtendedPlanningOptions.h>

#include "robot_state_manager.h"
#include <vigir_ocs_robot_model/moveit_ocs_model.h>
#include <moveit/robot_model/robot_model.h>

namespace vigir_ocs {

CartesianMotionWidget::CartesianMotionWidget(QWidget *parent, Qt::WindowFlags flags) : QWidget(parent, flags) {
    setupUi(this);

    setupWidgets();
    setupConnections();
}

CartesianMotionWidget::~CartesianMotionWidget() {
    treeWidget_DrakeOptions->clear();
}

void CartesianMotionWidget::getMotionSettings(CartesianMotionSettings &settings) {    
    settings.keep_eef_orientation = checkBox_KeepEndeffectorOrientation->isChecked();
    settings.use_collision_avoidance = checkBox_CollisionAvoidance->isChecked();
    settings.free_motion = checkBox_FreeMotion->isChecked();

    settings.planner_id = lineEdit_CustomPlanner->text().toStdString();
    settings.target_link_name = target_link_name_;

    settings.reference_point.x = spinBox_ReferencePointX->value();
    settings.reference_point.y = spinBox_ReferencePointY->value();
    settings.reference_point.z = spinBox_ReferencePointZ->value();

    if ( settings.planner_id == "drake") {
        settings.sample_rate = sample_rate_spin_->value();
        settings.orientation_type = orientation_type_combobox_->itemData( orientation_type_combobox_->currentIndex()).toUInt();
        settings.target_link_axis.x = target_link_axis_x_spin_->value();
        settings.target_link_axis.y = target_link_axis_y_spin_->value();
        settings.target_link_axis.z = target_link_axis_z_spin_->value();

        settings.planning_group = planning_group_combobox_->currentText().toStdString();        
    }
    else if ( settings.planner_id == "default") {
        settings.planner_id = "";
        settings.sample_rate = 0.0;
        settings.orientation_type = 0;
        settings.target_link_axis.x = 0;
        settings.target_link_axis.y = 0;
        settings.target_link_axis.z = 0;

        settings.planning_group = planning_group_combobox_->currentText().toStdString();
    }
}

void CartesianMotionWidget::setPlannerId(std::string planner_id) {
    if( planner_id == "" || planner_id == "default") {
        comboBox_Planner->setCurrentIndex(0);
        groupBox_DrakeOptions->setEnabled(false);
    }
    else if ( planner_id == "drake" ) {
        comboBox_Planner->setCurrentIndex(1);
        groupBox_DrakeOptions->setEnabled(true);
    }
    else {
        comboBox_Planner->setCurrentIndex(2);
        lineEdit_CustomPlanner->setText(QString::fromStdString(planner_id));
        groupBox_DrakeOptions->setEnabled(false);
    }
}

void CartesianMotionWidget::plannerSelected(int planner_index) {
    QString planner_id = comboBox_Planner->currentText();
    groupBox_DrakeOptions->setEnabled(false);

    if(planner_id == "custom") {
        label_CustomPlanner->setEnabled(true);
        lineEdit_CustomPlanner->setEnabled(true);
    }
    else if (planner_id == "drake") {
        groupBox_DrakeOptions->setEnabled(true);
    }
    else {
        label_CustomPlanner->setEnabled(false);
        lineEdit_CustomPlanner->setEnabled(false);
    }

    comboBox_Planner->setCurrentIndex(planner_index);
    lineEdit_CustomPlanner->setText(planner_id);
}

void CartesianMotionWidget::showAdvancedOptions(bool show) {
    frame_AdvancedOptions->setVisible(show);
    adjustSize();
}

void CartesianMotionWidget::updateTargetLinkAxisItemText() {
    QString itemText = QString("[x = %1, y = %2, z = %3]")
            .arg(target_link_axis_x_spin_->value(), 0, 'f', 2)
            .arg(target_link_axis_y_spin_->value(), 0, 'f', 2)
            .arg(target_link_axis_z_spin_->value(), 0, 'f', 2);

    target_link_axis_item_->setText(1, itemText);
}

void CartesianMotionWidget::startPlanning() {
    QObject *sender_object = sender();
    if ( sender_object == pushButton_SendToLeftArm)  {
        target_link_name_ = "l_hand";
        Q_EMIT sendMotionToArm();
    }
    else if ( sender_object == pushButton_SendToRightArm ) {
        target_link_name_ = "r_hand";
        Q_EMIT sendMotionToArm();
    }

    target_link_name_ = "";
}

QVBoxLayout *CartesianMotionWidget::getMainLayout() {
    return layout_MainLayout;
}

void CartesianMotionWidget::setupWidgets() {
    plannerSelected(0);         // select default planner

    showAdvancedOptions(false); // hide advanced options

    // setup Drake options widget
    treeWidget_DrakeOptions->clear();

    planning_group_item_ = new QTreeWidgetItem(treeWidget_DrakeOptions, QStringList() << "Planning Group: " << "");
    planning_group_combobox_ = new QComboBox(this);
    MoveItOcsModel *robot_model = RobotStateManager::Instance()->getGhostRobotStateSingleton();
    std::vector<srdf::Model::Group> joint_groups = robot_model->getGroups();
    for ( int i = 0; i < joint_groups.size(); i++) {
        planning_group_combobox_->addItem( QString::fromStdString(joint_groups[i].name_));
    }
    int idx = planning_group_combobox_->findText("whole_body_group");
    planning_group_combobox_->setCurrentIndex(idx);
    treeWidget_DrakeOptions->setItemWidget(planning_group_item_, 1, planning_group_combobox_);

    sample_rate_item_ = new QTreeWidgetItem(treeWidget_DrakeOptions, QStringList() << "Sample Rate:" << "");
    sample_rate_spin_ = new QDoubleSpinBox(this);
    sample_rate_spin_->setRange(0.1, 60.0);
    sample_rate_spin_->setSuffix(" Hz");
    sample_rate_spin_->setDecimals(2);
    sample_rate_spin_->setValue(4.0);
    treeWidget_DrakeOptions->setItemWidget(sample_rate_item_, 1, sample_rate_spin_);

    orientation_type_item_ = new QTreeWidgetItem(treeWidget_DrakeOptions, QStringList() << "Orientation Type: " << "");
    orientation_type_combobox_ = new QComboBox(this);
    orientation_type_combobox_->addItem(tr("Full Orientation"), vigir_planning_msgs::ExtendedPlanningOptions::ORIENTATION_FULL);
    orientation_type_combobox_->addItem(tr("Axis Orientation"), vigir_planning_msgs::ExtendedPlanningOptions::ORIENTATION_AXIS_ONLY);
    orientation_type_combobox_->addItem(tr("Ignore Orientation"), vigir_planning_msgs::ExtendedPlanningOptions::ORIENTATION_IGNORE);
    treeWidget_DrakeOptions->setItemWidget(orientation_type_item_, 1, orientation_type_combobox_);

    target_link_axis_item_ = new QTreeWidgetItem(treeWidget_DrakeOptions, QStringList() << "Target Link Axis: " << "[x = 0.0, y = 0.0, z = 0.0]");

    target_link_axis_x_item_ = new QTreeWidgetItem(target_link_axis_item_, QStringList() << "x: " << "");
    target_link_axis_x_spin_ = new QDoubleSpinBox(this);
    target_link_axis_x_spin_->setRange(-1000.0, 1000.0);
    target_link_axis_x_spin_->setDecimals(2);
    target_link_axis_x_spin_->setValue(0.0);
    treeWidget_DrakeOptions->setItemWidget(target_link_axis_x_item_, 1, target_link_axis_x_spin_);

    target_link_axis_y_item_ = new QTreeWidgetItem(target_link_axis_item_, QStringList() << "y: " << "");
    target_link_axis_y_spin_ = new QDoubleSpinBox(this);
    target_link_axis_y_spin_->setRange(-1000.0, 1000.0);
    target_link_axis_y_spin_->setDecimals(2);
    target_link_axis_y_spin_->setValue(0.0);
    treeWidget_DrakeOptions->setItemWidget(target_link_axis_y_item_, 1, target_link_axis_y_spin_);

    target_link_axis_z_item_ = new QTreeWidgetItem(target_link_axis_item_, QStringList() << "z: " << "");
    target_link_axis_z_spin_ = new QDoubleSpinBox(this);
    target_link_axis_z_spin_->setRange(-1000.0, 1000.0);
    target_link_axis_z_spin_->setDecimals(2);
    target_link_axis_z_spin_->setValue(1.0);
    treeWidget_DrakeOptions->setItemWidget(target_link_axis_z_item_, 1, target_link_axis_z_spin_);

    updateTargetLinkAxisItemText();
    adjustSize();
}

void CartesianMotionWidget::setupConnections() {
    connect(target_link_axis_x_spin_, SIGNAL(valueChanged(double)), this, SLOT(updateTargetLinkAxisItemText()));
    connect(target_link_axis_y_spin_, SIGNAL(valueChanged(double)), this, SLOT(updateTargetLinkAxisItemText()));
    connect(target_link_axis_z_spin_, SIGNAL(valueChanged(double)), this, SLOT(updateTargetLinkAxisItemText()));
}

}
