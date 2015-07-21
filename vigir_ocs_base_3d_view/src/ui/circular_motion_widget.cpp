#include "circular_motion_widget.h"
#include <cmath>

#include <vigir_planning_msgs/ExtendedPlanningOptions.h>

namespace vigir_ocs {

CircularMotionWidget::CircularMotionWidget(QWidget *parent, Qt::WindowFlags flags) : QWidget(parent, flags) {
    setupUi(this);

    setupWidgets();
    setupConnections();
}

CircularMotionWidget::~CircularMotionWidget() {

}

CircularMotionSettings CircularMotionWidget::getMotionSettings() {
    CircularMotionSettings settings;
    settings.keep_eef_orientation = checkBox_KeepEndeffectorOrientation->isChecked();
    settings.use_collision_avoidance = checkBox_CollisionAvoidance->isChecked();

    settings.planner_id = lineEdit_CustomPlanner->text().toStdString();

    if ( settings.planner_id == "drake") {
        settings.sample_rate = sample_rate_spin_->value();
        settings.orientation_type = orientation_type_combobox_->itemData( orientation_type_combobox_->currentIndex()).toUInt();
        settings.target_link_axis.x = target_link_axis_x_spin_->value();
        settings.target_link_axis.y = target_link_axis_y_spin_->value();
        settings.target_link_axis.z = target_link_axis_z_spin_->value();

        settings.planning_group = planning_group_edit_->text().toStdString();
        settings.target_link_name = target_link_name_edit_->text().toStdString();
    }

    return settings;
}

void CircularMotionWidget::setPlannerId(std::string planner_id) {
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

void CircularMotionWidget::plannerSelected(int planner_index) {
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

void CircularMotionWidget::showAdvancedOptions(bool show) {
    frame_AdvancedOptions->setVisible(show);
    adjustSize();
}

void CircularMotionWidget::updateTargetLinkAxisItemText() {
    QString itemText = QString("[x = %1, y = %2, z = %3]")
            .arg(target_link_axis_x_spin_->value(), 0, 'f', 2)
            .arg(target_link_axis_y_spin_->value(), 0, 'f', 2)
            .arg(target_link_axis_z_spin_->value(), 0, 'f', 2);

    target_link_axis_item_->setText(1, itemText);
}

void CircularMotionWidget::setupWidgets() {
    plannerSelected(0);         // select default planner

    showAdvancedOptions(false); // hide advanced options

    // setup Drake options widget
    treeWidget_DrakeOptions->clear();
    target_link_name_item_ = new QTreeWidgetItem(treeWidget_DrakeOptions, QStringList() << "Target Link Name: " << "");
    target_link_name_edit_ = new QLineEdit(this);
    treeWidget_DrakeOptions->setItemWidget(target_link_name_item_, 1, target_link_name_edit_);

    planning_group_item_ = new QTreeWidgetItem(treeWidget_DrakeOptions, QStringList() << "Planning Group: " << "");
    planning_group_edit_ = new QLineEdit(this);
    treeWidget_DrakeOptions->setItemWidget(planning_group_item_, 1, planning_group_edit_);

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

void CircularMotionWidget::setupConnections() {
    connect(pushButton_SendToLeftArm,  SIGNAL(clicked()), this, SIGNAL(sendMotionToLeftArm()));
    connect(pushButton_SendToRightArm, SIGNAL(clicked()), this, SIGNAL(sendMotionToRightArm()));

    connect(target_link_axis_x_spin_, SIGNAL(valueChanged(double)), this, SLOT(updateTargetLinkAxisItemText()));
    connect(target_link_axis_y_spin_, SIGNAL(valueChanged(double)), this, SLOT(updateTargetLinkAxisItemText()));
    connect(target_link_axis_z_spin_, SIGNAL(valueChanged(double)), this, SLOT(updateTargetLinkAxisItemText()));
}

}
