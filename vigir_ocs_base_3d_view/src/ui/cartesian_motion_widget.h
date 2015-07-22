#pragma once

#include <QWidget>
#include "ui_cartesian_motion_widget.h"

#include <geometry_msgs/Point.h>

class QDoubleSpinBox;

namespace vigir_ocs {

struct CartesianMotionSettings {
    bool keep_eef_orientation;
    bool use_collision_avoidance;

    std::string planner_id;
    double sample_rate;
    unsigned int orientation_type;
    geometry_msgs::Point target_link_axis;

    std::string planning_group;
    std::string target_link_name;

};

class CartesianMotionWidget : public QWidget, Ui::CartesianMotionWidgetUI {
    Q_OBJECT

public:
    CartesianMotionWidget(QWidget *parent = NULL, Qt::WindowFlags flags = 0);
    ~CartesianMotionWidget();

    void getMotionSettings(CartesianMotionSettings &settings);
    void setPlannerId(std::string planner_id);

public Q_SLOTS:
    void plannerSelected(int planner_index);
    void showAdvancedOptions(bool show);
    void updateTargetLinkAxisItemText();
    void startPlanning();

Q_SIGNALS:
    void sendMotionToArm();

protected:
    QVBoxLayout *getMainLayout();

private:
    void setupWidgets();
    void setupConnections();

    QTreeWidgetItem *sample_rate_item_;
    QDoubleSpinBox *sample_rate_spin_;

    QTreeWidgetItem *orientation_type_item_;
    QComboBox *orientation_type_combobox_;

    QTreeWidgetItem *target_link_axis_item_;
    QTreeWidgetItem *target_link_axis_x_item_;
    QDoubleSpinBox *target_link_axis_x_spin_;
    QTreeWidgetItem *target_link_axis_y_item_;
    QDoubleSpinBox *target_link_axis_y_spin_;
    QTreeWidgetItem *target_link_axis_z_item_;
    QDoubleSpinBox *target_link_axis_z_spin_;

    QTreeWidgetItem *planning_group_item_;
    QComboBox *planning_group_combobox_;

    std::string target_link_name_;


};

}
