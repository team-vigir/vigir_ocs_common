#pragma once

#include <QWidget>
#include "ui_circular_motion_widget.h"

#include <geometry_msgs/Point.h>

namespace vigir_ocs {

struct CircularMotionSettings {
    bool keep_eef_orientation;
    bool use_collision_avoidance;
    double rotation_angle_rad;

    std::string planner_id;
    double sample_rate;
    unsigned int orientation_type;
    geometry_msgs::Point target_link_axis;

    std::string planning_group;
    std::string target_link_name;
};

class CircularMotionWidget : public QWidget, Ui::CircularMotionWidgetUI {
    Q_OBJECT

public:
    CircularMotionWidget(QWidget *parent, Qt::WindowFlags flags);
    ~CircularMotionWidget();

    CircularMotionSettings getMotionSettings();
    void setPlannerId(std::string planner_id);

public Q_SLOTS:
    void plannerSelected(int planner_index);
    void showAdvancedOptions(bool show);
    void updateTargetLinkAxisItemText();

Q_SIGNALS:
    void sendMotionToLeftArm();
    void sendMotionToRightArm();

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

    QTreeWidgetItem *target_link_name_item_;
    QLineEdit *target_link_name_edit_;

    QTreeWidgetItem *planning_group_item_;
    QLineEdit *planning_group_edit_;
};

}
