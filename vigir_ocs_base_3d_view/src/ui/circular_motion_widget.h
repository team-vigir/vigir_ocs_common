#pragma once

#include <QWidget>
#include "cartesian_motion_widget.h"

namespace vigir_ocs {

struct CircularMotionSettings : public CartesianMotionSettings {
    double rotation_angle_rad;
};

class CircularMotionWidget : public CartesianMotionWidget {
    Q_OBJECT

public:
    CircularMotionWidget(QWidget *parent = NULL, Qt::WindowFlags flags = 0);
    void getMotionSettings(CircularMotionSettings &settings);

private:
    void setupWidgets();

    QDoubleSpinBox *rotation_angle_spin_;
};

}
