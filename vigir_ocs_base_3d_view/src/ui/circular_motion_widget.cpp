#include "circular_motion_widget.h"
#include <cmath>

#include <QDoubleSpinBox>

namespace vigir_ocs {

CircularMotionWidget::CircularMotionWidget(QWidget *parent, Qt::WindowFlags flags) : CartesianMotionWidget(parent, flags) {
    setupWidgets();
}

void CircularMotionWidget::getMotionSettings(CircularMotionSettings &settings) {
    CartesianMotionWidget::getMotionSettings(settings);
    settings.rotation_angle_rad = rotation_angle_spin_->value() * M_PI / 180.0;
}

void CircularMotionWidget::setupWidgets() {
    QHBoxLayout *rotation_angle_layout = new QHBoxLayout();
    QLabel *rotation_angle_label_ = new QLabel(tr("Rotation: "), this);

    rotation_angle_spin_ = new QDoubleSpinBox(this);
    rotation_angle_spin_->setRange(-1080.0, 1080.0);
    rotation_angle_spin_->setSuffix("\260");
    rotation_angle_spin_->setDecimals(2);
    rotation_angle_spin_->setValue(0.0);

    rotation_angle_layout->addWidget(rotation_angle_label_);
    rotation_angle_layout->addWidget(rotation_angle_spin_, 1);
    getMainLayout()->insertLayout(3, rotation_angle_layout);
    adjustSize();
}

}
