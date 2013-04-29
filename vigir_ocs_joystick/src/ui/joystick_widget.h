#ifndef JOYSTICK_WIDGET_H
#define JOYSTICK_WIDGET_H

#include <QWidget>

#include "../joystick.h"

namespace Ui {
class JoystickWidget;
}

class JoystickWidget : public QWidget
{
    Q_OBJECT
    
public:
    explicit JoystickWidget(QWidget *parent = 0);
    ~JoystickWidget();

public Q_SLOTS:
    void yawDialChanged();
    void yawDialReleased();
    void setProgressBar(unsigned char throttle);
    void throttleSliderMoved();
    void throttleSliderReleased();
    
private:
    Ui::JoystickWidget *ui;
    Joystick *joystick;
};

#endif // JOYSTICK_WIDGET_H
