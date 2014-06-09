#ifndef JOYSTICKWIDGET_H
#define JOYSTICKWIDGET_H

#include <QMainWindow>
#include <QString>
#include <QWidget>
#include <QtGui>
#include <QComboBox>
#include "Controller.h"
//#include "/opt/vigir/catkin_ws/src/vigir_ocs_common/vigir_ocs_template_joystick/src/Controller.h"

namespace Ui
{
    class JoystickWidget;
}

class JoystickWidget : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit JoystickWidget(QWidget *parent = 0);
    ~JoystickWidget();
protected:
    void keyPressEvent(QKeyEvent *);
    void keyReleaseEvent(QKeyEvent *);
private:
    Ui::JoystickWidget *ui;
    //handle multiple keys at once
    QSet<Qt::Key> keysPressed;
    void processKeys();
    vigir_ocs::Controller * controller;
    QSignalMapper * mapper;
    QString icon_path_;

public Q_SLOTS:
    void setDirection(QString s);
    void disableLeftLabel();
    void disableRightLabel();
    void selectTemplate();
    void populateTemplateComboBox(int tempId);
    void receiveCameraTransform(int viewId, float x, float y, float z, float rx, float ry, float rz, float w);
    void setManipulationMode(int);
};

#endif // JOYSTICKWIDGET_H
