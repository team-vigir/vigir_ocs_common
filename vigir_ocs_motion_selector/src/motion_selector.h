#ifndef MOTION_SELECTOR_H
#define MOTION_SELECTOR_H

#include <QMainWindow>
#include <QTreeWidgetItem>
#include <QBasicTimer>
#include <QDoubleSpinBox>
#include <QPushButton>
#include <QLayout>

#include <vector>
#include <algorithm>

#include <ros/publisher.h>
#include <ros/ros.h>

#include <vigir_ocs_msgs/OCSKeyEvent.h>
#include "hotkey_manager.h"

namespace Ui {
class motion_selector;
}

class motion_selector : public QMainWindow
{
    Q_OBJECT

public:
    explicit motion_selector(QWidget *parent = 0);
    ~motion_selector();

    //void processNewKeyEvent(const vigir_ocs_msgs::OCSKeyEvent::ConstPtr& pose);

private:
    typedef struct
    {
        QString name;
        std::vector<QString> primitives;
    } Group;
    typedef struct
    {
        QString name;
        std::vector<Group> groups;
    } Task;

    typedef struct
    {
        QDoubleSpinBox* timeFactor;
        QPushButton* button;
    } quickButton;

    std::vector<quickButton*> quickButtonList;
    Ui::motion_selector *ui;
    QString filePath;
    void processTextFile( QString path);
    void setupQuickButtons(QString path);
    void printTasks();
    void populateTree();

    ros::NodeHandle nh_;
    ros::Publisher message_pub_;
    float sliderVal;
    std::vector<Task> taskList;
    std::vector<QTreeWidgetItem*> treeItems;
    QBasicTimer timer;

    //std::vector<int> keys_pressed_list_;

    //ros::Subscriber key_event_sub_;

    //hotkey
    void addHotKeys();
    void toggleVisibilityHotkey();


public Q_SLOTS:
    void on_sendCommand_clicked();
    void on_timeFactorSlider_valueChanged(int value);
    void quickButtonClicked();
    void on_enableQuickButtons_clicked();

protected:
    void timerEvent(QTimerEvent *event);
};

#endif // MOTION_SELECTOR_H
