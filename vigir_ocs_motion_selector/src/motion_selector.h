#ifndef MOTION_SELECTOR_H
#define MOTION_SELECTOR_H

#include <QMainWindow>
#include <QTreeWidgetItem>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <QBasicTimer>
#include <QDoubleSpinBox>
#include <QPushButton>
#include <QLayout>

namespace Ui {
class motion_selector;
}

class motion_selector : public QMainWindow
{
    Q_OBJECT

public:
    explicit motion_selector(QWidget *parent = 0);
    ~motion_selector();

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
public Q_SLOTS:
    void on_sendCommand_clicked();
    void on_timeFactorSlider_valueChanged(int value);
    void quickButtonClicked();
    void on_enableQuickButtons_clicked();

protected:
    void timerEvent(QTimerEvent *event);
};

#endif // MOTION_SELECTOR_H
