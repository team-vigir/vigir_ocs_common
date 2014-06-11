#ifndef MAIN_VIEW_WIDGET_H
#define MAIN_VIEW_WIDGET_H

#include <QWidget>
#include <QPushButton>
#include <QSignalMapper>

#include <map>
#include <vector>
#include <algorithm>

#include <ros/ros.h>

#include <flor_ocs_msgs/OCSKeyEvent.h>
#include <std_msgs/Int8.h>

#include "statusBar.h"
#include "graspWidget.h"
#include <QSpacerItem>
#include <QBasicTimer>



namespace Ui
{
    class MainViewWidget;
}

class MainViewWidget : public QWidget
{
    Q_OBJECT
    
public:
    explicit MainViewWidget(QWidget *parent = 0);
    ~MainViewWidget();

    virtual bool eventFilter( QObject * o, QEvent * e );

    void processNewKeyEvent(const flor_ocs_msgs::OCSKeyEvent::ConstPtr& pose);

    // process window control messages to update toggle buttons
    void processWindowControl(const std_msgs::Int8::ConstPtr& visible);

    virtual void timerEvent(QTimerEvent *event);

public Q_SLOTS:
    void oneViewToggle();
    void fourViewToggle();
    void ft_sensorToggled(bool);
    void zero_leftPressed();
    void zero_rightPressed();
    void toggleWindow(int);
    void graspWidgetToggle();


private:
    void setupToolbar();
    void loadButtonIcon(QPushButton* btn, QString image_name);

    Ui::MainViewWidget *ui;

    std::map<std::string,QWidget*> views_list;

    QWidget* position_widget_;
    QPushButton* four_view_button_;
    QPushButton* one_view_button_;
    QSignalMapper* toggle_mapper_;

    QString icon_path_;

    std::vector<int> keys_pressed_list_;

    ros::NodeHandle n_;
    ros::Subscriber window_control_sub_;
    ros::Publisher window_control_pub_;
    ros::Subscriber key_event_sub_;
    ros::Publisher ft_zero_pub_;

    StatusBar * statusBar;    
    graspWidget * leftGraspWidget;
    graspWidget * rightGraspWidget;
    QPushButton* grasp_toggle_button_;
    QBasicTimer timer;

};

#endif // MAIN_VIEW_WIDGET_H
