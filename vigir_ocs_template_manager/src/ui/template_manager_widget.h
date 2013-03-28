#ifndef TemplateManagerWidget_H
#define TemplateManagerWidget_H


#include <QMainWindow>
#include <QWidget>
#include <QRadioButton>
#include <QSpinBox>
#include <QComboBox>

#include <QPainter>
#include <QtGui>

#include <flor_ocs_msgs/OCSTemplateList.h>
#include <flor_ocs_msgs/OCSTemplateRemove.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>

namespace Ui {
class TemplateManagerWidget;
}

class TemplateManagerWidget : public QWidget
{
    Q_OBJECT
    
public:
    explicit TemplateManagerWidget(QWidget *parent = 0);
    ~TemplateManagerWidget();

    void processTemplateList(const flor_ocs_msgs::OCSTemplateList::ConstPtr& msg);
    void removeTemplate(int id);

public Q_SLOTS:
    void editSlot(int,int);
    
private:

    Ui::TemplateManagerWidget* ui;
    QString templateDirPath;
    QString templatePath;

    ros::NodeHandle nh_;
    ros::Subscriber template_list_sub_;
    ros::Publisher template_remove_pub_;
};

#endif // TemplateManagerWidget_H
