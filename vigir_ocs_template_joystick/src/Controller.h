/*
 * RobotModel class definition.
 * 
 * Author: Brian Wright
 * 
 * Based on librviz_tutorials and the .
 * 
 */

#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <ros/ros.h>
#include <QWidget>
#include <QQuaternion>
#include <flor_ocs_msgs/OCSTemplateList.h>
#include <flor_ocs_msgs/OCSTemplateUpdate.h>
#include <flor_ocs_msgs/OCSInteractiveMarkerUpdate.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JoyFeedbackArray.h>
#include <string>
#include <QBasicTimer>
#include <geometry_msgs/PoseStamped.h>
#include <flor_planning_msgs/TargetConfigIkRequest.h>

#include <QLabel>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QApplication>


namespace vigir_ocs
{
// Class "Main3DView" implements the RobotModel class with joint manipulation that can be added to any QT application.
class Controller: public QWidget
{
    Q_OBJECT
public:
    Controller( QWidget* parent = 0 );
    virtual ~Controller();
    void templateListCb(const flor_ocs_msgs::OCSTemplateList::ConstPtr& msg);
    void buildmsg(float posX, float posZ , float rotY, float rotX);
    void joyCB(const sensor_msgs::Joy::ConstPtr& msg);
    void leftCB(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void rightCB(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void changeTemplate();
    std::vector<std::string> getTemplateNames();

protected:
    ros::Subscriber template_list_sub;
    ros::Publisher template_update_pub;
    ros::Subscriber joy_sub;
    ros::Publisher joy_pub;
    ros::Subscriber left_sub;
    ros::Subscriber right_sub;
    ros::Publisher ghost_hand_pub;
    flor_ocs_msgs::OCSTemplateList temList;
    sensor_msgs::Joy joy;
    sensor_msgs::Joy oldJoy;
    geometry_msgs::PoseStamped leftHand;
    geometry_msgs::PoseStamped rightHand;
    void timerEvent(QTimerEvent *event);

private:
   ros::NodeHandle nh;
   QBasicTimer timer;
   int templateIndex;
   bool initialPublish;
   bool robotMode;
   void buildTransformation(float posX, float posY ,float posZ, float rotX, float rotY,float rotZ, QQuaternion* rotation, QVector3D* position);
   void handleJoystick();
   bool compareJoyData();
   void handleButtons();    
   void buildJoy();
   void handleArms();


Q_SIGNALS:
   void updateTemplateComboBox(int tempID);

public Q_SLOTS:
    void changeTemplateID(int newID);
    void leftModeToggle();

};
}
#endif // Controller_H
