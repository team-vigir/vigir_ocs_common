#ifndef MANIPULATION_TEST_H
#define MANIPULATION_TEST_H

#include <QElapsedTimer>
#include <QTextStream>
#include <time.h>
#include <ctime>
#include <QFile>
#include <QDir>
#include "ui/main_view_widget.h"
#include "flor_ocs_msgs/OCSTemplateAdd.h"
#include "flor_ocs_msgs/OCSTemplateRemove.h"
#include "flor_ocs_msgs/OCSTemplateList.h"
#include <ros/ros.h>
#include <QQuaternion>
#include <QVector3D>
#include <QKeyEvent>



class ManipulationTest : public MainViewWidget
{
    Q_OBJECT

public:
    explicit ManipulationTest(QWidget *parent = 0);
    ~ManipulationTest();

public Q_SLOTS:
    void newChallenge();

private:
    void addInteractiveMarker( const flor_ocs_msgs::OCSInteractiveMarkerAdd::ConstPtr& msg );
    void insertTemplate(QVector3D*  position, QQuaternion* rotation, QString);
    void defineTransforms();

    void templateListCb(const flor_ocs_msgs::OCSTemplateList::ConstPtr& msg);

    std::vector<QVector3D *> templatePositions;
    std::vector<QQuaternion *> templateRotations;
    std::vector<QString> templateNames;
    QVector3D * startingPosition;
    QQuaternion* startingRotation;

    QFile * results;

    QSet<Qt::Key> keysPressed;
    QPushButton* startChallenge;
    int challengeCount;
    QElapsedTimer * elapsedTimer;

    ros::Publisher template_add_pub_;
    ros::Publisher template_update_pub;
    ros::Publisher template_remove_pub_;
    ros::Subscriber template_list_sub;

    ros::Subscriber interactive_marker_sub;

    flor_ocs_msgs::OCSTemplateList temList;

    ros::NodeHandle n_;    


};

#endif // MANIPULATION_TEST_H
